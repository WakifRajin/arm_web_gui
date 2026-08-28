"""
arm_controller.py

All real motor logic lives here. No web/HTTP/websocket code in this
file at all -- it only depends on GimMotor / Joint / can_bus, exactly
like 09_control_gui.py's ArmController did. That boundary is kept on
purpose: the web layer (main.py / worker.py) can be swapped out again
later (ROS 2, a different frontend, whatever) without touching a
single line of motor logic.

============================================================
 DROP-IN REQUIREMENT -- READ THIS BEFORE RUNNING THE BACKEND
============================================================
This module imports three files that are NOT included here because
they already exist in your scripts/ package and contain your actual,
calibrated CAN protocol implementation (encoder scaling, soft-limit
enforcement, the 0xA0-0xCF command set):

    gim_motor.py   - low-level CAN protocol driver
    joint.py       - degree-based Joint class (calibration, limits)
    can_bus.py     - CAN adapter connection settings (get_bus())

Copy your existing copies of these three files into backend/ next to
this file (same layout 09_control_gui.py used) before starting the
server. See README.md.

Everything below only calls the same public methods
09_control_gui.py already called: motor.read_status(),
motor.clear_faults(), joint.get_angle(), joint.get_current(),
joint.enable(max_speed_rpm, max_current_a), joint.disable(),
joint.set_angle(deg), joint.set_home_here(). Nothing here guesses at
byte-level protocol details for position control.
"""

import json
import os
import struct
import time
from typing import Dict, List, Optional

import can

from gim_motor import GimMotor, GimTimeout
from joint import Joint
from can_bus import get_bus

from logs import log_event

# ---- raw discovery protocol, ported 1:1 from can_scanner.py --------------
PUBLIC_ADDR = 0xFF
CMD_VERSION = 0xA0
CMD_STATUS = 0xAE
CMD_SET_ADDR = 0xBA
MIN_PROTOCOL_FOR_0xBA = 0x08

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "gui_joints.json")
DEFAULT_CONFIG_PATH = os.path.join(HERE, "gui_joints.default.json")


class ArmController:
    def __init__(self, config_path: str = CONFIG_PATH):
        self.config_path = config_path
        self.config = self._load_config()
        self.bus = None
        self.joints: Dict[str, Joint] = {}
        self.motors: Dict[str, GimMotor] = {}
        self.discovered: Dict[int, dict] = {}  # addr -> {protocol, app, last_seen}
        self.enabled: set = set()  # names of joints whose torque is currently on

    # ---- config -----------------------------------------------------
    def _load_config(self):
        path = self.config_path if os.path.exists(self.config_path) else DEFAULT_CONFIG_PATH
        with open(path) as f:
            cfg = json.load(f)
        cfg.setdefault("poll_interval_s", 0.4)
        cfg.setdefault("scan_interval_s", 2.0)
        cfg.setdefault("max_current_a", 3.0)
        # CAN-interface-level failsafe settings (see can_diag.py) -- these
        # are OS/SocketCAN concerns, not motor protocol, so they live at
        # the top of the config rather than per-joint.
        cfg.setdefault("can_interface", "can0")
        cfg.setdefault("can_bitrate", 1000000)
        # If NO joint has answered a poll in this many seconds while at
        # least one joint has torque enabled, the worker treats it as a
        # dead/bus-off bus and forces an E-STOP on its own -- see
        # worker.py's bus watchdog.
        cfg.setdefault("bus_watchdog_s", 3.0)
        # Differential wrist mixing (two motors -> pitch/roll), purely a
        # software coordinate transform on top of the two motors' own
        # degree-based Joint.set_angle()/get_angle() -- no new CAN
        # protocol involved. Disabled by default: the sign convention
        # (pitch_sign_*/roll_sign_*) and mix_ratio depend on which way
        # your differential is physically assembled and MUST be verified
        # on the bench (torque off or low speed/current, small angles)
        # before trusting it, same philosophy as joint.py's calibrate().
        # Edit here, or use the Wrist tab's config panel which writes
        # back to this same block.
        cfg.setdefault("wrist_diff", {
            "enabled": False,
            "motor_a": "wrist_l",
            "motor_b": "wrist_r",
            # deg_a = mix_ratio * (pitch_sign_a*pitch + roll_sign_a*roll)
            # deg_b = mix_ratio * (pitch_sign_b*pitch + roll_sign_b*roll)
            "pitch_sign_a": 1,
            "roll_sign_a": 1,
            "pitch_sign_b": 1,
            "roll_sign_b": -1,
            "mix_ratio": 1.0,
            # If the wrist was assembled/wired so that the "pitch" input
            # physically moves it like roll (and vice versa), a sign flip
            # alone can't fix that -- it's an axis swap, not a direction
            # reversal. Flipping this exchanges pitch_deg/roll_deg before
            # they're fed to the sign/ratio matrix above (and un-swaps
            # them again for display), instead of relabeling anything in
            # the GUI.
            "swap_pitch_roll": False,
            # Commanded-axis soft limits, enforced here BEFORE mixing, on
            # top of (not instead of) each motor's own min_deg/max_deg.
            # These are the physical wrist's real range of motion.
            "pitch_min_deg": -90.0,
            "pitch_max_deg": 90.0,
            "roll_min_deg": -180.0,
            "roll_max_deg": 180.0,
        })
        for j in cfg["joints"]:
            j.setdefault("max_speed_rpm", 15)
            j.setdefault("max_accel_rpm_s", 30)
            j.setdefault("max_current_a", cfg["max_current_a"])
            # Position-loop / velocity-loop gains. Default to 0.0 ("unset")
            # rather than guessing a value -- these only ever get sent to
            # the motor when someone explicitly saves them from the Gains
            # panel, same philosophy as everything else in this file: no
            # invented protocol values.
            j.setdefault("pos_kp", 0.0)
            j.setdefault("pos_ki", 0.0)
            j.setdefault("vel_kp", 0.0)
            j.setdefault("vel_ki", 0.0)
        return cfg

    def save_config(self):
        with open(self.config_path, "w") as f:
            json.dump(self.config, f, indent=2)

    # ---- connection ---------------------------------------------------
    def connect(self):
        """Open the CAN bus and (re)build a Joint per configured motor.
        Safe to call again after a failure -- rebuilds everything."""
        self.bus = get_bus()
        self.joints = {}
        self.motors = {}
        for j in self.config["joints"]:
            motor = GimMotor(self.bus, dev_addr=j["addr"])
            joint = Joint(
                j["name"], motor,
                direction=j.get("direction", 1),
                min_deg=j["min_deg"], max_deg=j["max_deg"],
            )
            self.joints[j["name"]] = joint
            self.motors[j["name"]] = motor
        log_event("info", f"Connected to CAN bus, {len(self.joints)} joints configured.")

    # ---- per-joint status ----------------------------------------------
    def joint_config(self, name: str) -> dict:
        for j in self.config["joints"]:
            if j["name"] == name:
                return j
        raise KeyError(name)

    def poll_one(self, name: str) -> dict:
        """Read status/angle/current for one joint. Never raises --
        returns connected=False on any CAN-level failure instead, so
        one dead/unreachable joint never blocks the others or kills
        the worker thread. This is also how the frontend knows a
        motor was unplugged: connected flips to False on the next
        poll after it stops answering."""
        joint = self.joints[name]
        jcfg = self.joint_config(name)
        try:
            status = joint.motor.read_status()
            angle = joint.get_angle()
            current = joint.get_current()
            return {
                "name": name, "addr": jcfg["addr"], "connected": True, "angle": angle,
                "current": current, "fault_text": status.get("fault_text", "OK"),
                "voltage": status.get("voltage"), "temperature": status.get("temperature"),
                "min_deg": jcfg["min_deg"], "max_deg": jcfg["max_deg"],
                "max_speed_rpm": jcfg["max_speed_rpm"], "max_accel_rpm_s": jcfg["max_accel_rpm_s"],
                "max_current_a": jcfg["max_current_a"],
                "pos_kp": jcfg.get("pos_kp", 0.0), "pos_ki": jcfg.get("pos_ki", 0.0),
                "vel_kp": jcfg.get("vel_kp", 0.0), "vel_ki": jcfg.get("vel_ki", 0.0),
                "direction": jcfg.get("direction", 1),
                "enabled": name in self.enabled,
            }
        except (GimTimeout, can.CanError) as e:
            return {
                "name": name, "addr": jcfg["addr"], "connected": False, "angle": None,
                "current": None, "fault_text": f"NO REPLY ({e.__class__.__name__})",
                "voltage": None, "temperature": None,
                "min_deg": jcfg["min_deg"], "max_deg": jcfg["max_deg"],
                "max_speed_rpm": jcfg["max_speed_rpm"], "max_accel_rpm_s": jcfg["max_accel_rpm_s"],
                "max_current_a": jcfg["max_current_a"],
                "pos_kp": jcfg.get("pos_kp", 0.0), "pos_ki": jcfg.get("pos_ki", 0.0),
                "vel_kp": jcfg.get("vel_kp", 0.0), "vel_ki": jcfg.get("vel_ki", 0.0),
                "direction": jcfg.get("direction", 1),
                "enabled": name in self.enabled,
            }

    # ---- commands --------------------------------------------------
    def enable(self, name: str):
        """Engage torque by holding the joint's CURRENT position --
        never jumps to a new angle just because torque turned on."""
        joint = self.joints[name]
        jcfg = self.joint_config(name)
        joint.enable(jcfg["max_speed_rpm"], jcfg["max_current_a"])
        current_angle = joint.get_angle()
        joint.set_angle(current_angle)
        self.enabled.add(name)
        log_event("info", f"{name}: enabled (holding {current_angle:.2f} deg)", joint=name)

    def disable(self, name: str):
        self.joints[name].disable()
        self.enabled.discard(name)
        log_event("info", f"{name}: disabled", joint=name)

    def disable_all(self):
        for name, joint in self.joints.items():
            try:
                joint.disable()
            except GimTimeout:
                pass  # already unreachable -- nothing more to do from here
        self.enabled.clear()
        log_event("warning", "EMERGENCY STOP: all joints disabled.")

    def set_home(self, name: str):
        self.joints[name].set_home_here()
        log_event("warning", f"{name}: home/origin set here (permanent, stored in motor)", joint=name)

    def go_to_angle(self, name: str, deg: float) -> float:
        """Returns the actual (possibly clamped) angle that was sent.
        Used both for direct commands and as the low-level primitive
        the ramp tick calls every cycle."""
        return self.joints[name].set_angle(deg)

    def clear_fault(self, name: str):
        self.joints[name].motor.clear_faults()
        log_event("info", f"{name}: fault cleared", joint=name)

    def set_limits(self, name: str, min_deg: float, max_deg: float):
        if min_deg >= max_deg:
            raise ValueError("min_deg must be < max_deg")
        joint = self.joints[name]
        joint.min_deg = min_deg
        joint.max_deg = max_deg
        self.joint_config(name)["min_deg"] = min_deg
        self.joint_config(name)["max_deg"] = max_deg
        self.save_config()
        log_event("info", f"{name}: angle limits set to {min_deg}..{max_deg} deg", joint=name)

    # ---- differential wrist (pitch/roll <-> two motor angles) ----------
    def _wrist_coeffs(self):
        wd = self.config.get("wrist_diff") or {}
        if not wd.get("enabled"):
            raise ValueError("differential wrist is disabled (enable it on the Wrist tab first)")
        a, b = wd.get("motor_a"), wd.get("motor_b")
        if a not in self.joints or b not in self.joints:
            raise ValueError(f"wrist_diff motors '{a}'/'{b}' are not both configured joints")
        ratio = wd.get("mix_ratio", 1.0)
        pa, ra = wd.get("pitch_sign_a", 1), wd.get("roll_sign_a", 1)
        pb, rb = wd.get("pitch_sign_b", 1), wd.get("roll_sign_b", -1)
        det = pa * rb - ra * pb
        if abs(det) < 1e-9 or abs(ratio) < 1e-9:
            raise ValueError("wrist_diff sign/ratio configuration is degenerate -- pitch and roll "
                              "would be indistinguishable (check the Wrist config panel)")
        swap = bool(wd.get("swap_pitch_roll", False))
        pitch_min = wd.get("pitch_min_deg", -90.0)
        pitch_max = wd.get("pitch_max_deg", 90.0)
        roll_min = wd.get("roll_min_deg", -180.0)
        roll_max = wd.get("roll_max_deg", 180.0)
        return a, b, ratio, pa, ra, pb, rb, det, swap, pitch_min, pitch_max, roll_min, roll_max

    @staticmethod
    def _clamp_axis(value: float, lo: float, hi: float, axis_name: str) -> float:
        clamped = max(lo, min(hi, value))
        if clamped != value:
            log_event("warning", f"Wrist {axis_name}: requested {value:.2f} deg clamped "
                                  f"to configured limit {clamped:.2f} deg")
        return clamped

    def wrist_mix(self, pitch_deg: float, roll_deg: float) -> Dict[str, object]:
        """Convert a desired wrist pitch/roll into the two differential
        motor angles, per the sign/ratio convention in config['wrist_diff'].

        Two limits are enforced, in this order:
          1. The configured pitch/roll soft limits (pitch_min/max_deg,
             roll_min/max_deg) -- the wrist's own real range of motion.
          2. Each motor's own min_deg/max_deg. Because pitch and roll are
             BOTH mixed onto BOTH motors, a combination that is legal on
             each axis individually (e.g. full roll at some non-zero
             pitch) can still ask a motor to travel further than it
             physically can. Rather than letting each motor clamp
             independently (which distorts the pitch:roll ratio into
             something that was never asked for), both motor targets are
             scaled back together by the same factor so the commanded
             DIRECTION of wrist motion is preserved and only the
             magnitude is reduced to what's actually reachable.

        Returns {motor_a: deg, motor_b: deg, "pitch_deg": achieved_pitch,
        "roll_deg": achieved_roll, "was_scaled": bool} -- "pitch_deg"/
        "roll_deg" in the result are what will actually be achieved
        (after any clamping/scaling), not necessarily what was asked for.
        """
        a, b, ratio, pa, ra, pb, rb, _, swap, pitch_min, pitch_max, roll_min, roll_max = self._wrist_coeffs()

        pitch_deg = self._clamp_axis(pitch_deg, pitch_min, pitch_max, "pitch")
        roll_deg = self._clamp_axis(roll_deg, roll_min, roll_max, "roll")

        eff_pitch, eff_roll = (roll_deg, pitch_deg) if swap else (pitch_deg, roll_deg)
        deg_a = ratio * (pa * eff_pitch + ra * eff_roll)
        deg_b = ratio * (pb * eff_pitch + rb * eff_roll)

        # Scale both motor targets down together (never up) so neither
        # exceeds its own configured travel, preserving the ratio between
        # the two -- i.e. preserving the direction of wrist motion.
        joint_a, joint_b = self.joints[a], self.joints[b]
        scale = 1.0
        for deg, joint in ((deg_a, joint_a), (deg_b, joint_b)):
            if deg > joint.max_deg and deg > 1e-9:
                scale = min(scale, joint.max_deg / deg)
            elif deg < joint.min_deg and deg < -1e-9:
                scale = min(scale, joint.min_deg / deg)
        scale = max(0.0, scale)
        was_scaled = scale < 1.0 - 1e-9

        deg_a *= scale
        deg_b *= scale
        eff_pitch *= scale
        eff_roll *= scale
        achieved_pitch, achieved_roll = (eff_roll, eff_pitch) if swap else (eff_pitch, eff_roll)

        if was_scaled:
            log_event("warning",
                       f"Wrist: requested pitch {pitch_deg:.2f}/roll {roll_deg:.2f} deg exceeds "
                       f"{a}/{b} travel at this combination -- scaled to pitch {achieved_pitch:.2f}/"
                       f"roll {achieved_roll:.2f} deg (direction preserved, magnitude reduced)")

        return {a: deg_a, b: deg_b, "pitch_deg": achieved_pitch, "roll_deg": achieved_roll,
                "was_scaled": was_scaled}

    def wrist_unmix(self, deg_a: Optional[float], deg_b: Optional[float]) -> Optional[tuple]:
        """Inverse of wrist_mix() -- recover (pitch, roll) from the two
        motors' current angles, for display only. Returns None if the
        wrist isn't configured/enabled or either angle is unknown
        (e.g. that motor isn't currently connected)."""
        if deg_a is None or deg_b is None:
            return None
        try:
            _, _, ratio, pa, ra, pb, rb, det, swap, *_ = self._wrist_coeffs()
        except ValueError:
            return None
        x_a, x_b = deg_a / ratio, deg_b / ratio
        eff_pitch = (x_a * rb - x_b * ra) / det
        eff_roll = (x_b * pa - x_a * pb) / det
        pitch, roll = (eff_roll, eff_pitch) if swap else (eff_pitch, eff_roll)
        return pitch, roll

    def set_wrist_diff_config(self, enabled: bool, motor_a: str, motor_b: str,
                               pitch_sign_a: int, roll_sign_a: int,
                               pitch_sign_b: int, roll_sign_b: int,
                               mix_ratio: float, swap_pitch_roll: bool = False,
                               pitch_min_deg: float = -90.0, pitch_max_deg: float = 90.0,
                               roll_min_deg: float = -180.0, roll_max_deg: float = 180.0):
        if motor_a == motor_b:
            raise ValueError("motor_a and motor_b must be different joints")
        if motor_a not in self.joints or motor_b not in self.joints:
            raise ValueError(f"'{motor_a}' and '{motor_b}' must both be configured joints")
        for s in (pitch_sign_a, roll_sign_a, pitch_sign_b, roll_sign_b):
            if s not in (1, -1):
                raise ValueError("signs must be 1 or -1")
        if abs(mix_ratio) < 1e-9:
            raise ValueError("mix_ratio can't be zero")
        det = pitch_sign_a * roll_sign_b - roll_sign_a * pitch_sign_b
        if abs(det) < 1e-9:
            raise ValueError("that sign combination can't distinguish pitch from roll -- "
                              "pitch_sign_a/roll_sign_b and roll_sign_a/pitch_sign_b can't cancel out")
        if pitch_min_deg >= pitch_max_deg:
            raise ValueError("pitch_min_deg must be < pitch_max_deg")
        if roll_min_deg >= roll_max_deg:
            raise ValueError("roll_min_deg must be < roll_max_deg")
        self.config["wrist_diff"] = {
            "enabled": enabled, "motor_a": motor_a, "motor_b": motor_b,
            "pitch_sign_a": pitch_sign_a, "roll_sign_a": roll_sign_a,
            "pitch_sign_b": pitch_sign_b, "roll_sign_b": roll_sign_b,
            "mix_ratio": mix_ratio, "swap_pitch_roll": bool(swap_pitch_roll),
            "pitch_min_deg": pitch_min_deg, "pitch_max_deg": pitch_max_deg,
            "roll_min_deg": roll_min_deg, "roll_max_deg": roll_max_deg,
        }
        self.save_config()
        log_event("info", f"Differential wrist config saved: {motor_a}/{motor_b}, "
                           f"enabled={enabled}, ratio={mix_ratio}, swap={bool(swap_pitch_roll)}, "
                           f"pitch=[{pitch_min_deg},{pitch_max_deg}], roll=[{roll_min_deg},{roll_max_deg}]")

    def set_direction(self, name: str, direction: int):
        """Flip a configured joint's direction (+1/-1) -- for when a
        motor was wired/mounted so that a positive command moves the
        wrong way. Same convention as joint.py's Joint.direction and
        the Add-as-joint form, just editable on an existing joint
        instead of only settable at creation.

        Requires torque OFF: this flips the sign used by BOTH
        set_angle() (deg->counts) and get_angle() (counts->deg), so
        doing it while a ramp is actively driving the joint would mean
        the very next tick computes a delta against a target that no
        longer means what it did a moment ago. Re-enabling afterward
        re-seeds the ramp's target/commanded from a fresh get_angle()
        read (see ArmController.enable()), so nothing is left stale."""
        if direction not in (1, -1):
            raise ValueError("direction must be 1 or -1")
        if name in self.enabled:
            raise ValueError(f"{name}: disable torque before reversing direction "
                              f"(then re-enable to re-seed the target)")
        joint = self.joints[name]
        if joint.direction == direction:
            return
        joint.direction = direction
        self.joint_config(name)["direction"] = direction
        self.save_config()
        log_event("warning",
                   f"{name}: direction reversed to {direction} "
                   f"({'normal' if direction == 1 else 'reversed'})", joint=name)

    def set_speed_limits(self, name: str, max_speed_rpm: float, max_accel_rpm_s: float, max_current_a: float):
        jcfg = self.joint_config(name)
        jcfg["max_speed_rpm"] = max_speed_rpm
        jcfg["max_accel_rpm_s"] = max_accel_rpm_s
        jcfg["max_current_a"] = max_current_a
        self.save_config()
        if name in self.enabled:
            # Previously this only took effect the *next* time someone
            # toggled torque on -- a mid-session tune silently did nothing
            # until you cycled Enable/Disable, which looked like "the
            # speed/current controls don't work". Re-issue enable() now
            # with the new numbers so a live tune actually takes effect
            # immediately, same as the firmware call enable() already makes.
            self.joints[name].enable(max_speed_rpm, max_current_a)
            log_event("info", f"{name}: speed/current limits re-applied live (torque already on)", joint=name)
        log_event("info",
                   f"{name}: speed limits set to {max_speed_rpm} rpm, "
                   f"accel {max_accel_rpm_s} rpm/s, current {max_current_a} A", joint=name)

    def set_position_gains(self, name: str, kp: float, ki: float):
        """Position-loop (angle) proportional/integral gains. Requires
        joint.py to implement `set_position_gains(kp, ki)` -- NOT included
        here for the same reason gim_motor.py/joint.py aren't (see the
        module docstring): we don't know your motor's byte-level gain
        registers, only that you have some, so this only ever calls
        through to your own driver. Add the method to your Joint class
        (it should write the gains over CAN, same drop-in pattern as
        set_angle()/enable()/set_home_here()) and this starts working."""
        joint = self.joints[name]
        if not hasattr(joint, "set_position_gains"):
            raise AttributeError(
                "joint.py has no set_position_gains(kp, ki) method yet. Add one "
                "(same drop-in pattern as set_angle()) -- see README, 'Tuning gains'."
            )
        joint.set_position_gains(kp, ki)
        jcfg = self.joint_config(name)
        jcfg["pos_kp"], jcfg["pos_ki"] = kp, ki
        self.save_config()
        log_event("info", f"{name}: position gains set to Kp={kp}, Ki={ki}", joint=name)

    def set_velocity_gains(self, name: str, kp: float, ki: float):
        """Velocity-loop proportional/integral gains -- see
        set_position_gains() above; same drop-in requirement applies to
        joint.py's set_velocity_gains(kp, ki)."""
        joint = self.joints[name]
        if not hasattr(joint, "set_velocity_gains"):
            raise AttributeError(
                "joint.py has no set_velocity_gains(kp, ki) method yet. Add one "
                "(same drop-in pattern as set_angle()) -- see README, 'Tuning gains'."
            )
        joint.set_velocity_gains(kp, ki)
        jcfg = self.joint_config(name)
        jcfg["vel_kp"], jcfg["vel_ki"] = kp, ki
        self.save_config()
        log_event("info", f"{name}: velocity gains set to Kp={kp}, Ki={ki}", joint=name)

    def add_joint(self, name: str, addr: int, direction: int, min_deg: float, max_deg: float,
                  max_speed_rpm: float, max_accel_rpm_s: float, max_current_a: float):
        if any(j["name"] == name for j in self.config["joints"]):
            raise ValueError(f"joint '{name}' already exists")
        if any(j["addr"] == addr for j in self.config["joints"]):
            raise ValueError(f"address {addr} is already assigned to another joint")
        self.config["joints"].append({
            "name": name, "addr": addr, "direction": direction,
            "min_deg": min_deg, "max_deg": max_deg,
            "max_speed_rpm": max_speed_rpm, "max_accel_rpm_s": max_accel_rpm_s,
            "max_current_a": max_current_a,
            "pos_kp": 0.0, "pos_ki": 0.0, "vel_kp": 0.0, "vel_ki": 0.0,
        })
        self.save_config()
        motor = GimMotor(self.bus, dev_addr=addr)
        self.joints[name] = Joint(name, motor, direction=direction, min_deg=min_deg, max_deg=max_deg)
        self.motors[name] = motor
        log_event("info", f"Added joint '{name}' at address {addr}.", joint=name)

    def remove_joint(self, name: str):
        self.config["joints"] = [j for j in self.config["joints"] if j["name"] != name]
        self.save_config()
        self.joints.pop(name, None)
        self.motors.pop(name, None)
        log_event("info", f"Removed joint '{name}' from configuration.")

    # ---- raw broadcast discovery (ported from can_scanner.py) --------
    def scan_broadcast(self, window_s: float = 0.5) -> List[dict]:
        """Pings the broadcast address and listens for replies, exactly
        like can_scanner.py's _execute_scan(). Must only be called from
        the single thread that owns self.bus (the Worker loop), so it
        never races with normal joint polling on the same bus."""
        if not self.bus:
            return []
        configured_addrs = {j["addr"] for j in self.config["joints"]}
        try:
            for cmd in (CMD_VERSION, CMD_STATUS):
                msg = can.Message(arbitration_id=PUBLIC_ADDR, data=bytes([cmd]), is_extended_id=False)
                self.bus.send(msg, timeout=0.05)

            end_time = time.monotonic() + window_s
            while time.monotonic() < end_time:
                m = self.bus.recv(timeout=max(0.0, end_time - time.monotonic()))
                if m is None:
                    continue
                dev = m.arbitration_id
                d = bytes(m.data)
                if not (1 <= dev <= 254) or not d:
                    continue
                entry = self.discovered.setdefault(dev, {"addr": dev})
                entry["last_seen"] = time.time()
                if d[0] == CMD_VERSION and len(d) >= 8:
                    entry["app_fw"] = int.from_bytes(d[3:5], "little")
                    entry["protocol"] = d[7]
                elif d[0] == CMD_STATUS and len(d) >= 8:
                    entry["voltage"] = int.from_bytes(d[1:3], "little") * 0.01
                    entry["temperature"] = d[5]
                    entry["fault"] = d[7]
        except can.CanOperationError:
            pass
        except Exception as e:
            log_event("error", f"scan error: {e}")

        # Only report devices not already claimed by a configured joint --
        # those already show up as normal per-joint rows.
        now = time.time()
        fresh = [v for a, v in self.discovered.items()
                 if a not in configured_addrs and now - v.get("last_seen", 0) < 30]
        return fresh

    def assign_address(self, old_addr: int, new_addr: int):
        """CAN-native address (re)assignment, ported from
        can_scanner.py's assign_id(). Per the README, RS485 +
        ZE300_GUI is still the primary/recommended way to address a
        motor -- this is the CAN-only path documented there as
        something to confirm against protocol v3.09b0 before relying
        on it. Requires protocol >= 0x08 and exactly this one device
        reachable to avoid an ambiguous multi-device write."""
        if not (1 <= new_addr <= 254):
            raise ValueError("new address must be 1..254")
        entry = self.discovered.get(old_addr)
        if entry is None or "protocol" not in entry:
            raise ValueError(f"no version info for device {old_addr}; run a scan first")
        if entry["protocol"] < MIN_PROTOCOL_FOR_0xBA:
            raise ValueError(f"device protocol 0x{entry['protocol']:02X} is below 0x{MIN_PROTOCOL_FOR_0xBA:02X}; "
                              f"use the RS485 ZE300_GUI to set this address instead")
        msg = can.Message(arbitration_id=old_addr, data=bytes([CMD_SET_ADDR, new_addr]), is_extended_id=False)
        self.bus.send(msg, timeout=0.05)
        ack = False
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            m = self.bus.recv(timeout=0.1)
            if m and bytes(m.data) and bytes(m.data)[0] == CMD_SET_ADDR:
                ack = True
                break
        if ack:
            log_event("info", f"Device {old_addr} acknowledged address change to {new_addr}. "
                               f"Power-cycle the motor to finalize.")
        else:
            log_event("warning", f"Address-change frame sent to {old_addr} -> {new_addr}, no ack received.")
        return ack
