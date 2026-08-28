"""
worker.py

Background thread that owns the CAN bus and is the ONLY thing that
ever touches it, so two joints (or a jog tick and a typed "go to")
can never send frames on top of each other. Everything else --
the FastAPI/websocket layer in main.py -- only ever talks to this
thread through two queues:

    cmd_q    <- commands from the browser (enable, go_to_angle, jog...)
    status_q -> live status snapshots, discovery results, log info

The E-STOP path is intentionally NOT a normal queued command: main.py
sets `worker.estop_event` directly, and the loop checks it first,
every single iteration, before doing anything else -- so a panic
click is never stuck behind a backlog of other commands.
"""

import queue
import threading
import time

import can

from arm_controller import ArmController
from gim_motor import GimTimeout
from motion import RampedJoint
from logs import log_event


class Worker(threading.Thread):
    def __init__(self, arm: ArmController, cmd_q: "queue.Queue", status_q: "queue.Queue"):
        super().__init__(daemon=True)
        self.arm = arm
        self.cmd_q = cmd_q
        self.status_q = status_q
        self._stop = threading.Event()
        self.estop_event = threading.Event()
        self.ramps: dict[str, RampedJoint] = {}
        self.jog_active: dict[str, int] = {}  # name -> direction (-1, 0, 1) while a jog key is held

    def stop(self):
        self._stop.set()

    # ---- lifecycle -----------------------------------------------------
    def run(self):
        try:
            self.arm.connect()
            self._rebuild_ramps()
        except Exception as e:
            self.status_q.put({"type": "connect_error", "error": str(e)})
            return

        poll_interval = self.arm.config.get("poll_interval_s", 0.4)
        scan_interval = self.arm.config.get("scan_interval_s", 2.0)
        bus_watchdog_s = self.arm.config.get("bus_watchdog_s", 3.0)
        tick_dt = 0.05
        last_poll = 0.0
        last_scan = 0.0
        last_tick = time.monotonic()
        last_any_connected = time.monotonic()

        while not self._stop.is_set():
            try:
                if self.estop_event.is_set():
                    self._handle_estop()

                while True:
                    try:
                        cmd = self.cmd_q.get_nowait()
                    except queue.Empty:
                        break
                    self._handle_command(cmd)

                now = time.monotonic()
                if now - last_tick >= tick_dt:
                    self._tick_ramps(now - last_tick)
                    last_tick = now

                if time.time() - last_poll >= poll_interval:
                    any_connected = False
                    polled_angles: dict[str, float | None] = {}
                    for name in self.arm.joints:
                        result = self.arm.poll_one(name)
                        polled_angles[name] = result["angle"]
                        if result["connected"] and result["angle"] is not None:
                            any_connected = True
                            self.ramps[name].seed(result["angle"])
                        elif name in self.ramps:
                            # Comms lost to this joint -- freeze its ramp
                            # exactly where it is rather than letting it
                            # keep computing (and trying to send) toward a
                            # target the motor hasn't heard about in a
                            # while. Cheap to call every poll; it's a
                            # no-op once already frozen.
                            self.ramps[name].stop_in_place()
                        result["type"] = "status"
                        result["target_angle"] = self.ramps[name].target if name in self.ramps else None
                        result["cmd_velocity_deg_s"] = self.ramps[name].velocity if name in self.ramps else None
                        result["at_target"] = self.ramps[name].at_target if name in self.ramps else None
                        self.status_q.put(result)
                    last_poll = time.time()

                    # Differential wrist: report live pitch/roll (derived
                    # from the two motors' current angles) and the ramp's
                    # target pitch/roll, purely for display -- see
                    # ArmController.wrist_unmix().
                    wd = self.arm.config.get("wrist_diff") or {}
                    if wd.get("enabled"):
                        a, b = wd.get("motor_a"), wd.get("motor_b")
                        actual = self.arm.wrist_unmix(polled_angles.get(a), polled_angles.get(b))
                        target_a = self.ramps[a].target if a in self.ramps else None
                        target_b = self.ramps[b].target if b in self.ramps else None
                        target = self.arm.wrist_unmix(target_a, target_b)
                        self.status_q.put({
                            "type": "wrist_status",
                            "pitch": actual[0] if actual else None,
                            "roll": actual[1] if actual else None,
                            "target_pitch": target[0] if target else None,
                            "target_roll": target[1] if target else None,
                        })

                    if any_connected:
                        last_any_connected = time.monotonic()
                    elif self.arm.enabled and (time.monotonic() - last_any_connected) > bus_watchdog_s:
                        # BUS WATCHDOG: nothing has answered in a while and
                        # at least one joint still thinks its torque is on
                        # -- treat this as a dead/bus-off link and force
                        # the same E-STOP a human would hit, instead of
                        # quietly polling into the void.
                        log_event("error", f"BUS WATCHDOG: no motor has responded in over "
                                            f"{bus_watchdog_s:.1f}s while torque was enabled -- forcing E-STOP.")
                        self.estop_event.set()
                        last_any_connected = time.monotonic()  # don't re-trigger every cycle

                if time.time() - last_scan >= scan_interval:
                    discovered = self.arm.scan_broadcast(window_s=0.3)
                    self.status_q.put({"type": "discovered", "devices": discovered})
                    last_scan = time.time()

            except Exception as e:
                self.status_q.put({"type": "info", "text": f"worker loop error: {e}"})
                log_event("error", f"worker loop error: {e}")

            time.sleep(0.005)

    def _rebuild_ramps(self):
        # Joint.set_angle() works in output-shaft degrees already (per
        # joint.py / the README's calibration step), while max_speed_rpm
        # is an RPM figure that's more intuitive to tune from the GUI.
        # 1 RPM = 6 deg/s, so max_speed_deg_s = max_speed_rpm * 6.
        self.ramps = {}
        for j in self.arm.config["joints"]:
            self.ramps[j["name"]] = RampedJoint(
                name=j["name"], min_deg=j["min_deg"], max_deg=j["max_deg"],
                max_speed_deg_s=j["max_speed_rpm"] * 6.0,
                max_accel_deg_s2=j["max_accel_rpm_s"] * 6.0,
            )

    # ---- E-STOP ----------------------------------------------------------
    def _handle_estop(self):
        self.arm.disable_all()
        for r in self.ramps.values():
            r.stop_in_place()
        self.jog_active = {}
        self.status_q.put({"type": "estop_ack"})
        self.estop_event.clear()

    # ---- ramp tick -------------------------------------------------------
    def _tick_ramps(self, dt: float):
        # Continuous jog: turn a held direction into a moving target,
        # same soft-start/stop idea as 09_control_gui.py's velocity jog,
        # just driven centrally here instead of per-GUI-widget.
        for name, direction in list(self.jog_active.items()):
            if direction == 0 or name not in self.ramps or name not in self.arm.enabled:
                continue
            step_deg = direction * self.ramps[name].max_speed_deg_s * dt
            self.ramps[name].nudge_target(step_deg)

        for name, ramp in self.ramps.items():
            if ramp.commanded is None:
                continue  # not seeded with a live angle yet
            if name not in self.arm.enabled:
                continue  # torque is off -- nothing to ramp toward
            new_cmd = ramp.tick(dt)
            if new_cmd is None:
                continue
            try:
                self.arm.go_to_angle(name, new_cmd)
            except (GimTimeout, can.CanError) as e:
                self.status_q.put({"type": "info", "text": f"[{name}] ramp send failed: {e}"})

    # ---- commands from the browser ----------------------------------------
    def _handle_command(self, cmd):
        name = cmd.get("joint")
        try:
            t = cmd["type"]
            if t == "enable":
                self.arm.enable(name)
                if name in self.ramps:
                    self.ramps[name].seed(self.arm.joints[name].get_angle())
            elif t == "disable":
                self.arm.disable(name)
                if name in self.ramps:
                    self.ramps[name].stop_in_place()
                self.jog_active[name] = 0
            elif t == "enable_batch":
                for n in cmd["joints"]:
                    self.arm.enable(n)
                    if n in self.ramps:
                        self.ramps[n].seed(self.arm.joints[n].get_angle())
            elif t == "disable_batch":
                for n in cmd["joints"]:
                    self.arm.disable(n)
                    if n in self.ramps:
                        self.ramps[n].stop_in_place()
                    self.jog_active[n] = 0
            elif t == "disable_all":
                self.arm.disable_all()
                for r in self.ramps.values():
                    r.stop_in_place()
                self.jog_active = {}
            elif t == "wrist_go":
                result = self.arm.wrist_mix(cmd["pitch_deg"], cmd["roll_deg"])
                targets = {k: v for k, v in result.items()
                           if k not in ("pitch_deg", "roll_deg", "was_scaled")}
                skipped = []
                sent_desc = []
                for n, deg in targets.items():
                    if n not in self.arm.enabled:
                        skipped.append(n)
                        continue
                    if cmd.get("ramped", True) and n in self.ramps:
                        self.ramps[n].set_target(deg)
                    else:
                        sent = self.arm.go_to_angle(n, deg)
                        if n in self.ramps:
                            self.ramps[n].sync_to(sent)
                    sent_desc.append(f"{n}={deg:.2f}")
                if skipped:
                    self.status_q.put({"type": "info", "text":
                                        f"Wrist go skipped (torque OFF): {', '.join(skipped)}"})
                else:
                    note = " (reduced -- requested combo exceeded motor travel, see Event Log)" \
                        if result.get("was_scaled") else ""
                    self.status_q.put({"type": "info", "text":
                                        f"Wrist: requested pitch {cmd['pitch_deg']:.2f}/roll {cmd['roll_deg']:.2f} deg "
                                        f"-> achieved pitch {result['pitch_deg']:.2f}/roll {result['roll_deg']:.2f} deg "
                                        f"-> {', '.join(sent_desc)}{note}"})
            elif t == "set_wrist_diff_config":
                self.arm.set_wrist_diff_config(**cmd["wrist_diff"])
            elif t == "set_home_batch":
                for n in cmd["joints"]:
                    self.arm.set_home(n)
            elif t == "set_home":
                self.arm.set_home(name)
            elif t == "go_to_angle":
                if name not in self.arm.enabled:
                    self.status_q.put({"type": "info", "text": f"[{name}] ignored go-to: torque is OFF (enable first)"})
                elif cmd.get("ramped", True) and name in self.ramps:
                    clamped, was_clamped = self.ramps[name].set_target(cmd["deg"])
                    self.status_q.put({"type": "info", "text":
                                        f"[{name}] ramping to {clamped:.2f} deg" +
                                        (" (clamped to limit)" if was_clamped else "")})
                else:
                    sent = self.arm.go_to_angle(name, cmd["deg"])
                    if name in self.ramps:
                        self.ramps[name].sync_to(sent)
                    self.status_q.put({"type": "info", "text": f"[{name}] commanded {sent:.2f} deg (direct)"})
            elif t == "go_to_angle_batch":
                skipped = []
                for n, deg in cmd["targets"].items():
                    if n not in self.arm.enabled:
                        skipped.append(n)
                        continue
                    if n in self.ramps:
                        self.ramps[n].set_target(deg)
                if skipped:
                    self.status_q.put({"type": "info", "text": f"Batch go-to skipped (torque OFF): {', '.join(skipped)}"})
            elif t == "jog_start":
                if name not in self.arm.enabled:
                    self.status_q.put({"type": "info", "text": f"[{name}] ignored jog: torque is OFF (enable first)"})
                else:
                    self.jog_active[name] = cmd["direction"]
            elif t == "jog_stop":
                self.jog_active[name] = 0
            elif t == "set_position_gains":
                self.arm.set_position_gains(name, cmd["kp"], cmd["ki"])
            elif t == "set_velocity_gains":
                self.arm.set_velocity_gains(name, cmd["kp"], cmd["ki"])
            elif t == "clear_fault":
                self.arm.clear_fault(name)
            elif t == "clear_fault_batch":
                for n in cmd["joints"]:
                    self.arm.clear_fault(n)
            elif t == "set_direction":
                self.arm.set_direction(name, cmd["direction"])
            elif t == "set_limits":
                self.arm.set_limits(name, cmd["min_deg"], cmd["max_deg"])
                if name in self.ramps:
                    self.ramps[name].set_limits(cmd["min_deg"], cmd["max_deg"])
            elif t == "set_speed_limits":
                self.arm.set_speed_limits(name, cmd["max_speed_rpm"], cmd["max_accel_rpm_s"], cmd["max_current_a"])
                if name in self.ramps:
                    self.ramps[name].set_speed_limits(cmd["max_speed_rpm"] * 6.0, cmd["max_accel_rpm_s"] * 6.0)
            elif t == "add_joint":
                self.arm.add_joint(**cmd["joint_def"])
                self._rebuild_ramps()
            elif t == "remove_joint":
                self.arm.remove_joint(name)
                self.ramps.pop(name, None)
                self.jog_active.pop(name, None)
            elif t == "assign_address":
                self.arm.assign_address(cmd["old_addr"], cmd["new_addr"])
            elif t == "reconnect":
                self.arm.connect()
                self._rebuild_ramps()
                self.status_q.put({"type": "info", "text": "Reconnected to CAN bus."})
        except (GimTimeout, can.CanError) as e:
            self.status_q.put({"type": "info", "text": f"[{name}] {e.__class__.__name__}: {e}"})
            log_event("error", f"{e.__class__.__name__}: {e}", joint=name)
        except Exception as e:
            self.status_q.put({"type": "info", "text": f"[{name}] ERROR: {e}"})
            log_event("error", f"ERROR: {e}", joint=name)
