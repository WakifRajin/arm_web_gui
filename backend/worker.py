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

TICK RATE / LAG FIX
---------------------------------------------------------------------------
gim_motor.py no longer blocks this thread on a CAN round-trip (see
can_link.py) -- sending a position command and reading back the latest
angle are both now just in-memory cache operations. That's what lets the
ramp tick run at CMD_HZ (50Hz, matching diff_wrist.py's control loop)
instead of the old 20Hz, and lets status polling run far more often than
the old poll_interval_s allowed without adding any real bus load, which
is the direct fix for "the browser sliders don't track in real time."

FOLLOW-ERROR / STALENESS SAFETY CHECK
---------------------------------------------------------------------------
Ported from diff_wrist.py's WristController._check_safety(): while a
joint's torque is on, compare what we just commanded against its last
known actual angle. If they diverge by more than follow_err_trip_deg for
longer than follow_err_trip_hold_s, something is physically wrong (stall,
jam, slipping load) even though the motor itself hasn't reported a fault
code -- so disable that joint's torque and log it clearly. The old arm
backend had no equivalent; a stalled joint would just keep commanding a
position it could never reach.
"""

import queue
import threading
import time

import can

from arm_controller import ArmController
from gim_motor import GimTimeout
from motion import RampedJoint
from logs import log_event

FOLLOW_ERR_TRIP_DEG = 10.0
FOLLOW_ERR_TRIP_HOLD_S = 0.3


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
        self.follow_err_since: dict[str, float | None] = {}  # name -> monotonic time trip started
        self._scan_lock = threading.Lock()
        self._scan_running = False

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

        poll_interval = self.arm.config.get("poll_interval_s", 0.08)
        scan_interval = self.arm.config.get("scan_interval_s", 2.0)
        bus_watchdog_s = self.arm.config.get("bus_watchdog_s", 3.0)
        # 50Hz -- matches diff_wrist.py's CMD_HZ. Safe now that go_to_angle()
        # never blocks on the bus (see can_link.py); at the old 20Hz every
        # jog/slider move visibly stair-stepped instead of gliding.
        tick_dt = 0.02
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
                    last_scan = time.time()
                    self._start_scan(window_s=0.3)

            except Exception as e:
                self.status_q.put({"type": "info", "text": f"worker loop error: {e}"})
                log_event("error", f"worker loop error: {e}")

            time.sleep(0.005)

    # ---- discovery scan (off the ramp-tick thread) ------------------------
    def _start_scan(self, window_s: float):
        # scan_broadcast() sleeps for window_s while it listens for
        # replies -- it used to do that inline in this loop, freezing
        # every joint's ramp for 300ms every scan_interval. Now that
        # ArmController.scan_broadcast() only touches the thread-safe
        # CanLink cache (never a raw bus.recv()), it's safe to run it on
        # its own short-lived thread instead, so discovery never stalls
        # motion. A lock just prevents two scans overlapping if the
        # previous one is still running.
        with self._scan_lock:
            if self._scan_running:
                return
            self._scan_running = True

        def _run():
            try:
                discovered = self.arm.scan_broadcast(window_s=window_s)
                self.status_q.put({"type": "discovered", "devices": discovered})
            except Exception as e:
                log_event("error", f"scan error: {e}")
            finally:
                with self._scan_lock:
                    self._scan_running = False

        threading.Thread(target=_run, daemon=True, name="arm-scan").start()

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
        self.follow_err_since = {}
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

        self._check_follow_errors(time.monotonic())

    # ---- follow-error safety check (ported from diff_wrist.py) ------------
    def _check_follow_errors(self, now: float):
        for name in list(self.arm.enabled):
            ramp = self.ramps.get(name)
            if ramp is None or ramp.commanded is None:
                continue
            try:
                actual = self.arm.joints[name].get_angle()
            except (GimTimeout, can.CanError):
                continue  # staleness is handled by the connected/bus-watchdog path already
            err = abs(ramp.commanded - actual)
            if err > FOLLOW_ERR_TRIP_DEG:
                since = self.follow_err_since.get(name)
                if since is None:
                    self.follow_err_since[name] = now
                elif now - since > FOLLOW_ERR_TRIP_HOLD_S:
                    self.arm.disable(name)
                    ramp.stop_in_place()
                    self.jog_active[name] = 0
                    self.follow_err_since[name] = None
                    log_event("error", f"{name}: FOLLOW-ERROR TRIP -- commanded "
                                        f"{ramp.commanded:+.2f} deg vs actual {actual:+.2f} deg "
                                        f"(>{FOLLOW_ERR_TRIP_DEG:.1f} deg for "
                                        f">{FOLLOW_ERR_TRIP_HOLD_S:.2f}s) -- torque disabled. "
                                        f"Check for a jam/stall before re-enabling.", joint=name)
                    self.status_q.put({"type": "info", "text":
                                        f"[{name}] follow-error trip -- torque disabled, "
                                        f"check for a jam before re-enabling"})
            else:
                self.follow_err_since[name] = None

    # ---- commands from the browser ----------------------------------------
    def _handle_command(self, cmd):
        name = cmd.get("joint")
        try:
            t = cmd["type"]
            if t == "enable":
                self.arm.enable(name)
                if name in self.ramps:
                    self.ramps[name].seed(self.arm.joints[name].get_angle())
                self.follow_err_since[name] = None
            elif t == "disable":
                self.arm.disable(name)
                if name in self.ramps:
                    self.ramps[name].stop_in_place()
                self.jog_active[name] = 0
                self.follow_err_since[name] = None
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
            elif t == "jog_nudge":
                # Discrete, exact-size step -- the customizable jog the
                # frontend's step selector drives (see JOG_STEPS in
                # app.js, ported from diff_wrist.py's JOG_STEPS_DEG /
                # bracket-key step cycling). One tap = one step, instead
                # of only a variable-duration hold at a fixed velocity.
                if name not in self.arm.enabled:
                    self.status_q.put({"type": "info", "text": f"[{name}] ignored jog: torque is OFF (enable first)"})
                elif name in self.ramps:
                    clamped, was_clamped = self.ramps[name].nudge_target(cmd["direction"] * cmd["step_deg"])
                    self.status_q.put({"type": "info", "text":
                                        f"[{name}] jog step {cmd['direction'] * cmd['step_deg']:+.3f} deg "
                                        f"-> target {clamped:.2f} deg" +
                                        (" (clamped to limit)" if was_clamped else "")})
            elif t == "jog_analog":
                # Continuous, proportional-rate jog for analog stick input
                # (gamepad). Same underlying mechanism as jog_start/
                # jog_stop -- both just set self.jog_active[name], which
                # _tick_ramps() multiplies by max_speed_deg_s every tick --
                # but here the magnitude is a float in [-1, 1] instead of
                # a fixed -1/0/1, so stick deflection controls rate
                # smoothly instead of only full-speed on/off. Silently
                # ignored (no Event Log entry) when torque is off, since
                # this can arrive at ~25 Hz while a stick is held, unlike
                # the one-shot jog_start/jog_stop clicks -- logging every
                # ignored frame would flood the log far worse than a
                # single "torque is OFF" notice would help.
                if name in self.arm.enabled:
                    self.jog_active[name] = max(-1.0, min(1.0, cmd.get("value", 0.0)))
                elif name in self.jog_active:
                    self.jog_active[name] = 0.0
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
