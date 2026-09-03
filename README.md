## Install & run

```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000
```

## Gamepad mapping

| Input | Action |
|---|---|
| Left stick X / Y | Base roll / Shoulder — proportional jog rate |
| Right stick X / Y | Wrist roll / Elbow — proportional jog rate |
| RT / LT | Shared jog-rate boost / precision-slow, applied to whatever the sticks are driving (this is the old "shared motor PWM slider," reinterpreted as a rate control since motion is now position-commanded, not PWM) |
| LB / RB | Wrist pitch − / + (hold) |
| D-pad ↑ / ↓ | Gripper open / close (hold) |
| D-pad → / ← | Wrist roll + / − (hold) — stands in for the old rig's "roller," which this arm doesn't have |
| L3 (left stick click) | Gripper stop |
| R3 (right stick click) | Wrist roll stop |
| B / X / Y | Toggle base / shoulder / elbow torque |
| A | Toggle gripper torque (not in the old mapping — added since B/X/Y only covered 3 of the 6 joints) |
| View / Back | Toggle wrist torque (both wrist motors together) |
| Menu / Start | Reset arm — torque OFF on every joint |

The wrist pitch/roll actions above act through the differential-wrist
config on the Wrist tab (`wrist_diff`) when it's enabled — both motors
move together, mixed per its pitch/roll signs. If `wrist_diff` isn't
enabled, they fall back to jogging `wrist_l` (pitch slot) or `wrist_r`
(roll slot) alone as a single-axis proxy, so the gamepad still does
something sensible on an un-mixed wrist.

Analog stick jogging uses a new `jog_analog` command (float `-1..1`,
sent at ~25 Hz while a stick is held) rather than the digital
`jog_start`/`jog_stop` the rest of the GUI uses — same underlying
per-joint ramp, just with a variable rate instead of always-max-speed,
and it never writes to the Event Log (unlike `jog_nudge`) since it can
arrive dozens of times a second.

"Menu/Start: Reset arm" disables torque on every joint (same as
`disable_all`) but is **not** the same as the red EMERGENCY STOP button
in the top bar: E-STOP also freezes every ramp exactly in place and is
the same path the bus watchdog uses, whereas Reset is a plain,
non-latching disable-all for "stop and start the wizard over."

## Safety notes carried over on purpose

- Soft angle limits are enforced by `Joint.set_angle()` itself (your
  existing calibrated clamp) — the GUI's slider/limit fields are a
  convenience for *tuning* those limits, never a way around them.
- The speed-ramp (soft-start/stop) is implemented in
  `backend/motion.py` at the control-loop level, not as a motor
  firmware register write — see the comment at the top of that file
  for why. It applies uniformly to jogging AND to typed "go to angle"
  commands, so a big commanded jump never snaps the joint.
- All CAN traffic — status polling, commands, ramp ticks, and the
  discovery scan — is serialized through the single `Worker` thread
  that owns the bus (`backend/worker.py`), exactly like
  `09_control_gui.py`'s `Worker`. Nothing else ever touches
  `self.bus` directly.
- The CAN-address-assign feature (`assign_address` /
  `CMD_SET_ADDR 0xBA`) is included because it already existed in your
  `can_scanner.py`, but per your own README, RS485 + ZE300_GUI
  remains the recommended way to address a motor — this CAN path is
  the one your README flagged as worth confirming against protocol
  v3.09b0 before relying on.
- A speed/current tune saved from the Limits panel now re-applies
  immediately if the joint's torque is already on (previously it
  silently waited for the next Enable, which looked like the
  controls just didn't work).
- Losing comms to a joint mid-ramp now freezes that joint's ramp in
  place instead of continuing to compute (and try to send) toward a
  target the motor may not be hearing anymore.

### CAN failsafes (`backend/can_diag.py`)

The CAN Bus tab's status/up/down/candump controls shell out to
Linux SocketCAN tooling (`ip link`, `candump`) through their own
short-lived subprocess, completely independent of the Worker's
python-can `bus` object — so a wedged or bus-off link can never
block a diagnostic read, and a candump capture never contends with
the Worker for the bus. Requires:

- a SocketCAN interface (`"can_interface"` in `gui_joints.json`,
  default `can0`)
- `can-utils` for candump: `sudo apt install can-utils`
- permission to run `ip link set <iface> up/down` — run the backend
  as root, grant it `CAP_NET_ADMIN`
  (`sudo setcap cap_net_admin+ep $(readlink -f $(which python3))`),
  or add a narrow passwordless sudoers rule (see the docstring at
  the top of `can_diag.py` for the exact line). We try the bare
  command first and fall back to a non-interactive `sudo -n`, so
  option 1 or 2 need nothing extra.

## Extending to the wheels (`test_gui.py`'s rig)

The wheel motors (GIM6010-36 / GIM8108-9, no encoder, velocity-only)
use a different command set (`0xC1` velocity, `0xB5` ramp, `0xB3` max
current, `0xCF` E-STOP, `0xAF` clear fault — all in `test_gui.py`).
This GUI's architecture (Worker thread owning the bus, ramp-per-axis,
websocket status fan-out) would extend cleanly to a second
`DriveController` alongside `ArmController` if/when you want the
wheels in the same browser dashboard — say the word and that's a
follow-up, not a rewrite.

## Project layout

```
backend/
  main.py               FastAPI app: websocket + REST, serves frontend/
  worker.py              background thread: owns the bus, ramps, E-STOP
  arm_controller.py       motor logic (wraps your gim_motor/joint/can_bus)
  motion.py               per-joint soft-ramp (speed/accel limited moves)
  can_diag.py             CAN interface up/down/status + candump (failsafes)
  logs.py                 rotating file log + in-memory ring buffer
  gui_joints.default.json seed config (base/shoulder/elbow/wristL/R/gripper)
  requirements.txt
frontend/
  index.html
  app.js
  styles.css
```
