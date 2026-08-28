# Rover Arm Web GUI

A browser-based dashboard for bringing up, testing, and driving the
6-DOF arm over CAN. 

## Install & run

```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000
```

Then open `http://<this-machine's-ip>:8000` from any browser on the
same network (the bench laptop itself, or a phone/tablet next to the
arm). First run creates `backend/gui_joints.json` from
`gui_joints.default.json` (the base/shoulder/elbow/wrist-L/wrist-R/
gripper map from your README, IDs 9–14) — edit that file directly, or
just use the GUI's limit/speed editors, which write back to it.

## What's in the dashboard

- **Dashboard** — a live card per joint: connected/disconnected
  (flips automatically the moment a motor stops answering polls, or
  a fresh one starts answering — no manual "rescan" needed for
  configured joints), onboard encoder angle, target angle, current,
  voltage, temperature, fault code.
- **Individual Control** — pick a joint, then: type-and-go to an
  angle or drag the slider (soft-ramped by default, or "direct" if
  you untick ramped), hold-to-jog buttons, Enable/Disable torque,
  Set Home (with confirmation — it's a permanent write to the
  motor), Clear Fault, and per-joint tunables: min/max angle, max
  speed (RPM), max accel (RPM/s), max current (A) — all saved to
  `gui_joints.json` immediately.
- **Batch Configure** — check any subset of joints and Enable /
  Disable / Clear Fault / send-to-angle all of them together in one
  click.
- **Auto-Detected Devices** — a periodic broadcast scan (same
  approach as `can_scanner.py`) lists any motor answering on the bus
  that isn't yet a configured joint, so plugging in a freshly-set
  motor shows up here without editing JSON by hand. "Add as joint"
  opens a small form (name, direction, limits, speed) and appends it
  to the live config.
- **Event Log** — every enable/disable/fault/limit-change/error is
  logged with a timestamp and (when applicable) which joint it's
  about, filterable by level and joint in the browser, and also
  written to `backend/arm_logs/arm.log` (rotating) so a session can
  be reviewed after the fact even if nobody was watching the screen.
- **EMERGENCY STOP** — top-right, always visible. Bypasses the normal
  command queue entirely (see `worker.py`'s `estop_event`), disables
  every joint's torque, and freezes every in-progress ramp exactly
  where it is instead of letting it coast to its old target.
- **CAN Bus tab** — interface state/bitrate/error-counter readout,
  a candump-style raw-frame snapshot, and two failsafes one level
  below E-STOP: "Bring CAN Up" (re-opens the SocketCAN link at the
  configured bitrate and reconnects) and "Bring CAN Down" (E-STOPs,
  then kills the link at the OS level — for when a motor's firmware
  won't stop responding to a normal disable frame). See "CAN
  failsafes" below for setup.
- **Bus watchdog** — if *no* motor answers a poll for
  `bus_watchdog_s` (default 3s, in `gui_joints.json`) while any
  joint still has torque enabled, the worker treats it as a dead/
  bus-off link and force-triggers the same E-STOP path a human
  would, instead of quietly polling into the void. The topbar shows
  a pulsing "BUS UNRESPONSIVE" banner whenever every configured
  joint is disconnected.

The GUI now also refuses (with a clear Event Log note, not a silent
no-op) to send a Go-To, jog, or batch move to a joint whose torque
is off — previously those commands went out anyway and did nothing,
which looked like the position/speed controls were broken.

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

## Extending to the wheels

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
