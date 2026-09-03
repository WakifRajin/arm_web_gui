## Interplanetar Arm Control — production build

This is the finished, production-grade version of the rover arm GUI:
the same calibrated backend (CAN protocol, ramping, E-STOP, watchdogs)
you had, wrapped in a completely rebuilt frontend — a sidebar mission-
control layout, refreshed typography and color system, custom icon
set, and full responsive support down to a phone screen. Every
websocket command, REST endpoint, and safety behavior is unchanged
from the version this was built on; only the browser UI was redone.

## Install & run

```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000
```

`gim_motor.py`, `joint.py`, and `can_bus.py` (your calibrated CAN
protocol driver, degree-based Joint class, and bus adapter settings)
are already included in `backend/` — the module docstrings mention
them as drop-in files because that's how this project started; if
you've since updated any of the three on your bench copy, drop your
newer version in before starting the server.

Then open `http://<this-machine's-ip>:8000` from any browser on the
same network (the bench laptop itself, or a phone/tablet next to the
arm — the UI is fully responsive, with a slide-in nav drawer below
~980px wide). First run creates `backend/gui_joints.json` from
`gui_joints.default.json` (the base/shoulder/elbow/wrist-L/wrist-R/
gripper map, IDs 9–14) — edit that file directly, or just use the
GUI's limit/speed editors, which write back to it.

## The frontend, in this build

The nav is a collapsible left rail grouped by workflow phase —
**Operate** (Cockpit, Simulation), **Monitor** (Dashboard, Event Log),
**Configure** (Control, Wrist, Batch, Discovery, CAN Bus, Settings) —
instead of a single row of tabs. Click the rail's top-right icon to
collapse it to icons-only on a wide screen; on a narrow one it becomes
a slide-in drawer behind a hamburger button in the top bar.

Numeric telemetry (angles, currents, voltages, targets) is set in a
monospaced type with tabular figures throughout, so digits line up
column-to-column instead of jittering as they update. The Emergency
Stop button uses a hazard-stripe treatment (the only place that motif
appears) so it stays visually distinct from every other control in
the app, at any screen size, at a glance.

Every websocket message type, REST route, element ID, and CSS class
the JavaScript depends on is unchanged from the prototype, so nothing
about the backend contract shifted — this was purely a visual and
structural rebuild of `frontend/index.html`, `frontend/styles.css`,
and a small additive block at the end of `frontend/app.js` for the
new collapsible/mobile nav. If you customize the UI further, the
color tokens, type scale, and spacing scale all live at the top of
`styles.css` under `:root`.

## What's in the dashboard

- **Cockpit** (new, and the tab you land on) — everything you need for
  a normal session, decluttered from the tuning/diagnostic tabs below:
  - **Startup** — a 3-step wizard for "just powered the arm on":
    confirm the motors are responding, Set Home for every joint (same
    permanent `set_home_batch` as the old Batch tab, just one click),
    then Enable Torque for every joint. It collapses itself once every
    configured joint is enabled, and "Re-run Startup" brings it back.
  - **Arm Position** — a live schematic (base turret, shoulder/elbow/
    wrist links, gripper jaws) redrawn from the same status stream the
    other tabs use. It's proportional-within-range, not a literal
    kinematic replica: each link's on-screen angle is its live angle
    normalized into that joint's own configured min/max, so it stays
    legible even for a joint whose real travel is asymmetric (e.g.
    0..165 deg) without needing a hardware-verified sign convention.
    Numeric readouts sit next to it for the exact values.
  - **Quick Control** — one compact card per joint: connected dot,
    live angle, an Enable/Disable button, and hold-to-jog −/+ buttons.
    Exact typed angles, ramped-vs-direct, limits/speed/gains/direction
    stay on the Control tab under Advanced.
  - **Gamepad** — Xbox controller support, off by default (tick
    "Enable gamepad control"). See "Gamepad mapping" below.
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

## Gamepad mapping (Cockpit tab)

Built against the standard Gamepad API mapping (Xbox controller on
Chrome/Edge). Ported loosely from an older rig's PWM-jog mapping — this
arm is position-controlled and has two more DOF (no drive "roller"),
so a few slots were re-purposed rather than copied literally:

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
