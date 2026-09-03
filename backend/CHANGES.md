# arm_web_gui changes — ported from diff_wrist.py

## The lag fix (root cause)

`gim_motor.py` used to send a CAN command and **block the calling thread**
waiting for that exact motor's reply (up to 0.5s timeout) before returning.
The Worker's ramp loop called this synchronously, once per active joint,
every tick — so multiple joints moving together meant multiple real CAN
round-trips serialized inside a single control tick. That's the reported
"somewhat laggy motor rotation."

`diff_wrist.py` never has this problem: it fires a command and moves on,
while one dedicated background thread continuously drains the bus and
decodes whatever comes back into a shared, lock-protected snapshot. Every
reader just reads the latest snapshot — nothing waits on the wire.

**This rewrite ports that architecture to every joint, not just the wrist.**

New/changed files:

- `can_link.py` **(new)** — shared, non-blocking CAN transport. One RX
  thread owns `bus.recv()` exclusively; a background poller keeps every
  registered motor's cache warm; sends are fire-and-forget.
- `gim_motor.py` **(rewritten)** — identical public API (`read_status()`,
  `read_position()`, `move_absolute_counts()`, etc.) so `joint.py` and
  `arm_controller.py` needed **no interface changes** — but every method is
  now backed by `can_link.CanLink`'s cache instead of a private blocking
  request/reply.
- `arm_controller.py` — discovery scan (`scan_broadcast`) and address
  assignment (`assign_address`) rerouted off raw `bus.recv()` (which would
  now race the new RX thread) onto the shared cache. Added the
  **driver-side comms watchdog (0xCD)**, ported straight from
  `diff_wrist.py`'s `arm()` — the old backend never sent this frame, so a
  wedged host previously left torque held until the (much coarser) browser
  bus-watchdog noticed.
- `worker.py` —
  - Ramp tick raised from 20Hz to 50Hz (`diff_wrist.py`'s `CMD_HZ`), safe
    now that `go_to_angle()` never blocks.
  - Discovery scan moved to its own short-lived thread instead of running
    inline in the ramp loop — it used to freeze every joint's motion for
    ~300ms every `scan_interval_s`.
  - **New: follow-error safety trip**, ported from
    `WristController._check_safety()`. While a joint's torque is on, its
    commanded angle is compared against its last known actual angle; if
    they diverge by more than `FOLLOW_ERR_TRIP_DEG` (10°) for longer than
    `FOLLOW_ERR_TRIP_HOLD_S` (0.3s), that joint is disabled and logged —
    catches a stall/jam/slipping load even when the motor reports no fault
    code. The old backend had nothing like this.
  - **New: `jog_nudge` command** — discrete, exact-size single-step jog
    (see frontend below), alongside the existing hold-to-jog.
- `gui_joints.json` / `gui_joints.default.json` — `poll_interval_s` lowered
  from 0.4s to 0.08s (matches `diff_wrist.py`'s `GUI_REFRESH_MS`=80ms) now
  that reads are cache hits instead of bus round-trips, so the browser
  gets real position/target/velocity updates ~5x more often. **CAN
  addresses (9–14) are untouched.**

## Frontend (jog + real-time feedback)

- Jog is now **customizable**: a step-size chip row (0.1° – 30°, ported
  from `diff_wrist.py`'s `JOG_STEPS_DEG`) plus a custom numeric step.
  `−step`/`+step` buttons issue one exact nudge each via the new
  `jog_nudge` command; the existing hold-to-jog buttons are unchanged
  (continuous, soft-ramped).
- Keyboard shortcuts on the Control tab (ported from `diff_wrist.py`'s key
  bindings): `[`/`]` cycle the step size, `,`/`.` nudge by the current
  step, hold `←`/`→` to jog continuously, `X`/`Esc` triggers E-STOP from
  anywhere.
- Same jog step now also drives small pitch/roll nudge buttons on the
  Wrist tab (the wrist is just two of the same joints, mixed in software —
  no separate protocol involved).
- The "sliders/values don't update in real time" complaint was the same
  root cause as the lag: `poll_interval_s` throttled how often the backend
  even *read* a fresh angle, and reading was itself slow. Both are fixed
  above — no frontend polling-rate change was needed once the backend
  could push updates cheaply and often.

## Verified

`can_link.py`, `gim_motor.py` → `joint.py` → `arm_controller.py`, and
`worker.py` were all exercised end-to-end against a `python-can` **virtual
bus** with simulated GIM motors (see the three test scripts used during
development): non-blocking sends confirmed sub-millisecond, a 6-joint
simultaneous command burst went from what would have been 50–500ms/joint
blocking to ~0.3ms total, and the new follow-error trip correctly
auto-disabled a simulated stalled joint while leaving healthy joints
untouched. `app.js`/`index.html` were syntax- and reference-checked
(every `$(id)` resolves, tags balanced).
