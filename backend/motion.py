"""
motion.py

Per-joint software speed ramp (soft-start / soft-stop).

This is deliberately implemented at the *control-loop* level rather
than relying on any motor-firmware ramp register, for two reasons:

  1. We only know for certain that the arm's GimMotor/Joint stack
     exposes enable()/set_angle()/get_angle() (see 09_control_gui.py).
     Whatever firmware ramp/accel registers exist are model- and
     firmware-revision-specific -- doing it here works identically
     for every joint regardless of firmware version.
  2. It gives one single place -- this file -- that both "hold arrow
     key to jog" AND "type an angle and hit Go" flow through, so a
     typed target 90 degrees away ramps exactly the same way a jog
     does, instead of snapping to a step position command.

RampedJoint wraps one Joint's angle target with a max velocity and
max acceleration (both live-tunable). Call set_target(deg) whenever
the desired final angle changes (jog delta, typed "go to", etc.) and
call tick(dt) at a fixed rate (driven by the Worker loop) to get the
next intermediate command angle, already clamped to the joint's soft
limits. tick() returns None on ticks where nothing new needs to be
sent (already at target), so callers don't spam the bus.
"""

from dataclasses import dataclass


@dataclass
class RampedJoint:
    name: str
    min_deg: float
    max_deg: float
    max_speed_deg_s: float = 20.0
    max_accel_deg_s2: float = 40.0

    def __post_init__(self):
        self._current_cmd = None   # last commanded angle (deg), None until seeded
        self._target = None        # desired final angle (deg)
        self._velocity = 0.0       # deg/s, signed
        self._last_sent = None

    def seed(self, live_angle: float):
        """Call once, the first time we get a real live angle back
        from the motor, so the ramp starts from reality instead of 0."""
        if self._current_cmd is None:
            self._current_cmd = live_angle
            self._target = live_angle

    def set_limits(self, min_deg: float, max_deg: float):
        self.min_deg, self.max_deg = min_deg, max_deg

    def set_speed_limits(self, max_speed_deg_s: float, max_accel_deg_s2: float):
        self.max_speed_deg_s = max(0.0, max_speed_deg_s)
        self.max_accel_deg_s2 = max(0.0, max_accel_deg_s2)

    def set_target(self, deg: float):
        clamped = max(self.min_deg, min(self.max_deg, deg))
        self._target = clamped
        if self._current_cmd is None:
            self._current_cmd = clamped
        return clamped, (clamped != deg)  # (clamped value, was_clamped)

    def nudge_target(self, delta_deg: float):
        base = self._target if self._target is not None else 0.0
        return self.set_target(base + delta_deg)

    def sync_to(self, value: float):
        """Snap both the commanded and target angle to `value` with no
        ramping involved. Used right after a *direct* (unramped) move so
        the ramp doesn't try to silently "catch up" to the old target the
        next time a ramped move is requested for this joint."""
        self._current_cmd = value
        self._target = value
        self._velocity = 0.0
        self._last_sent = value

    def stop_in_place(self):
        """Used by E-STOP / jog release: freeze the ramp exactly where
        it is right now instead of continuing toward the old target."""
        if self._current_cmd is not None:
            self._target = self._current_cmd
        self._velocity = 0.0

    def tick(self, dt: float):
        if self._current_cmd is None or self._target is None:
            return None

        error = self._target - self._current_cmd
        # Desired velocity this instant: move toward target, capped at max speed.
        desired_v = max(-self.max_speed_deg_s, min(self.max_speed_deg_s, error / max(dt, 1e-3)))

        dv = self.max_accel_deg_s2 * dt
        if self._velocity < desired_v:
            self._velocity = min(self._velocity + dv, desired_v)
        elif self._velocity > desired_v:
            self._velocity = max(self._velocity - dv, desired_v)

        step = self._velocity * dt
        if abs(step) > abs(error):
            step = error  # don't overshoot a target we're about to reach

        new_cmd = self._current_cmd + step
        new_cmd = max(self.min_deg, min(self.max_deg, new_cmd))
        self._current_cmd = new_cmd

        if abs(error) < 0.02 and abs(self._velocity) < 0.02:
            self._velocity = 0.0

        if self._last_sent is not None and abs(new_cmd - self._last_sent) < 0.01:
            return None  # no meaningful change -- don't spam the bus

        self._last_sent = new_cmd
        return new_cmd

    @property
    def target(self):
        return self._target

    @property
    def commanded(self):
        return self._current_cmd

    @property
    def velocity(self):
        return self._velocity

    @property
    def at_target(self):
        return (self._target is not None and self._current_cmd is not None
                and abs(self._target - self._current_cmd) < 0.02)
