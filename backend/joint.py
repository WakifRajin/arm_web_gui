"""
joint.py

Wraps a GimMotor with:
  - degree-based interface (never touch raw counts elsewhere in your code)
  - soft angular limits enforced in software
  - an empirical calibration step, because whether the protocol's
    "16384 counts = 1 revolution" refers to the MOTOR shaft or the
    OUTPUT shaft (after your 48:1 / 40:1 gearbox) is NOT stated
    outright in the protocol doc -- we measure it directly instead
    of assuming.

This is the ONLY place degrees<->counts conversion should happen.
"""

import json
import os
from gim_motor import GimMotor, COUNTS_PER_REV


class Joint:
    def __init__(self, name: str, motor: GimMotor, direction: int = 1,
                 min_deg: float = -170.0, max_deg: float = 170.0,
                 counts_per_output_deg: float = None,
                 calib_file: str = "joint_calibration.json"):
        """
        name:  human-readable joint name, e.g. "base"
        motor: a connected GimMotor instance
        direction: +1 or -1 -- flip if positive command moves the wrong way
        min_deg/max_deg: SOFT limits, enforced here before any command is sent
        counts_per_output_deg: if known already (from calibrate()), pass it in.
            If None, falls back to the naive assumption (counts are already
            output-shaft-referenced) until you run calibrate().
        """
        if direction not in (1, -1):
            raise ValueError("direction must be 1 or -1")
        if min_deg >= max_deg:
            raise ValueError("min_deg must be < max_deg")

        self.name = name
        self.motor = motor
        self.direction = direction
        self.min_deg = min_deg
        self.max_deg = max_deg
        self.calib_file = calib_file

        if counts_per_output_deg is not None:
            self.counts_per_output_deg = counts_per_output_deg
        else:
            self.counts_per_output_deg = self._load_calibration()

    # ------------------------------------------------------------------
    # Calibration -- run this once per joint, before trusting angles
    # ------------------------------------------------------------------

    def _load_calibration(self):
        naive = COUNTS_PER_REV / 360.0  # assumes counts are output-shaft already
        if os.path.exists(self.calib_file):
            with open(self.calib_file) as f:
                data = json.load(f)
            if self.name in data:
                print(f"[{self.name}] loaded calibration: "
                      f"{data[self.name]:.4f} counts/deg")
                return data[self.name]
        print(f"[{self.name}] WARNING: no calibration found, using naive "
              f"assumption ({naive:.4f} counts/deg). Run calibrate() first.")
        return naive

    def _save_calibration(self):
        data = {}
        if os.path.exists(self.calib_file):
            with open(self.calib_file) as f:
                data = json.load(f)
        data[self.name] = self.counts_per_output_deg
        with open(self.calib_file, "w") as f:
            json.dump(data, f, indent=2)

    def calibrate(self, test_counts: int = 900):
        """
        Interactive calibration. Sends a small KNOWN relative move (in raw
        counts) and asks you to physically measure how far the OUTPUT
        shaft actually rotated (protractor, angle gauge, or a marked
        reference). From that we compute real counts-per-output-degree,
        which correctly accounts for whatever internal scaling the
        firmware does (or doesn't do) for the gear ratio.

        Run this ONCE per joint, after mounting, before any real testing.
        Torque should be enabled with LOW speed/current limits already set.
        """
        print(f"\n=== Calibrating joint '{self.name}' ===")
        print("Make sure max speed/current are set LOW before continuing.")
        input("Press Enter to command a small test rotation...")

        before = self.motor.read_position()
        self.motor.move_relative_counts(test_counts * self.direction)
        import time
        time.sleep(1.0)
        after = self.motor.read_position()

        raw_delta_deg = after["multi_turn_deg"] - before["multi_turn_deg"]
        print(f"Motor-reported change: {raw_delta_deg:.3f} deg "
              f"(from {test_counts} raw counts)")

        measured = input(
            "Physically measure how far the OUTPUT shaft actually rotated, "
            "in degrees (use a protractor / angle gauge), and enter it here: "
        )
        measured_deg = float(measured)
        if abs(measured_deg) < 1e-6:
            print("Measured angle too close to zero, aborting calibration.")
            return

        self.counts_per_output_deg = abs(test_counts / measured_deg)
        print(f"Computed: {self.counts_per_output_deg:.4f} counts per output degree")
        expected_naive = COUNTS_PER_REV / 360.0
        ratio = self.counts_per_output_deg / expected_naive
        print(f"(naive assumption was {expected_naive:.4f}; "
              f"your joint's real value is {ratio:.2f}x that -- "
              f"if this is close to your gear ratio, counts were motor-side)")

        self._save_calibration()
        print(f"Saved to {self.calib_file}\n")

    # ------------------------------------------------------------------
    # Degree <-> counts conversion (single source of truth)
    # ------------------------------------------------------------------

    def _deg_to_counts(self, deg: float) -> int:
        return int(round(deg * self.counts_per_output_deg * self.direction))

    def _counts_deg_to_output_deg(self, motor_reported_deg: float) -> float:
        # motor_reported_deg is already in "motor's own 360/16384 degrees";
        # convert through our measured counts_per_output_deg for consistency
        raw_counts = motor_reported_deg * (COUNTS_PER_REV / 360.0)
        return (raw_counts / self.counts_per_output_deg) * self.direction

    # ------------------------------------------------------------------
    # Public interface -- everything else in your codebase uses THIS
    # ------------------------------------------------------------------

    def clamp(self, deg: float) -> float:
        clamped = max(self.min_deg, min(self.max_deg, deg))
        if clamped != deg:
            print(f"[{self.name}] WARNING: requested {deg:.2f} deg clamped "
                  f"to soft limit {clamped:.2f} deg")
        return clamped

    def set_home_here(self):
        """Call with the joint physically at your intended zero pose,
        torque disabled."""
        self.motor.set_home()
        print(f"[{self.name}] home set at current physical position")

    def enable(self, max_speed_rpm: float, max_current_a: float):
        """Sets conservative limits (NOT saved across power cycles --
        call this every boot before commanding motion)."""
        self.motor.set_max_speed(max_speed_rpm)
        self.motor.set_max_current(max_current_a)

    def disable(self):
        self.motor.disable()

    def set_angle(self, deg: float, relative: bool = False):
        deg = self.clamp(deg)
        if relative:
            current = self.get_angle()
            deg = self.clamp(current + deg)
            delta_deg = deg - current
            self.motor.move_relative_counts(self._deg_to_counts(delta_deg))
        else:
            self.motor.move_absolute_counts(self._deg_to_counts(deg))
        return deg

    def get_angle(self) -> float:
        pos = self.motor.read_position()
        return self._counts_deg_to_output_deg(pos["multi_turn_deg"])

    def get_current(self) -> float:
        return self.motor.read_current()

    def estimate_torque(self, torque_constant_Nm_per_A: float) -> float:
        return self.get_current() * torque_constant_Nm_per_A
