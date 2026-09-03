"""
gim_motor.py

Low-level driver for SteadyWin GIM motors ("Custom CAN Communication
Protocol_V3.07b0"). Same public API as before -- read_status(),
read_position(), move_absolute_counts(), enable-time set_max_speed()/
set_max_current(), disable(), etc. -- so joint.py and arm_controller.py
needed NO changes to keep working.

WHAT CHANGED UNDERNEATH
---------------------------------------------------------------------------
Every one of those methods used to send a command and then block the
calling thread waiting for THIS motor's reply. That serialized every
joint's motion command behind a real CAN round-trip, inside the Worker's
single control-loop thread -- the direct cause of the reported motor lag.

Now every GimMotor is a thin facade over one shared can_link.CanLink
(one per underlying bus, memoized by `id(bus)` so every joint built from
the same `ArmController.bus` shares the same RX thread and background
poller -- see can_link.py's module docstring, and diff_wrist.py's
WristDriver, which this whole design is ported from):

  - read_status() / read_position() / read_current() / read_speed()
    return the latest value CanLink's background poller already fetched
    -- no bus round-trip on the calling thread at all. They raise
    GimTimeout if this motor has never answered, or hasn't answered
    recently enough to trust (see can_link.RX_TIMEOUT_S).
  - move_absolute_counts() / move_relative_counts() / set_max_speed() /
    set_max_current() / set_home() / disable() / brake() fire the frame
    and return immediately -- exactly like diff_wrist's WristDriver.send().
    Their return values are now the CLAMPED/COMMANDED value (best-effort),
    not a device-echoed confirmation -- callers that need to know the
    device actually got there read the next read_position()/read_status().

Requires: pip install python-can
"""

import time
from typing import Dict

import can

from can_link import CanLink, decode_fault, PUBLIC_ADDR, CMD_VERSION, CMD_STATUS, \
    CMD_SET_ADDR, RX_TIMEOUT_S

COUNTS_PER_REV = 16384  # 14-bit encoder, confirmed by the spec sheet

_links: Dict[int, CanLink] = {}  # id(bus) -> shared CanLink


class GimFault(Exception):
    """Raised when the motor reports a fault condition."""
    pass


class GimTimeout(Exception):
    """Raised when a motor's cached status/position is missing or stale --
    same meaning as before (no reply), just detected by cache staleness
    instead of a per-call recv() timeout."""
    pass


def get_link(bus: can.BusABC) -> CanLink:
    """One CanLink per underlying bus object, shared by every GimMotor
    built on top of it. ArmController.connect() creates one `bus` and
    passes it to a GimMotor per joint, so in practice this always
    returns the same CanLink for the whole arm."""
    key = id(bus)
    link = _links.get(key)
    if link is None:
        link = CanLink(bus)
        link.start_polling()
        _links[key] = link
    return link


class GimMotor:
    """
    One GIM motor on the CAN bus, addressed by its Dev_addr (1-254).
    Same constructor signature as before: GimMotor(bus, dev_addr).
    """

    def __init__(self, bus: can.BusABC, dev_addr: int, timeout: float = RX_TIMEOUT_S):
        if not (1 <= dev_addr <= 254):
            raise ValueError("dev_addr must be 1-254")
        self.addr = dev_addr
        self.timeout = timeout
        self.link = get_link(bus)
        self.link.register(dev_addr)

    # ------------------------------------------------------------------
    # System commands
    # ------------------------------------------------------------------

    def read_version(self) -> dict:
        """0xA0 - boot/app/hardware/protocol versions (cached; the poller
        doesn't request this on its own, so trigger one explicitly and
        give it a brief moment to land -- this is a rare, one-off call,
        never in the motion hot path)."""
        self.link.read_version(self.addr)
        st = self._wait_fresh(lambda s: s.protocol_version is not None, timeout=1.0)
        return {"protocol": st.protocol_version}

    def read_status(self) -> dict:
        """0xAE - bus voltage/current, temperature, run mode, fault code.
        Backed entirely by CanLink's background poller -- no send here."""
        st = self._require_fresh(self.link.state(self.addr).last_status_rx
                                  if self.link.state(self.addr) else 0.0)
        return {
            "bus_voltage_v": st.voltage_V,
            "bus_current_a": st.bus_current_A,
            "temperature_c": st.temperature_C,
            "run_mode": st.run_mode,
            "fault_code": st.fault_code,
            "fault_text": decode_fault(st.fault_code),
        }

    def clear_faults(self) -> int:
        """0xAF - Clear faults (fire-and-forget; read back via read_status())."""
        self.link.clear_faults(self.addr)
        return 0

    def read_motor_params(self) -> dict:
        """0xB0 - not tracked by the background poller (rarely needed,
        firmware-constant). Left unimplemented in the async cache on
        purpose -- add a request_and_wait() call here if you need it."""
        raise NotImplementedError("read_motor_params: use link.request_and_wait if needed")

    # ------------------------------------------------------------------
    # Homing / zero
    # ------------------------------------------------------------------

    def set_home(self):
        """0xB1 - Set CURRENT physical position as home/zero (permanent).
        Fire-and-forget, same as diff_wrist.py's set_home_persistent() --
        the driver's ack is logged by can_link.py when it arrives.
        IMPORTANT: physically move the joint (motor disabled) to your
        intended zero pose BEFORE calling this."""
        self.link.set_home(self.addr)

    def return_home(self):
        """0xC4 - Move to stored home along shortest path."""
        self.link.return_home(self.addr)

    # ------------------------------------------------------------------
    # Position / speed / current limits (re-send every boot)
    # ------------------------------------------------------------------

    def set_max_speed(self, rpm: float):
        """0xB2 - Max speed in position mode."""
        self.link.set_max_speed_rpm(self.addr, rpm)
        return rpm

    def set_max_current(self, amps: float):
        """0xB3 - Max Q-axis current in position/speed mode."""
        self.link.set_max_current_A(self.addr, amps)
        return amps

    # ------------------------------------------------------------------
    # Motion commands
    # ------------------------------------------------------------------

    def read_position(self) -> dict:
        """0xA3 - single-turn / multi-turn absolute angle, degrees."""
        st = self._require_fresh(self.link.state(self.addr).last_pos_rx
                                  if self.link.state(self.addr) else 0.0)
        return {
            "single_turn_deg": st.raw_single_deg,
            "multi_turn_deg": st.raw_multi_deg,
        }

    def read_current(self) -> float:
        """0xA1 - real-time Q-axis current, Amps."""
        st = self._require_fresh(self.link.state(self.addr).last_current_rx
                                  if self.link.state(self.addr) else 0.0)
        return st.current_A

    def read_speed(self) -> float:
        """0xA2 - real-time speed, RPM."""
        st = self._require_fresh(self.link.state(self.addr).last_speed_rx
                                  if self.link.state(self.addr) else 0.0)
        return st.speed_rpm

    def move_absolute_counts(self, counts: int):
        """0xC2 - Absolute position control, raw counts. Fire-and-forget:
        does NOT wait for the driver to echo the new position -- read
        read_position() on the next poll for that. This is the call the
        ramp tick makes every cycle, so it must never block."""
        self.link.move_absolute_counts(self.addr, counts)

    def move_relative_counts(self, delta_counts: int):
        """0xC3 - Relative position control, raw counts. Fire-and-forget."""
        self.link.move_relative_counts(self.addr, delta_counts)

    # ------------------------------------------------------------------
    # Enable / disable / brake
    # ------------------------------------------------------------------

    def disable(self):
        """0xCF - Disable output, free state. Fire-and-forget -- this is
        the E-STOP path, and E-STOP must never be able to block on a
        motor that's stopped answering."""
        self.link.disable(self.addr)

    def brake(self, engaged: bool):
        """0xCE - Holding brake on/off, if wired."""
        self.link.brake(self.addr, engaged)
        return engaged

    def set_driver_watchdog(self, enable: bool, ms: int, action: int = 0x02):
        """0xCD - driver-side comms watchdog. See can_link.py -- ported
        from diff_wrist.py, not present in the arm backend before this."""
        self.link.set_driver_watchdog(self.addr, enable, ms, action)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _require_fresh(self, last_rx: float):
        st = self.link.state(self.addr)
        now = time.monotonic()
        if st is None or last_rx == 0.0 or (now - last_rx) > self.timeout:
            raise GimTimeout(
                f"No fresh reply cached for motor {self.addr} "
                f"(last update {('%.2fs ago' % (now - last_rx)) if last_rx else 'never'})"
            )
        return st

    def _wait_fresh(self, predicate, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            st = self.link.state(self.addr)
            if st is not None and predicate(st):
                return st
            time.sleep(0.01)
        raise GimTimeout(f"Motor {self.addr}: no matching reply within {timeout}s")
