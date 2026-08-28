"""
gim_motor.py

Low-level driver for SteadyWin GIM motors, implementing
"Custom CAN Communication Protocol_V3.07b0" (the PDF you supplied).

This talks the STANDARD command set (0xA0-0xCF), NOT the MIT-type
motion-control mode (0xF0/0xF1) -- standard mode uses simple integer
"counts" for position, which is much harder to misuse than the scaled
16-bit MIT encoding. Start here; move to MIT mode later only if you
specifically need combined position+velocity+torque+gain in one frame.

Requires: pip install python-can
"""

import struct
import time
import can


COUNTS_PER_REV = 16384  # 14-bit encoder, confirmed by your spec sheet


class GimFault(Exception):
    """Raised when the motor reports a fault condition."""
    pass


class GimTimeout(Exception):
    """Raised when the motor doesn't reply within the timeout window."""
    pass


class GimMotor:
    """
    One GIM motor on the CAN bus, addressed by its Dev_addr (1-254).

    Example:
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        base = GimMotor(bus, dev_addr=1)
        base.clear_faults()
        base.set_home()
    """

    def __init__(self, bus: can.BusABC, dev_addr: int, timeout: float = 0.5):
        if not (1 <= dev_addr <= 254):
            raise ValueError("dev_addr must be 1-254")
        self.bus = bus
        self.addr = dev_addr
        self.timeout = timeout

    # ------------------------------------------------------------------
    # Low-level frame send/receive
    # ------------------------------------------------------------------

    def _send(self, data: bytes, use_alt_id: bool = False):
        """Send a frame to this motor. use_alt_id sends to (0x100 | addr)
        instead of addr -- both are accepted by the slave per the spec."""
        arb_id = (0x100 | self.addr) if use_alt_id else self.addr
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def _recv(self, expect_cmd: int, expect_dlc: int = None, timeout: float = None):
        """Wait for a reply frame from this motor with the given command
        byte in Data[0]. Ignores frames from other devices on the bus."""
        deadline = time.time() + (timeout or self.timeout)
        while time.time() < deadline:
            msg = self.bus.recv(timeout=deadline - time.time())
            if msg is None:
                break
            if msg.arbitration_id != self.addr:
                continue  # not from this motor
            if len(msg.data) == 0 or msg.data[0] != expect_cmd:
                continue  # not the reply we're waiting for
            if expect_dlc is not None and len(msg.data) != expect_dlc:
                continue
            return msg.data
        raise GimTimeout(
            f"No reply from motor {self.addr} for command 0x{expect_cmd:02X} "
            f"within {timeout or self.timeout}s"
        )

    def _cmd(self, cmd: int, payload: bytes = b"", expect_dlc: int = None, timeout: float = None):
        """Send a command byte + payload, wait for the matching reply."""
        self._send(bytes([cmd]) + payload)
        return self._recv(cmd, expect_dlc=expect_dlc, timeout=timeout)

    # ------------------------------------------------------------------
    # System commands
    # ------------------------------------------------------------------

    def read_version(self) -> dict:
        """0xA0 - Read boot/app/hardware/protocol versions."""
        d = self._cmd(0xA0, expect_dlc=8)
        boot_fw = struct.unpack_from("<H", d, 1)[0]
        app_fw = struct.unpack_from("<H", d, 3)[0]
        hw_ver = struct.unpack_from("<H", d, 5)[0]
        proto = d[7]
        return {"boot_fw": boot_fw, "app_fw": app_fw, "hw_version": hw_ver, "protocol": proto}

    def read_status(self) -> dict:
        """0xAE - Read bus voltage/current, temperature, run mode, fault code."""
        d = self._cmd(0xAE, expect_dlc=8)
        bus_v = struct.unpack_from("<H", d, 1)[0] * 0.01
        bus_i = struct.unpack_from("<H", d, 3)[0] * 0.01
        temp_c = d[5]
        run_mode = d[6]
        fault = d[7]
        return {
            "bus_voltage_v": bus_v,
            "bus_current_a": bus_i,
            "temperature_c": temp_c,
            "run_mode": run_mode,
            "fault_code": fault,
            "fault_text": self._decode_fault(fault),
        }

    @staticmethod
    def _decode_fault(fault_code: int) -> str:
        if fault_code == 0:
            return "OK"
        bits = []
        names = {0: "Voltage fault", 1: "Current fault", 2: "Temperature fault",
                 3: "Encoder fault", 6: "Hardware fault", 7: "Software fault"}
        for bit, name in names.items():
            if fault_code & (1 << bit):
                bits.append(name)
        return ", ".join(bits) if bits else f"Unknown fault code 0x{fault_code:02X}"

    def clear_faults(self) -> int:
        """0xAF - Clear faults. Returns current fault code after clearing."""
        d = self._cmd(0xAF, expect_dlc=2)
        return d[1]

    def read_motor_params(self) -> dict:
        """0xB0 - Read pole pairs, torque constant, gear ratio."""
        d = self._cmd(0xB0, expect_dlc=7)
        pole_pairs = d[1]
        torque_const = struct.unpack_from("<f", d, 2)[0]
        gear_ratio = d[6]
        return {"pole_pairs": pole_pairs, "torque_constant_Nm_per_A": torque_const,
                "gear_ratio": gear_ratio}

    # ------------------------------------------------------------------
    # Homing / zero
    # ------------------------------------------------------------------

    def set_home(self) -> int:
        """
        0xB1 - Set CURRENT physical position as home/zero.
        Stored in the driver's persistent memory, survives power-off.

        IMPORTANT: physically move the joint (by hand, motor disabled)
        to your intended zero pose BEFORE calling this.
        """
        d = self._cmd(0xB1, expect_dlc=3, timeout=1.0)  # can take longer w/ 2nd encoder
        mech_offset = struct.unpack_from("<H", d, 1)[0]
        return mech_offset

    def return_home(self):
        """0xC4 - Move to stored home along shortest path, max 180 deg travel."""
        return self._cmd(0xC4, expect_dlc=7, timeout=2.0)

    # ------------------------------------------------------------------
    # Position / speed / current limits (NOT saved after power-off --
    # must be re-sent every boot, before enabling torque)
    # ------------------------------------------------------------------

    def set_max_speed(self, rpm: float):
        """0xB2 - Max speed in position mode. unit 0.01 rpm."""
        raw = int(round(rpm / 0.01))
        d = self._cmd(0xB2, struct.pack("<i", raw), expect_dlc=5)
        return struct.unpack_from("<i", d, 1)[0] * 0.01

    def set_max_current(self, amps: float):
        """0xB3 - Max Q-axis current in position/speed mode. unit 0.001 A."""
        raw = int(round(amps / 0.001))
        d = self._cmd(0xB3, struct.pack("<i", raw), expect_dlc=5)
        return struct.unpack_from("<i", d, 1)[0] * 0.001

    # ------------------------------------------------------------------
    # Motion commands
    # ------------------------------------------------------------------

    def read_position(self) -> dict:
        """0xA3 - Read single-turn and multi-turn absolute angle (degrees)."""
        d = self._cmd(0xA3, expect_dlc=7)
        raw_single = struct.unpack_from("<H", d, 1)[0]
        raw_multi = struct.unpack_from("<i", d, 3)[0]
        single_deg = raw_single * (360.0 / COUNTS_PER_REV)
        multi_deg = raw_multi * (360.0 / COUNTS_PER_REV)
        return {"single_turn_deg": single_deg, "multi_turn_deg": multi_deg,
                "raw_single": raw_single, "raw_multi": raw_multi}

    def read_current(self) -> float:
        """0xA1 - Read real-time Q-axis current, in Amps."""
        d = self._cmd(0xA1, expect_dlc=5)
        raw = struct.unpack_from("<i", d, 1)[0]
        return raw * 0.001

    def read_speed(self) -> float:
        """0xA2 - Read real-time speed, in RPM."""
        d = self._cmd(0xA2, expect_dlc=5)
        raw = struct.unpack_from("<i", d, 1)[0]
        return raw * 0.01

    def move_absolute_counts(self, counts: int):
        """0xC2 - Absolute position control, raw counts. Low-level; prefer Joint.set_angle()."""
        d = self._cmd(0xC2, struct.pack("<i", counts), expect_dlc=7)
        raw_single = struct.unpack_from("<H", d, 1)[0]
        return raw_single * (360.0 / COUNTS_PER_REV)

    def move_relative_counts(self, delta_counts: int):
        """0xC3 - Relative position control, raw counts."""
        d = self._cmd(0xC3, struct.pack("<i", delta_counts), expect_dlc=7)
        raw_single = struct.unpack_from("<H", d, 1)[0]
        return raw_single * (360.0 / COUNTS_PER_REV)

    # ------------------------------------------------------------------
    # Enable / disable / brake
    # ------------------------------------------------------------------

    def disable(self):
        """0xCF - Disable output, free state. This is the safe default state."""
        return self._cmd(0xCF, expect_dlc=8)

    def brake(self, engaged: bool):
        """0xCE - Holding brake on/off (if your driver variant has one wired)."""
        d = self._cmd(0xCE, bytes([0x01 if engaged else 0x00]), expect_dlc=2)
        return d[1] == 0x01

    def read_brake_state(self) -> bool:
        d = self._cmd(0xCE, bytes([0xFF]), expect_dlc=2)
        return d[1] == 0x01
