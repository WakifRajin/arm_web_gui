"""
can_link.py

Shared, non-blocking CAN transport for every motor on the arm.

WHY THIS FILE EXISTS
---------------------------------------------------------------------------
The old gim_motor.py sent a command and then BLOCKED the calling thread
waiting for that exact motor's reply (up to `timeout` seconds) before
returning -- classic request/reply. That's fine for one-off commissioning
calls, but the Worker's ramp loop called it every tick, for every active
joint, in a single thread. Six joints all slewing at once meant six
blocking round-trips serialized back-to-back *inside one 50ms tick*,
which is exactly the "somewhat laggy motor rotation" this rewrite fixes.

diff_wrist.py never has this problem: its WristDriver sends 0xC2 and
moves on immediately, while ONE dedicated background thread continuously
drains the bus and decodes whatever comes back into a shared, lock-
protected snapshot per motor. Every reader (the GUI refresh, the safety
loop, a status poll) just reads the latest snapshot -- nothing ever waits
on the bus. This file ports that architecture so it covers every joint on
the arm, not just two wrist motors.

CanLink is a SINGLETON per bus: one RX thread, one TX lock, shared by
every GimMotor facade (see gim_motor.py) so two joints can never contend
for the wire directly -- they contend for the (cheap, in-memory) lock
around `bus.send()` instead.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

import can

logger = logging.getLogger("arm")

COUNTS_PER_REV = 16384  # 14-bit encoder, same constant gim_motor.py always used

# ---- protocol command bytes (Custom CAN Communication Protocol) ---------
CMD_RESET = 0x00
CMD_VERSION = 0xA0
CMD_CURRENT = 0xA1
CMD_SPEED = 0xA2
CMD_ANGLES = 0xA3
CMD_STATUS = 0xAE
CMD_CLEAR_FAULT = 0xAF
CMD_MOTOR_INFO = 0xB0
CMD_SET_HOME = 0xB1
CMD_MAX_SPEED_POS = 0xB2
CMD_MAX_CURRENT = 0xB3
CMD_POS_KP = 0xB6
CMD_POS_KI = 0xB7
CMD_SET_ADDR = 0xBA
CMD_ABS_POSITION = 0xC2
CMD_REL_POSITION = 0xC3
CMD_RETURN_HOME = 0xC4
CMD_COMMS_TIMEOUT = 0xCD
CMD_BRAKE = 0xCE
CMD_DISABLE = 0xCF

PUBLIC_ADDR = 0xFF  # broadcast address used for discovery pings

FAULT_BITS = {0: "Voltage fault", 1: "Current fault", 2: "Temperature fault",
              3: "Encoder fault", 5: "Comms fault", 6: "Hardware fault",
              7: "Software fault"}

# A motor that hasn't answered ANY frame in this long is stale/disconnected,
# independent of the GUI's poll cadence -- ported from diff_wrist's
# RX_TIMEOUT_S, generalized to every joint instead of just the wrist pair.
RX_TIMEOUT_S = 0.6


def decode_fault(fault_code: int) -> str:
    if fault_code == 0:
        return "OK"
    bits = [name for bit, name in FAULT_BITS.items() if fault_code & (1 << bit)]
    return ", ".join(bits) if bits else f"Unknown fault code 0x{fault_code:02X}"


@dataclass
class MotorState:
    """Everything CanLink knows about one motor, updated only by the RX
    thread and read (under lock) by everyone else. Nothing here is ever
    populated by blocking on a reply -- it just reflects the most recent
    frame that arrived, whenever it arrived."""
    addr: int
    raw_single_deg: float = 0.0     # single-turn angle, motor's own 0-360
    raw_multi_deg: float = 0.0      # multi-turn angle, motor's own scale
    current_A: float = 0.0
    speed_rpm: float = 0.0
    voltage_V: float = 0.0
    bus_current_A: float = 0.0
    temperature_C: int = 0
    run_mode: int = 0
    fault_code: int = 0
    protocol_version: Optional[int] = None
    last_status_rx: float = 0.0     # monotonic time of last 0xAE
    last_pos_rx: float = 0.0        # monotonic time of last 0xA3/0xC2/0xC3
    last_current_rx: float = 0.0
    last_speed_rx: float = 0.0
    last_any_rx: float = 0.0

    def faults(self):
        return [name for bit, name in FAULT_BITS.items() if self.fault_code & (1 << bit)]

    def is_stale(self, now: float, timeout: float = RX_TIMEOUT_S) -> bool:
        return self.last_any_rx == 0.0 or (now - self.last_any_rx) > timeout


class CanLink:
    """One bus, one RX thread, non-blocking sends. Register every motor
    address you care about; everything else fans out from there."""

    def __init__(self, bus: can.BusABC):
        self.bus = bus
        self._tx_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self.motors: Dict[int, MotorState] = {}
        self.tx_errors = 0
        # Raw "last frame seen from this address" cache, for EVERY address
        # (registered joint or not) -- this is what discovery scanning and
        # address-assignment read from. They used to call bus.recv() on
        # their own private loop; now that the RX thread is the only
        # caller of bus.recv() (see _rx_loop), a second direct reader
        # would just steal frames from it unpredictably. Routing discovery
        # through the same cache keeps "one thread owns the bus" true
        # without losing any functionality.
        self.last_frame: Dict[int, dict] = {}
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="can-rx")
        self._rx_thread.start()

    # -- registration -------------------------------------------------
    def register(self, addr: int) -> MotorState:
        with self._state_lock:
            if addr not in self.motors:
                self.motors[addr] = MotorState(addr=addr)
            return self.motors[addr]

    def state(self, addr: int) -> Optional[MotorState]:
        with self._state_lock:
            m = self.motors.get(addr)
            # dataclasses aren't atomic snapshots across threads by
            # default, but every field here is a single int/float
            # assignment in the RX thread, so a torn read just means
            # "one field is one frame older" -- never inconsistent
            # enough to matter for degrees/amps/temperature display.
            return m

    # -- sending (never blocks on a reply) -----------------------------
    def send(self, dev: int, data: bytes) -> bool:
        msg = can.Message(arbitration_id=dev, data=data, is_extended_id=False)
        try:
            with self._tx_lock:
                self.bus.send(msg, timeout=0.02)
            return True
        except Exception as e:
            self.tx_errors += 1
            logger.error(f"CAN TX failed (dev {dev}): {e}")
            return False

    # -- RX: the ONLY thread that ever calls bus.recv() -----------------
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                msg = self.bus.recv(timeout=0.05)
                if msg is None:
                    continue
                # Decode every frame, not just ones from already-registered
                # motors -- discovery scanning and address-assignment rely
                # on seeing replies from addresses that aren't (yet) a
                # configured joint. _decode() itself splits the handling:
                # every address gets a last_frame entry, only registered
                # ones get full MotorState field decoding.
                self._decode(msg.arbitration_id, bytes(msg.data))
            except Exception:
                time.sleep(0.01)

    def _decode(self, dev: int, data: bytes):
        if not data:
            return
        code = data[0]
        now = time.monotonic()
        with self._state_lock:
            self.last_frame[dev] = {"code": code, "time": now, "data": data}
            if dev not in self.motors:
                return  # discovery/unassigned device -- scan_broadcast() etc. read last_frame directly
            m = self.motors[dev]
            try:
                m.last_any_rx = now
                if code == CMD_STATUS and len(data) >= 8:
                    prev = m.fault_code
                    m.voltage_V = struct.unpack_from("<H", data, 1)[0] * 0.01
                    m.bus_current_A = struct.unpack_from("<H", data, 3)[0] * 0.01
                    m.temperature_C = data[5]
                    m.run_mode = data[6]
                    m.fault_code = data[7]
                    m.last_status_rx = now
                    if m.fault_code and m.fault_code != prev:
                        logger.error(f"Motor {dev}: FAULT {decode_fault(m.fault_code)} "
                                     f"(0x{m.fault_code:02X})")
                elif code == CMD_ANGLES and len(data) >= 7:
                    raw_single = struct.unpack_from("<H", data, 1)[0]
                    raw_multi = struct.unpack_from("<i", data, 3)[0]
                    m.raw_single_deg = raw_single * (360.0 / COUNTS_PER_REV)
                    m.raw_multi_deg = raw_multi * (360.0 / COUNTS_PER_REV)
                    m.last_pos_rx = now
                elif code in (CMD_ABS_POSITION, CMD_REL_POSITION, CMD_RETURN_HOME) and len(data) >= 7:
                    raw_single = struct.unpack_from("<H", data, 1)[0]
                    m.raw_single_deg = raw_single * (360.0 / COUNTS_PER_REV)
                    m.last_pos_rx = now
                elif code == CMD_CURRENT and len(data) >= 5:
                    m.current_A = struct.unpack_from("<i", data, 1)[0] * 0.001
                    m.last_current_rx = now
                elif code == CMD_SPEED and len(data) >= 5:
                    m.speed_rpm = struct.unpack_from("<i", data, 1)[0] * 0.01
                    m.last_speed_rx = now
                elif code == CMD_CLEAR_FAULT and len(data) >= 2:
                    m.fault_code = data[1]
                    m.last_status_rx = now
                elif code == CMD_VERSION and len(data) >= 8:
                    m.protocol_version = data[7]
                    m.last_status_rx = now
                elif code == CMD_DISABLE and len(data) >= 8:
                    m.run_mode = data[6]
                    m.fault_code = data[7]
                    m.last_status_rx = now
                elif code == CMD_SET_HOME and len(data) >= 3:
                    off = struct.unpack_from("<H", data, 1)[0]
                    logger.warning(f"Motor {dev}: home written to driver (mech offset {off})")
            except struct.error:
                pass

    # -- non-blocking command helpers -----------------------------------
    # These fire the frame and return immediately -- the RESULT (new
    # angle, ack, whatever) shows up later as a normal decoded frame in
    # MotorState, same as every other status update. Callers that need
    # a *confirmed* result (set_home, assign_address) use
    # request_and_wait() below instead, sparingly, off the hot path.
    def read_version(self, dev):      self.send(dev, bytes([CMD_VERSION]))
    def read_status(self, dev):       self.send(dev, bytes([CMD_STATUS]))
    def read_angles(self, dev):       self.send(dev, bytes([CMD_ANGLES]))
    def read_current(self, dev):      self.send(dev, bytes([CMD_CURRENT]))
    def read_speed(self, dev):        self.send(dev, bytes([CMD_SPEED]))
    def clear_faults(self, dev):      self.send(dev, bytes([CMD_CLEAR_FAULT]))
    def disable(self, dev):           self.send(dev, bytes([CMD_DISABLE]))

    def set_max_speed_rpm(self, dev, rpm: float):
        self.send(dev, bytes([CMD_MAX_SPEED_POS]) + struct.pack("<i", int(round(rpm / 0.01))))

    def set_max_current_A(self, dev, amps: float):
        self.send(dev, bytes([CMD_MAX_CURRENT]) + struct.pack("<i", int(round(amps / 0.001))))

    def move_absolute_counts(self, dev, counts: int):
        self.send(dev, bytes([CMD_ABS_POSITION]) + struct.pack("<i", int(counts)))

    def move_relative_counts(self, dev, delta_counts: int):
        self.send(dev, bytes([CMD_REL_POSITION]) + struct.pack("<i", int(delta_counts)))

    def brake(self, dev, engaged: bool):
        self.send(dev, bytes([CMD_BRAKE, 0x01 if engaged else 0x00]))

    def set_driver_watchdog(self, dev, enable: bool, ms: int, action: int = 0x02):
        # 0xCD: [1]=enable [2..3]=duration ms [4]=action. Bit1 of action
        # opens the brake output on timeout. Requires protocol >= 0x08.
        # Ported straight from diff_wrist.py -- if the HOST dies or the
        # bus drops, the driver itself cuts torque after `ms` with no
        # host in the loop at all. The old arm backend never sent this,
        # so a wedged host previously left torque held until the (much
        # coarser) bus-watchdog on the *browser* side noticed.
        self.send(dev, bytes([CMD_COMMS_TIMEOUT, 1 if enable else 0]) +
                  struct.pack("<H", ms) + bytes([action]))

    def set_home(self, dev):
        self.send(dev, bytes([CMD_SET_HOME]))

    def return_home(self, dev):
        self.send(dev, bytes([CMD_RETURN_HOME]))

    # -- discovery / address assignment (see last_frame comment above) ---
    def send_broadcast(self, cmd: int):
        self.send(PUBLIC_ADDR, bytes([cmd]))

    def frame_since(self, dev: int, cmd: int, after: float) -> Optional[dict]:
        """Latest raw frame from `dev` with command byte `cmd`, if it
        arrived at or after monotonic time `after`. Used to detect an
        ack without a private bus.recv() loop."""
        with self._state_lock:
            f = self.last_frame.get(dev)
        if f and f["code"] == cmd and f["time"] >= after:
            return f
        return None

    def frames_since(self, after: float, exclude: Optional[set] = None) -> Dict[int, dict]:
        """All addresses that have sent ANY frame at/after monotonic
        time `after` -- what scan_broadcast() polls after pinging the
        broadcast address, in place of its old private recv() window."""
        exclude = exclude or set()
        with self._state_lock:
            return {addr: dict(f) for addr, f in self.last_frame.items()
                    if f["time"] >= after and addr not in exclude}

    # -- the one place we DO still wait for a reply ----------------------
    def request_and_wait(self, dev: int, cmd: int, payload: bytes = b"",
                          expect_dlc: Optional[int] = None, timeout: float = 1.0):
        """Blocking request/reply, for rare commissioning-only calls
        (set_home confirmation, address assignment) where the caller
        genuinely needs to know the device accepted it before moving on,
        and where "blocks the calling thread for up to `timeout`s" is
        acceptable because it's a one-off UI action, not something in
        the 50Hz motion loop. NEVER call this from the ramp tick path.

        Reads the shared MotorState instead of doing a private recv, so
        it doesn't race the RX thread -- it just polls the cache until a
        frame newer than `deadline_start` arrives (or the deadline passes).
        """
        start = time.monotonic()
        self.send(dev, bytes([cmd]) + payload)
        deadline = start + timeout
        while time.monotonic() < deadline:
            st = self.state(dev)
            if st is not None:
                newest = max(st.last_status_rx, st.last_pos_rx, st.last_current_rx,
                             st.last_speed_rx, st.last_any_rx)
                if newest >= start:
                    return st
            time.sleep(0.01)
        raise TimeoutError(f"No reply from device {dev} for command 0x{cmd:02X} "
                            f"within {timeout}s")

    # -- background poller: keeps every registered motor's cache warm ---
    # This is what replaces "read_status()/read_position() send a request
    # and block for the reply." One thread continuously round-robins
    # requests to every registered motor; read_status()/read_position()
    # in gim_motor.py just return whatever's already in the cache. That
    # means ArmController.poll_one() -- called from the Worker loop for
    # every joint, every poll cycle -- never touches the bus at all
    # anymore, so the browser can be told about a new angle far more
    # often than the old poll_interval_s allowed (see worker.py).
    def start_polling(self, request_interval_s: float = 0.004):
        self._poll_thread = threading.Thread(
            target=self._poll_loop, args=(request_interval_s,), daemon=True, name="can-poll")
        self._poll_thread.start()

    def _poll_loop(self, interval: float):
        cycle = (self.read_status, self.read_angles, self.read_current)
        i = 0
        while not self._stop.is_set():
            with self._state_lock:
                devs = list(self.motors.keys())
            if devs:
                dev = devs[i % len(devs)]
                fn = cycle[(i // len(devs)) % len(cycle)]
                fn(dev)
                i += 1
            time.sleep(interval)

    def close(self):
        self._stop.set()
        try:
            for dev in list(self.motors.keys()):
                self.disable(dev)
            time.sleep(0.05)
        except Exception:
            pass
        self._rx_thread.join(timeout=0.5)
        try:
            self.bus.shutdown()
        except Exception:
            pass
