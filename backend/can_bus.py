"""
can_bus.py

One place to configure how you connect to the CAN bus, so every
script uses the same setup. Edit CHANNEL/BUSTYPE for your adapter.

Common setups:
  Linux + native CAN adapter (e.g. via SocketCAN):
      bustype='socketcan', channel='can0'
      (bring the interface up first: sudo ip link set can0 up type can bitrate 1000000)

  USB-CAN adapter presenting as serial (slcan protocol):
      bustype='slcan', channel='/dev/ttyUSB0'

  PCAN, Kvaser, Vector, etc: see python-can docs for the right bustype string.
"""

import can

CHANNEL = "can0"
BUSTYPE = "socketcan"
BITRATE = 1_000_000  # matches the protocol doc's default 1 MHz


def get_bus() -> can.BusABC:
    return can.interface.Bus(channel=CHANNEL, bustype=BUSTYPE, bitrate=BITRATE)
