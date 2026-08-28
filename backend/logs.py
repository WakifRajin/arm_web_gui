"""
logs.py

One logger ("arm") used by everything in the backend. Two sinks:

  - RotatingFileHandler -> arm_logs/arm_<date>.log on disk, so a
    session can be reviewed after the fact (or attached to a bug
    report) even if nobody was watching the browser at the time.
  - RingBufferHandler -> an in-memory deque the web GUI polls /
    streams over the websocket, each entry tagged with level,
    timestamp, and (when known) which joint it's about.

Call log_event(level, message, joint=None) from anywhere in the
backend instead of touching the logger directly -- it keeps the
joint tag consistent so the frontend can filter by joint.
"""

import logging
import logging.handlers
import os
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_logs")
os.makedirs(LOG_DIR, exist_ok=True)

RING_CAPACITY = 2000


@dataclass
class LogEntry:
    ts: float
    level: str
    message: str
    joint: Optional[str] = None

    def to_dict(self):
        return {"ts": self.ts, "level": self.level, "message": self.message, "joint": self.joint}


class RingBufferHandler(logging.Handler):
    def __init__(self, capacity: int = RING_CAPACITY):
        super().__init__()
        self.buffer: Deque[LogEntry] = deque(maxlen=capacity)
        self._subscribers = []  # list of asyncio.Queue-like objects with put_nowait

    def emit(self, record: logging.LogRecord):
        joint = getattr(record, "joint", None)
        entry = LogEntry(ts=record.created, level=record.levelname, message=record.getMessage(), joint=joint)
        self.buffer.append(entry)
        for q in list(self._subscribers):
            try:
                q.put_nowait(entry)
            except Exception:
                pass

    def subscribe(self, q):
        self._subscribers.append(q)

    def unsubscribe(self, q):
        if q in self._subscribers:
            self._subscribers.remove(q)

    def recent(self, limit: int = 200):
        items = list(self.buffer)[-limit:]
        return [e.to_dict() for e in items]


_logger = logging.getLogger("arm")
_logger.setLevel(logging.DEBUG)
_ring_handler = RingBufferHandler()
_ring_handler.setLevel(logging.DEBUG)

if not _logger.handlers:
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")

    file_handler = logging.handlers.RotatingFileHandler(
        os.path.join(LOG_DIR, "arm.log"), maxBytes=5_000_000, backupCount=5
    )
    file_handler.setFormatter(fmt)
    file_handler.setLevel(logging.DEBUG)

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(fmt)
    console_handler.setLevel(logging.INFO)

    _logger.addHandler(file_handler)
    _logger.addHandler(console_handler)
    _logger.addHandler(_ring_handler)


def log_event(level: str, message: str, joint: Optional[str] = None):
    lvl = getattr(logging, level.upper(), logging.INFO)
    _logger.log(lvl, message, extra={"joint": joint})


def get_ring_handler() -> RingBufferHandler:
    return _ring_handler


def recent_logs(limit: int = 200):
    return _ring_handler.recent(limit)
