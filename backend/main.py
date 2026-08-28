"""
main.py

FastAPI app. This is the ONLY new code that talks CAN-adjacent
concepts over the network -- it never touches the bus itself, only
the Worker's two queues (see worker.py). Run with:

    uvicorn main:app --host 0.0.0.0 --port 8000

then open http://<this-machine-ip>:8000 in a browser on the same
network as the rover / bench setup.
"""

import asyncio
import json
import queue
import threading
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import os

from arm_controller import ArmController
from worker import Worker
from logs import log_event, recent_logs, get_ring_handler
import can_diag

app = FastAPI(title="Team Interplanetar - Rover Arm Control")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

arm = ArmController()
cmd_q: "queue.Queue" = queue.Queue()
status_q: "queue.Queue" = queue.Queue()
worker = Worker(arm, cmd_q, status_q)

CAN_IFACE = arm.config.get("can_interface", "can0")
CAN_BITRATE = arm.config.get("can_bitrate", 1000000)

FRONTEND_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "frontend")

_ws_clients: set[WebSocket] = set()
_latest_status: dict[str, dict] = {}      # joint name -> latest status dict
_latest_discovered: list[dict] = []
_status_lock = threading.Lock()


@app.on_event("startup")
async def startup():
    worker.start()
    asyncio.create_task(_pump_status_queue())
    log_event("info", "Backend started.")


@app.on_event("shutdown")
async def shutdown():
    worker.stop()


async def _pump_status_queue():
    """Drains the worker's status_q (filled from a plain thread) and
    fans each message out to every connected websocket client."""
    loop = asyncio.get_event_loop()
    while True:
        msg = await loop.run_in_executor(None, _blocking_get, status_q)
        if msg is None:
            await asyncio.sleep(0.01)
            continue
        if msg.get("type") == "status":
            with _status_lock:
                _latest_status[msg["name"]] = msg
        elif msg.get("type") == "discovered":
            global _latest_discovered
            _latest_discovered = msg["devices"]
        await _broadcast(msg)


def _blocking_get(q: "queue.Queue"):
    try:
        return q.get(timeout=0.2)
    except queue.Empty:
        return None


async def _broadcast(msg: dict):
    dead = []
    for ws in list(_ws_clients):
        try:
            await ws.send_json(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        _ws_clients.discard(ws)


# ---- websocket: commands in, live status + logs out --------------------
@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    _ws_clients.add(websocket)
    # Bring a fresh client up to date immediately instead of waiting
    # for the next poll cycle.
    await websocket.send_json({"type": "config", "config": arm.config})
    with _status_lock:
        for s in _latest_status.values():
            await websocket.send_json(s)
    await websocket.send_json({"type": "discovered", "devices": _latest_discovered})
    for entry in recent_logs(200):
        await websocket.send_json({"type": "log", **entry})

    log_handler = get_ring_handler()
    log_q: "asyncio.Queue" = asyncio.Queue()
    loop = asyncio.get_event_loop()

    class _Bridge:
        def put_nowait(self, entry):
            loop.call_soon_threadsafe(log_q.put_nowait, entry)

    bridge = _Bridge()
    log_handler.subscribe(bridge)

    async def _pump_logs():
        while True:
            entry = await log_q.get()
            try:
                await websocket.send_json({"type": "log", **entry.to_dict()})
            except Exception:
                break

    log_task = asyncio.create_task(_pump_logs())

    try:
        while True:
            raw = await websocket.receive_text()
            try:
                cmd = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if cmd.get("type") == "estop":
                worker.estop_event.set()
            else:
                cmd_q.put(cmd)
    except WebSocketDisconnect:
        pass
    finally:
        _ws_clients.discard(websocket)
        log_handler.unsubscribe(bridge)
        log_task.cancel()


# ---- REST: config + one-shot actions that don't need a live socket -----
@app.get("/api/config")
def get_config():
    return arm.config


@app.get("/api/logs")
def get_logs(limit: int = 200):
    return recent_logs(limit)


@app.post("/api/estop")
def rest_estop():
    worker.estop_event.set()
    return {"ok": True}


@app.post("/api/scan")
def rest_scan():
    cmd_q.put({"type": "_noop"})  # keep the worker loop responsive
    return {"discovered": _latest_discovered}


# ---- CAN interface failsafes / diagnostics -------------------------------
# Deliberately implemented in can_diag.py, independent of the Worker's
# python-can `bus` object -- see that module's docstring. These never
# touch cmd_q/status_q except can_up/can_down nudging the worker
# afterward, so a wedged bus can't block a diagnostic read.
@app.get("/api/can/status")
def can_status():
    try:
        return {"ok": True, "iface": CAN_IFACE, **can_diag.interface_status(CAN_IFACE)}
    except can_diag.CanDiagError as e:
        return {"ok": False, "iface": CAN_IFACE, "error": str(e)}


@app.post("/api/can/up")
def can_up():
    try:
        msg = can_diag.interface_up(CAN_IFACE, CAN_BITRATE)
        log_event("warning", f"CAN: {msg}")
        cmd_q.put({"type": "reconnect"})
        return {"ok": True, "message": msg}
    except can_diag.CanDiagError as e:
        log_event("error", f"CAN up failed: {e}")
        return {"ok": False, "error": str(e)}


@app.post("/api/can/down")
def can_down():
    # Hard kill switch, one level below software E-STOP: latch E-STOP
    # first (disables every joint and freezes every ramp over the still-
    # live bus) *then* drop the OS-level link, so nothing is left
    # mid-command when the bus disappears out from under it.
    worker.estop_event.set()
    try:
        msg = can_diag.interface_down(CAN_IFACE)
        log_event("warning", f"CAN: {msg}")
        return {"ok": True, "message": msg}
    except can_diag.CanDiagError as e:
        log_event("error", f"CAN down failed: {e}")
        return {"ok": False, "error": str(e)}


@app.get("/api/can/candump")
async def can_candump(duration: float = 2.0):
    duration = max(0.5, min(duration, 10.0))
    try:
        lines = await can_diag.candump_snapshot(CAN_IFACE, duration_s=duration)
        return {"ok": True, "lines": lines}
    except can_diag.CanDiagError as e:
        return {"ok": False, "error": str(e)}


# ---- frontend (static) ---------------------------------------------------
app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")


@app.get("/")
def index():
    return FileResponse(os.path.join(FRONTEND_DIR, "index.html"))
