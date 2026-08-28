"""
can_diag.py

Standalone CAN-*interface* diagnostics and failsafes -- deliberately kept
independent of worker.py/arm_controller.py's python-can `bus` object.
Everything here shells out to Linux SocketCAN tooling (`ip link`,
`candump`) through its own short-lived subprocess/socket, so:

  - a hung, bus-off, or unplugged bus can never block a diagnostic read
    (it doesn't go anywhere near the Worker thread's queues), and
  - a `candump` capture never contends with the Worker for `self.bus` --
    SocketCAN happily supports multiple independent listeners on the
    same interface, and candump only reads.

Requires on the host machine:
  - a SocketCAN interface (default "can0" -- set "can_interface" in
    gui_joints.json if yours is different, e.g. "can1", "vcan0")
  - `can-utils` for candump:      sudo apt install can-utils
  - permission to run `ip link set <iface> up/down`. Easiest options,
    pick one:
      1. run the backend as root (fine on a dedicated bench Pi), or
      2. grant the interpreter CAP_NET_ADMIN:
           sudo setcap cap_net_admin+ep $(readlink -f $(which python3))
      3. add a narrowly-scoped passwordless sudoers rule, e.g.
         (via `sudo visudo -f /etc/sudoers.d/arm-can`):
           youruser ALL=(root) NOPASSWD: /sbin/ip link set can0 up *, \\
                                          /sbin/ip link set can0 down
  We try the bare command first and fall back to a *non-interactive*
  `sudo -n` (never one that could sit waiting on a password prompt)
  so option 1 or 2 need nothing extra, and option 3 covers the rest.
"""

import asyncio
import shutil
import subprocess


class CanDiagError(Exception):
    pass


def _run(cmd: list, timeout: float = 5.0) -> str:
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        raise CanDiagError(f"'{cmd[0]}' not found on this machine. "
                            f"Install it (`sudo apt install can-utils iproute2`).")
    except subprocess.TimeoutExpired:
        raise CanDiagError(f"'{' '.join(cmd)}' timed out after {timeout}s.")

    if result.returncode != 0:
        stderr = (result.stderr or "").strip()
        looks_like_permissions = "Operation not permitted" in stderr or "RTNETLINK" in stderr
        if looks_like_permissions and cmd[0] != "sudo":
            try:
                result2 = subprocess.run(["sudo", "-n"] + cmd, capture_output=True, text=True, timeout=timeout)
                if result2.returncode == 0:
                    return result2.stdout
                stderr = (result2.stderr or stderr).strip()
            except FileNotFoundError:
                pass
        raise CanDiagError(stderr or f"'{' '.join(cmd)}' failed (exit {result.returncode}).")
    return result.stdout


def interface_up(iface: str, bitrate: int) -> str:
    """Bring the CAN link up at `bitrate`. Safe to call on an
    already-up interface -- brings it down first so a stale bitrate
    setting can't linger."""
    _run(["ip", "link", "set", iface, "down"], timeout=3.0)
    _run(["ip", "link", "set", iface, "up", "type", "can", "bitrate", str(bitrate)], timeout=5.0)
    return f"{iface} is up at {bitrate} bps."


def interface_down(iface: str) -> str:
    """Hard kill: drops the CAN link at the OS level. This is a deeper
    stop than software E-STOP -- useful if a motor's firmware is stuck
    ignoring disable frames, since no CAN traffic can move at all once
    the link is down. Call site (main.py) latches E-STOP first."""
    _run(["ip", "link", "set", iface, "down"], timeout=5.0)
    return f"{iface} is down."


def interface_status(iface: str) -> dict:
    out = _run(["ip", "-details", "-statistics", "link", "show", iface], timeout=3.0)
    status = {
        "state": "UNKNOWN", "bitrate": None,
        "rx_errors": None, "tx_errors": None,
        "bus_off": "BUS-OFF" in out, "error_passive": "ERROR-PASSIVE" in out,
        "raw": out.strip(),
    }
    for line in out.splitlines():
        stripped = line.strip()
        tokens = stripped.split()
        if status["state"] == "UNKNOWN":
            for tok in tokens:
                if tok in ("UP", "DOWN", "UNKNOWN"):
                    status["state"] = tok
                    break
        if "bitrate" in tokens:
            try:
                status["bitrate"] = int(tokens[tokens.index("bitrate") + 1])
            except (ValueError, IndexError):
                pass
    lines = out.splitlines()
    for i, line in enumerate(lines):
        head = line.strip()
        if head.startswith("RX:") and i + 1 < len(lines):
            vals = lines[i + 1].split()
            if len(vals) >= 4:
                status["rx_errors"] = vals[3]
        if head.startswith("TX:") and i + 1 < len(lines):
            vals = lines[i + 1].split()
            if len(vals) >= 4:
                status["tx_errors"] = vals[3]
    return status


async def candump_snapshot(iface: str, duration_s: float = 2.0, max_lines: int = 400) -> list:
    """Runs `candump` for a fixed window and returns the captured lines
    (raw frame dumps -- id + data bytes), for eyeballing what's actually
    on the bus when something looks wrong. Independent socket -- never
    touches the Worker's python-can bus object."""
    if not shutil.which("candump"):
        raise CanDiagError("`candump` not found. Install can-utils: sudo apt install can-utils")

    proc = await asyncio.create_subprocess_exec(
        "candump", "-n", str(max_lines), iface,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
    )
    lines = []
    try:
        async def _read():
            while True:
                line = await proc.stdout.readline()
                if not line:
                    break
                lines.append(line.decode(errors="replace").rstrip())
                if len(lines) >= max_lines:
                    break
        await asyncio.wait_for(_read(), timeout=duration_s)
    except asyncio.TimeoutError:
        pass
    finally:
        if proc.returncode is None:
            proc.terminate()
            try:
                await asyncio.wait_for(proc.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                proc.kill()
    return lines
