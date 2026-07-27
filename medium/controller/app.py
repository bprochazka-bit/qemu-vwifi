#!/usr/bin/env python3
"""
vwifi Medium Controller

Web-based GUI for the vwifi-medium control socket.
Designed for offline Debian 13 -- all assets served locally.

Install (Debian 13):
    apt install python3-uvicorn python3-starlette

Run:
    python3 app.py --ctl /tmp/vwifi.ctl [--host 0.0.0.0] [--port 8080]
"""

import argparse
import json
import os
import socket
import sys
import threading
from pathlib import Path

import uvicorn
from starlette.applications import Starlette
from starlette.responses import HTMLResponse, JSONResponse
from starlette.routing import Route, Mount
from starlette.staticfiles import StaticFiles

BASE_DIR = Path(__file__).resolve().parent

# ---------------------------------------------------------------------------
#  Hub control socket client
# ---------------------------------------------------------------------------

class MediumHubClient:
    """Thread-safe client for the vwifi-medium control socket.

    The hub keeps control connections alive (it never closes them after
    a response).  To avoid blocking on recv() until a timeout expires,
    we piggyback a QUIT command after every real command.  The hub
    processes both, sends the response, then sends "OK bye\\n" and
    closes the socket -- so our recv() returns immediately.
    """

    def __init__(self, ctl_path: str):
        self.ctl_path = ctl_path
        self._lock = threading.Lock()

    def _read_all(self, sock: socket.socket) -> str:
        """Read until the socket is closed (hub closes after QUIT)."""
        chunks = []
        while True:
            try:
                data = sock.recv(8192)
                if not data:
                    break
                chunks.append(data.decode("utf-8", errors="replace"))
            except socket.timeout:
                break
        return "".join(chunks)

    @staticmethod
    def _strip_quit_response(raw: str) -> str:
        """Remove the trailing 'OK bye' line left by our QUIT command."""
        lines = raw.rstrip("\n").split("\n")
        if lines and lines[-1].strip() == "OK bye":
            lines = lines[:-1]
        return "\n".join(lines)

    def send(self, cmd: str) -> str:
        """Send one command, return its response."""
        with self._lock:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            try:
                sock.connect(self.ctl_path)
                sock.sendall((cmd.strip() + "\nQUIT\n").encode())
                raw = self._read_all(sock)
                return self._strip_quit_response(raw)
            except (ConnectionRefusedError, FileNotFoundError, OSError) as e:
                return f"ERR connection failed: {e}"
            finally:
                sock.close()

    def send_multi(self, commands: list) -> str:
        """Send multiple commands in one connection, return combined output."""
        with self._lock:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            try:
                sock.connect(self.ctl_path)
                payload = "\n".join(c.strip() for c in commands) + "\nQUIT\n"
                sock.sendall(payload.encode())
                raw = self._read_all(sock)
                return self._strip_quit_response(raw)
            except (ConnectionRefusedError, FileNotFoundError, OSError) as e:
                return f"ERR connection failed: {e}"
            finally:
                sock.close()

    @staticmethod
    def _parse_peer_line(stripped: str) -> dict | None:
        """Parse a single indented peer line into a dict, or None."""
        parts = stripped.split()
        if len(parts) < 2:
            return None
        # The second field must be online/offline to confirm a real node
        if parts[1] not in ("online", "offline"):
            return None
        node = {"id": parts[0], "status": parts[1]}
        for part in parts[2:]:
            if part.startswith("macs="):
                node["macs"] = part[5:].strip("[]")
            elif part.startswith("pos="):
                pos_str = part[4:]
                if pos_str == "none":
                    node["pos"] = None
                else:
                    try:
                        node["pos"] = [float(c) for c in pos_str.strip("()").split(",")]
                    except ValueError:
                        node["pos"] = None
            elif "=" in part:
                k, v = part.split("=", 1)
                try:
                    node[k] = float(v)
                except ValueError:
                    node[k] = v
        return node

    def get_full_status(self) -> dict:
        """Fetch GET_MODEL + LIST_PEERS + STATS in one connection.

        Returns a dict ready for JSON serialisation.
        """
        raw = self.send_multi(["GET_MODEL", "LIST_PEERS", "STATS"])

        model_info = {"type": "none", "params": {}}
        peers = []
        stats_raw = ""

        for line in raw.split("\n"):
            stripped = line.strip()
            if not stripped:
                continue

            # GET_MODEL: "OK model=xxx path_loss_exp=... ..."
            if stripped.startswith("OK ") and "model=" in stripped:
                for part in stripped[3:].split():
                    if "=" in part:
                        k, v = part.split("=", 1)
                        if k == "model":
                            model_info["type"] = v
                        else:
                            try:
                                model_info["params"][k] = float(v)
                            except ValueError:
                                model_info["params"][k] = v
                continue

            # STATS: "OK fwd=... drop=... peers=... nodes=... links=..."
            if stripped.startswith("OK ") and "fwd=" in stripped:
                stats_raw = stripped
                continue

            # LIST_PEERS summary line: "OK N nodes (M peers online)"
            # Skip any other OK/ERR lines
            if stripped.startswith("OK") or stripped.startswith("ERR"):
                continue

            # Remaining lines should be peer data
            node = self._parse_peer_line(stripped)
            if node:
                peers.append(node)

        return {
            "connected": True,
            "model": model_info,
            "peers": peers,
            "stats": stats_raw,
        }

    def is_connected(self) -> bool:
        try:
            return self.send("STATS").startswith("OK")
        except Exception:
            return False

    def list_peers(self) -> list:
        """Parse LIST_PEERS into a list of node dicts."""
        raw = self.send("LIST_PEERS")
        peers = []
        for line in raw.split("\n"):
            stripped = line.strip()
            if not stripped or stripped.startswith("OK") or stripped.startswith("ERR"):
                continue
            node = self._parse_peer_line(stripped)
            if node:
                peers.append(node)
        return peers


# ---------------------------------------------------------------------------
#  Global state
# ---------------------------------------------------------------------------

hub: MediumHubClient | None = None

# ---------------------------------------------------------------------------
#  Route handlers
# ---------------------------------------------------------------------------

async def index(request):
    return HTMLResponse((BASE_DIR / "static" / "index.html").read_text())


async def api_status(request):
    if hub is None:
        return JSONResponse({"connected": False, "error": "No control socket configured"})
    try:
        data = hub.get_full_status()
        data["ctl_path"] = hub.ctl_path
        return JSONResponse(data)
    except Exception as e:
        return JSONResponse({"connected": False, "error": str(e)})


async def api_send(request):
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    body = await request.json()
    cmd = body.get("cmd", "").strip()
    if not cmd:
        return JSONResponse({"ok": False, "error": "Empty command"}, status_code=400)
    resp = hub.send(cmd)
    return JSONResponse({"ok": resp.startswith("OK"), "response": resp})


async def api_set_model(request):
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    body = await request.json()
    model_name = body.get("model", "").strip()
    if model_name not in ("none", "snr_table", "log_distance", "free_space"):
        return JSONResponse({"ok": False, "error": f"Unknown model: {model_name}"}, status_code=400)
    resp = hub.send(f"SET_MODEL {model_name}")
    return JSONResponse({"ok": resp.startswith("OK"), "response": resp})


async def api_set_snr(request):
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    body = await request.json()
    a, b, snr = body.get("a"), body.get("b"), body.get("snr")
    if not a or not b or snr is None:
        return JSONResponse({"ok": False, "error": "Missing a, b, or snr"}, status_code=400)
    resp = hub.send(f"SET_SNR {a} {b} {snr}")
    return JSONResponse({"ok": resp.startswith("OK"), "response": resp})


async def api_set_loss(request):
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    body = await request.json()
    a, b, loss = body.get("a"), body.get("b"), body.get("loss")
    if not a or not b or loss is None:
        return JSONResponse({"ok": False, "error": "Missing a, b, or loss"}, status_code=400)
    resp = hub.send(f"SET_LOSS {a} {b} {loss}")
    return JSONResponse({"ok": resp.startswith("OK"), "response": resp})


async def api_set_pos(request):
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    body = await request.json()
    node, x, y = body.get("node"), body.get("x"), body.get("y")
    z = body.get("z", 0.0)
    if not node or x is None or y is None:
        return JSONResponse({"ok": False, "error": "Missing node, x, or y"}, status_code=400)
    resp = hub.send(f"SET_POS {node} {x} {y} {z}")
    return JSONResponse({"ok": resp.startswith("OK"), "response": resp})


async def api_set_positions(request):
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    body = await request.json()
    positions = body.get("positions", [])
    commands = [f"SET_POS {p['node']} {p.get('x',0)} {p.get('y',0)} {p.get('z',0)}"
                for p in positions]
    raw = hub.send_multi(commands)
    return JSONResponse({"ok": True, "response": raw})


async def api_clear_overrides(request):
    """Clear all SNR and LOSS overrides (used when switching to path-loss models)."""
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    # Get current peers to know which pairs to clear
    peers = hub.list_peers()
    ids = [p["id"] for p in peers]
    commands = []
    for i, a in enumerate(ids):
        for b in ids[i+1:]:
            commands.append(f"CLEAR_SNR {a} {b}")
            commands.append(f"CLEAR_LOSS {a} {b}")
    if commands:
        hub.send_multi(commands)
    return JSONResponse({"ok": True, "cleared": len(commands) // 2})


async def api_dump_links(request):
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    raw = hub.send("DUMP_LINKS")
    return JSONResponse({"ok": raw.startswith("OK"), "response": raw})


async def api_list_peers(request):
    if hub is None:
        return JSONResponse({"ok": False, "error": "No control socket"}, status_code=503)
    raw = hub.send("LIST_PEERS")
    peers = []
    for line in raw.split("\n"):
        stripped = line.strip()
        if stripped.startswith("OK") or stripped.startswith("ERR") or not stripped:
            continue
        node = hub._parse_peer_line(stripped)
        if node:
            peers.append(node)
    return JSONResponse({"ok": True, "peers": peers})


# ---------------------------------------------------------------------------
#  App
# ---------------------------------------------------------------------------

routes = [
    Route("/", index),
    Route("/api/status", api_status),
    Route("/api/send", api_send, methods=["POST"]),
    Route("/api/model", api_set_model, methods=["POST"]),
    Route("/api/snr", api_set_snr, methods=["POST"]),
    Route("/api/loss", api_set_loss, methods=["POST"]),
    Route("/api/pos", api_set_pos, methods=["POST"]),
    Route("/api/positions", api_set_positions, methods=["POST"]),
    Route("/api/clear_overrides", api_clear_overrides, methods=["POST"]),
    Route("/api/dump_links", api_dump_links),
    Route("/api/peers", api_list_peers),
    Mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static"),
]

app = Starlette(routes=routes)


def main():
    parser = argparse.ArgumentParser(description="vwifi Medium Controller Web UI")
    parser.add_argument("--ctl", default="/tmp/vwifi.ctl",
                        help="Path to the medium hub control socket")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Listen address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080,
                        help="Listen port (default: 8080)")
    args = parser.parse_args()

    global hub
    hub = MediumHubClient(args.ctl)

    print(f"vwifi Medium Controller")
    print(f"  Control socket: {args.ctl}")
    print(f"  Listening on:   http://{args.host}:{args.port}")
    print()

    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
