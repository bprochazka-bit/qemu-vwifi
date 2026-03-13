# ath9k Medium Controller

Web-based GUI for controlling the `ath9k_medium_hub` virtual wireless medium
via its Unix control socket. Mirrors the design of the React reference UI
(`medium-gui.jsx`) but runs as a standalone Python web application.

## Screenshot

The UI provides two views matching the medium hub's operating modes:

- **None (SNR Table)** — A pairwise matrix where you click any cell to set
  SNR and loss probability. Values are pushed to the hub immediately on save.

- **LOG Distance / Free Space** — A spatial canvas where you drag nodes to
  set their physical positions. Path loss is computed from distance. When you
  release a node, its position is sent to the hub.

## Requirements

All packages available from the Debian 13 repositories:

```bash
apt install python3-uvicorn python3-starlette
```

No internet access needed at runtime. All fonts, CSS, and JavaScript are
served locally from the `static/` directory.

## Quick Start

```bash
# 1. Start the medium hub with a control socket
./ath9k_medium_hub /tmp/ath9k.sock -c /tmp/ath9k.ctl

# 2. Start the web controller
python3 app.py --ctl /tmp/ath9k.ctl --host 0.0.0.0 --port 8080

# 3. Open in browser
xdg-open http://localhost:8080
```

### Command-Line Options

```
python3 app.py [OPTIONS]

  --ctl PATH    Path to hub control socket (default: /tmp/ath9k.ctl)
  --host ADDR   Listen address (default: 127.0.0.1)
  --port PORT   Listen port (default: 8080)
```

Or run directly with uvicorn:

```bash
# Set the control socket path via environment or edit app.py
uvicorn app:app --host 0.0.0.0 --port 8080
```

## How It Works

The web UI communicates with the hub through a REST API layer:

```
Browser  ──HTTP──▶  app.py (uvicorn)  ──Unix socket──▶  ath9k_medium_hub
```

### API Endpoints

| Method | Path              | Description                           |
|--------|-------------------|---------------------------------------|
| GET    | `/api/status`     | Hub connection, model, peers, stats   |
| POST   | `/api/model`      | `SET_MODEL` — change propagation model|
| POST   | `/api/snr`        | `SET_SNR` — set link SNR              |
| POST   | `/api/loss`       | `SET_LOSS` — set loss probability     |
| POST   | `/api/pos`        | `SET_POS` — set node position         |
| POST   | `/api/positions`  | Batch `SET_POS` for all nodes         |
| POST   | `/api/send`       | Send any raw command                  |
| GET    | `/api/peers`      | `LIST_PEERS`                          |
| GET    | `/api/dump_links` | `DUMP_LINKS`                          |

### Model Behavior

| UI Model          | Hub Model      | Behavior on Edit                            |
|-------------------|----------------|---------------------------------------------|
| None              | `snr_table`    | Click cell → modal → Save pushes `SET_SNR`/`SET_LOSS` immediately |
| LOG Distance Loss | `log_distance` | Drag node → release → `SET_POS` sent to hub |
| Free Space Loss   | `free_space`   | Drag node → release → `SET_POS` sent to hub |

When you click **Change Model**, the app sends `SET_MODEL <name>` to the hub.
For path-loss models, all node positions are also pushed on model change.

## Project Structure

```
medium-controller/
├── app.py                          # ASGI app (uvicorn + starlette)
├── README.md
└── static/
    ├── index.html                  # Single-page app (all JS inline)
    ├── css/
    │   ├── fonts.css               # @font-face declarations
    │   └── style.css               # All styles
    └── fonts/
        ├── dm-sans-latin-*.woff2   # DM Sans 400/500/600
        ├── jetbrains-mono-*.woff2  # JetBrains Mono 400/500/600
        └── sora-latin-*.woff2      # Sora 400/600/700
```

## Terminal Integration (Future: xterm.js)

The current UI has placeholder terminals that simulate output locally. To
connect them to actual VM consoles using [xterm.js](https://xtermjs.org/),
you'll need the following changes:

### What Needs to Change on the VMs

1. **Serial console on a Unix socket or virtio-serial port**

   Each QEMU VM needs a character device exposed for terminal access. The
   simplest approach is a Unix socket or a virtio-serial channel:

   ```bash
   # Option A: Unix socket serial console
   qemu-system-x86_64 ... \
     -chardev socket,id=term,path=/tmp/vm-sta1-term.sock,server=on,wait=off \
     -serial chardev:term

   # Option B: virtio-serial channel (guest needs agent)
   qemu-system-x86_64 ... \
     -device virtio-serial \
     -chardev socket,id=term,path=/tmp/vm-sta1-term.sock,server=on,wait=off \
     -device virtserialport,chardev=term,name=org.qemu.terminal
   ```

2. **Guest serial login (Option A)**

   Enable `getty` on the serial console inside each VM:

   ```bash
   # systemd-based guests:
   systemctl enable serial-getty@ttyS0.service
   systemctl start serial-getty@ttyS0.service

   # Or add to /etc/inittab for sysvinit:
   # T0:23:respawn:/sbin/getty -L ttyS0 115200 vt100
   ```

3. **Guest kernel command line**

   Ensure the kernel has console output on the serial port:

   ```
   console=ttyS0,115200
   ```

### What Needs to Change on the Host / Controller

1. **Install xterm.js locally** (for offline use):

   ```bash
   # Download the npm package
   npm pack xterm
   npm pack xterm-addon-fit
   # Extract and copy to static/vendor/
   ```

2. **Add a WebSocket proxy in `app.py`**

   The controller needs a WebSocket endpoint that bridges between the
   browser (xterm.js) and the QEMU serial socket. Using starlette's
   WebSocket support:

   ```python
   from starlette.websockets import WebSocket
   import asyncio

   async def ws_terminal(websocket: WebSocket):
       node_id = websocket.path_params["node_id"]
       sock_path = f"/tmp/vm-{node_id}-term.sock"
       await websocket.accept()

       # Connect to QEMU serial socket
       reader, writer = await asyncio.open_unix_connection(sock_path)

       async def read_from_vm():
           while True:
               data = await reader.read(4096)
               if not data:
                   break
               await websocket.send_bytes(data)

       async def write_to_vm():
           while True:
               data = await websocket.receive_bytes()
               writer.write(data)
               await writer.drain()

       await asyncio.gather(read_from_vm(), write_to_vm())
   ```

3. **Map node IDs to socket paths**

   You'll need a configuration mapping (or naming convention) so the
   controller knows which Unix socket corresponds to which node. The
   simplest approach is a convention like `/tmp/vm-{node_id}-term.sock`,
   which you'd set in your QEMU launch scripts.

4. **xterm.js frontend integration**

   Replace the placeholder terminal widget with:

   ```javascript
   import { Terminal } from 'xterm';
   import { FitAddon } from 'xterm-addon-fit';

   const term = new Terminal({ cursorBlink: true });
   const fit = new FitAddon();
   term.loadAddon(fit);
   term.open(document.getElementById(`termBody_${nodeId}`));
   fit.fit();

   const ws = new WebSocket(`ws://${location.host}/ws/terminal/${nodeId}`);
   ws.onmessage = (e) => term.write(e.data);
   term.onData((data) => ws.send(data));
   ```

### Summary of Required Changes

| Component          | Change                                              |
|--------------------|-----------------------------------------------------|
| QEMU launch script | Add `-chardev socket` + `-serial chardev:term`      |
| Guest VM           | Enable `serial-getty@ttyS0`, add `console=ttyS0` to kernel cmdline |
| `app.py`           | Add WebSocket route proxying to QEMU serial sockets |
| `index.html`       | Replace placeholder terminals with xterm.js widget  |
| `static/vendor/`   | Bundle xterm.js + xterm-addon-fit locally            |
| Config/convention  | Map node_id → `/tmp/vm-{node_id}-term.sock`         |
