# vwifi Windows driver — development plan

Master plan for the Windows-guest virtual Wi-Fi driver and its companion
QEMU device. Every phase below has a scope, concrete deliverables, and
testable exit criteria. The phases are ordered so each one produces
something useful standalone, even if the later phases are never built.

> **Where these files live now.** This plan was written when the device
> and driver were their own repository (`qemu-windows-vwifi`), and its
> deliverables sections still describe that layout. Everything has since
> been consolidated into `qemu-vwifi`:
>
> | Plan says | Now at |
> |---|---|
> | `qemu/` | `devices/vwifi/src/` |
> | `driver/src/`, `driver/inf/` | `drivers/vwifi/windows/` |
> | `abi/vwifi_abi.h` | `abi/vwifi_abi.h` (unchanged — top of the repo) |
> | the stub `ath9k_medium.h` | gone; the device includes `abi/vwifi.h` |
>
> The technical content of the plan is unaffected — only the paths.

## Table of contents

- [Goals and non-goals](#goals-and-non-goals)
- [System overview](#system-overview)
- [Architecture decisions](#architecture-decisions)
- [Phase 0 — Pipeline validation](#phase-0--pipeline-validation)
- [Phase 1 — Probe-only driver and device](#phase-1--probe-only-driver-and-device)
- [Phase 1.5 — Monitor mode and injection](#phase-15--monitor-mode-and-injection)
- [Phase 2 — Scan and BSS discovery](#phase-2--scan-and-bss-discovery)
- [Phase 3 — Associate, connect, data path](#phase-3--associate-connect-data-path)
- [Phase 4 — Security: WPA2 then WPA3](#phase-4--security-wpa2-then-wpa3)
- [Phase 5 — Polish and realism](#phase-5--polish-and-realism)
- [Cross-cutting concerns](#cross-cutting-concerns)
- [Risk register](#risk-register)
- [References](#references)


## Goals and non-goals

**Goals**

- A Windows 10 and Windows 11 guest sees a Wi-Fi adapter indistinguishable
  in daily use from a real one: `netsh wlan show interfaces`, the
  system tray flyout, Settings → Network & Internet → Wi-Fi, `wlansvc`,
  `NetworkManager`-equivalent apps, and third-party tools all work.
- The adapter participates in the same virtual medium as existing
  Linux VMs running `ath9k-virt` and the host `ath9k_medium_host`
  kernel module.
- Wireshark (via Npcap) can capture raw 802.11 + radiotap inside the
  Windows guest and can inject.
- Security: WPA2-PSK must work end-to-end; WPA3-SAE as a stretch goal.

**Non-goals**

- WHQL certification. The driver is intended for research and
  development; distribution without test-signing is explicitly out
  of scope.
- Windows 11 WiFiCx. A second driver using the newer WiFiCx model
  is a future possibility but not this plan's target; WDI covers
  both Windows 10 and Windows 11.
- Real radio frequency modeling. The medium is a lossless fan-out
  with channel filtering; physics stays out of scope.
- Hyper-V integration. QEMU/KVM and plain QEMU only.
- Virtio-based design. The device is PCI with custom descriptor
  rings, not a virtio device — to give the driver clean per-opcode
  semantics matching the WDI task model.


## System overview

```
Windows 10/11 guest VM
  Wi-Fi UI / wlansvc / netsh / Wireshark+Npcap
    │ (Native 802.11 OIDs, WDI tasks)
  Microsoft WLAN component
    │
  vwifi.sys  ← our driver
    │ MMIO + MSI-X + DMA (PCI)
  ──────────── guest/host boundary ────────────
  vwifi-virt  ← our QEMU device
    │ length-prefixed messages over Unix socket
  ath9k_medium_hub_scalable  ← unchanged
    ├── other QEMU VMs running ath9k-virt (Linux)
    ├── host Linux wlanX via ath9k_medium_host.ko
    └── (optional) real radio via ath9k_phys_bridge
```

The hub and everything to its right already exists in the qemu-vwifi
repository. This plan covers the two new components (QEMU device and
Windows driver) plus their shared ABI.


## Architecture decisions

The following decisions are locked in and referenced throughout.

1. **WDI, not Native 802.11, not WiFiCx.** Native 802.11 is deprecated
   since Windows 10. WiFiCx is Windows 11 only. WDI is the only single-
   driver answer that runs on Windows 10 *and* Windows 11.
2. **Separate QEMU device** — new `vwifi-virt`, not a reuse of
   `ath9k-virt`. The AR9285 register-level model is Linux-ath9k-shaped;
   a Windows WDI driver wants structured commands (scan / connect /
   set-key), so the device speaks that directly.
3. **One driver, multiple op modes.** Monitor mode is an operating
   mode of the same WDI miniport, not a separate driver. This matches
   how Npcap discovers and switches Wi-Fi adapters into monitor mode.
4. **Packed ring with OWN bit.** Four rings (ctrl-req, ctrl-rsp, tx,
   rx) use a per-descriptor owner bit, simpler than a virtio-style
   available/used split. Producer sets OWN=1 and doorbells; consumer
   clears OWN and advances.
5. **Wire protocol = `ath9k_medium.h` v2 unchanged.** The QEMU device
   translates between its internal command/ring ABI and the medium
   protocol. The hub doesn't know the Windows device exists.
6. **Development signing only.** Test-signing + self-signed cert +
   HVCI off. Distribution signing (EV cert + attestation) is deferred.


## Phase 0 — Pipeline validation

**Goal:** confirm the substrate works end-to-end before writing any
new code.

**Scope**

- Build qemu-ath9k and qemu-vwifi from current `main`.
- Run two Linux VMs with `-device ath9k-virt` connecting to the same
  hub socket.
- Load the host `ath9k_medium_host.ko` and `ath9k_host_relay`.
- Prove hostapd on one VM + `wpa_supplicant` on the other connect,
  get DHCP, ping.
- Optionally bridge a real radio via `ath9k_phys_bridge` and confirm
  it joins the same medium.

**Deliverables**

- No code. A short "Phase 0 bringup log" in the repo documenting
  exact QEMU invocations and the observed success.

**Exit criteria**

- Two Linux VMs successfully exchange WPA2-protected traffic over
  the hub.
- The host `wlanX` interface is up and can scan the virtual medium.

**Why this first**

Every later phase adds complexity on top of the assumption that
"the hub plus medium protocol plus existing Linux pieces work."
Validating that once, before adding Windows-specific unknowns,
means every future bug can be localized to "the new thing" without
second-guessing the substrate.

**Estimate:** 1 day.


## Phase 1 — Probe-only driver and device

**Goal:** a Windows guest loads the driver cleanly, the device
enumerates, capabilities are read, and the adapter appears in Device
Manager — no Wi-Fi functionality yet.

**Scope**

### QEMU side (`vwifi-virt`)

- New PCI device, VEN=0x1AF4 DEV=0x0E00 (or equivalent reserved pair).
- BAR0 = 4 KiB MMIO register file (`vwifi_abi.h` layout):
  signature, ABI version, caps, four ring base/size/doorbell/head
  registers, IRQ status/mask, reset, stats.
- BAR1 = exclusive MSI-X with 4 vectors (ctrl-rsp, rx, tx-complete,
  event).
- Control-command dispatcher handling `NOP`, `GET_CAPS`, `SET_STA_MAC`,
  `SET_OP_MODE`, `SET_CHANNEL`, `SET_RAW_FILTER`, `SCAN_ABORT`.
  All other opcodes return `-ENOSYS`.
- Medium chardev: connect to hub Unix socket, send hello
  (`A9KR` magic + node_id), perform length-prefixed framing,
  reassembly buffer for RX.
- TX ring drain: pull 802.11 frames, wrap in `ath9k_medium_frame_hdr`
  v2, write to hub.
- RX ring drain: channel filter, echo suppression, write frame +
  metadata into driver-posted RX buffers, raise MSI-X RX vector.

### Windows driver (`vwifi.sys`)

- NDIS 6.50 WDI miniport registered via `NdisMRegisterWdiMiniport
  Driver` with both `NDIS_MINIPORT_DRIVER_CHARACTERISTICS` and
  `NDIS_MINIPORT_DRIVER_WDI_CHARACTERISTICS` populated.
- `MiniportInitializeEx`: parse PCI resources, `MmMapIoSpace` BAR0,
  verify signature and ABI version, allocate four rings via
  `NdisMAllocateSharedMemory`, program MMIO, connect MSI-X via
  `NdisMRegisterInterruptEx`, enable device, issue sync `GET_CAPS`,
  push station MAC.
- `VwifiCtrlSendSync`: per-slot request tracking, `KEVENT`-based
  blocking wait; `VwifiCtrlRspDrain` in DPC context wakes pending
  requesters by `req_id`.
- `VwifiMiniportCheckForHangEx`: probe signature register.
- WDI callback stubs for `Allocate/Free/Open/Close/Start/Stop/Post
  Pause/PostRestart/HangDiagnose/TalTxRxInit/TalTxRxDeinit/LeIdle/
  LeCancelIdle`, all returning success.
- `VwifiOidRequest` returns `NDIS_STATUS_NOT_SUPPORTED` for all
  OIDs (Microsoft WLAN component handles the defaults).
- INF file: `PCI\VEN_1AF4&DEV_0E00`, `*IfType=71`, `*MediaType=16`,
  `*PhysicalMediaType=9`.

**Deliverables**

- `qemu-windows-vwifi` repo (or subtree) with:
  - `qemu/` — the new QEMU device (`vwifi_pci.c`, `vwifi_abi.h`,
    `meson.build`, `Kconfig`, `scripts/integrate.sh`).
  - `driver/src/` — driver skeleton (`driver.c`, `hardware.c`,
    `rings.c`, `wdi_ops.c`, `vwifi.h`, `vwifi_abi.h`).
  - `driver/inf/vwifi.inx`.
  - `driver/README.md` with build, sign, install, debug instructions.
- Self-signed test cert procedure documented.
- Kernel debugging over QEMU serial pipe working.

**Exit criteria**

1. `pnputil /add-driver vwifi.inf /install` succeeds without user
   prompts (cert already trusted).
2. Device Manager shows "vwifi virtual Wi-Fi adapter" under Network
   Adapters with no error.
3. `netsh wlan show interfaces` lists the adapter even if disconnected.
4. QEMU log in verbose mode shows the `GET_CAPS` round trip and the
   SET_STA_MAC handshake.
5. WinDbg/DebugView shows `vwifi: adapter initialized successfully`.
6. `VwifiCtrlSendSync` measured round-trip under 1 ms in the common
   case.

**Risks**

- *WDI header drift between WDK versions.* Mitigation: build once
  against a pinned WDK version; document it; lock the Dockerfile
  for CI builds.
- *MSI-X routing issues.* On some chipsets/QEMU configurations, MSI-X
  falls back to INTx silently; make sure `msix_enabled(pdev)` is
  asserted in QEMU and add an explicit log.

**Estimate:** 1–2 weeks.


## Phase 1.5 — Monitor mode and injection

**Goal:** Wireshark running in the Windows guest, via Npcap, captures
raw 802.11 + radiotap traffic flowing on the hub and can inject.

**Background**

Npcap is the path for all modern Windows raw-802.11 capture. It
installs a Lightweight Filter (LWF) driver that sits above any
Wi-Fi miniport and uses the Native 802.11 OIDs to switch the
miniport into `dot11_operation_mode_network_monitor` and enable
`NDIS_PACKET_TYPE_802_11_RAW_DATA/MGMT` packet filters. The miniport
then delivers raw 802.11 MPDUs with a `DOT11_EXTSTA_RECV_CONTEXT`
attached as `NET_BUFFER_LIST_INFO(MediaSpecificInformation)`. Npcap
synthesizes the radiotap header from that context and hands it to
Wireshark as DLT_IEEE802_11_RADIOTAP. This works with WDI miniports
because the Microsoft WLAN component still honors the legacy raw-
indication contract.

**Scope**

### QEMU side

- Wire up `SET_OP_MODE { MONITOR }` to the device's mode state and
  have the RX path set `VWIFI_RX_F_RAW` flag when delivering frames
  in monitor mode.
- Honor `SET_RAW_FILTER` flags (DATA, MGMT, CTRL, FCS_FAIL) to gate
  which MPDUs are forwarded to the guest.
- Accept injected frames via `VWIFI_TX_F_INJECT`: the device forwards
  the frame to the hub verbatim, stamping the medium header from the
  device's current channel and the driver-supplied rate hint.
- No BSSID filter in monitor mode — deliver every frame that matches
  the tuned channel.

### Windows driver

- Implement `VwifiRxDrain` for real:
  1. Allocate an NBL + NET_BUFFER + MDL backed by the RX slot buffer
     (reuse the `RxBufferPool` with per-slot NET_BUFFER structures).
  2. Build a `DOT11_EXTSTA_RECV_CONTEXT` with:
     - `uReceiveFlags |= DOT11_RECV_FLAG_RAW_PACKET` when in NetMon
       mode; add `_FCS_FAILURE` if the frame lacked a valid FCS; add
       `_TIMESTAMP` if TSF is populated.
     - `uPhyId` = the current PHY index
     - `uChCenterFrequency` in MHz
     - `lRSSI` in dBm (signed)
     - `ucDataRate` in 500 kbps units (convert from `rate_code`)
     - `ullTimestamp` in microseconds
     - `usNumberOfMPDUsReceived = 1`
  3. Attach via `NDIS_SET_NET_BUFFER_LIST_MEDIA_SPECIFIC_INFO`.
  4. `NdisMIndicateReceiveNetBufferLists`.
- Implement `VwifiMiniportReturnNetBufferLists` to reclaim RX slots
  and re-post the buffer with OWN=1.
- Add OID handlers for `OID_DOT11_CURRENT_OPERATION_MODE` (set),
  `OID_DOT11_CURRENT_CHANNEL` (set/get), `OID_DOT11_CURRENT_
  FREQUENCY` (set/get), and `OID_GEN_CURRENT_PACKET_FILTER` with
  the raw-data/raw-mgmt bits honored. Each dispatches to a
  corresponding `VWIFI_OP_*` on the control ring.
- Advertise NetMon support in `OpModeCapability` (via the WLAN
  component's attribute-query callback, once Phase 2 wires that up;
  in Phase 1.5 we can hardcode the bit).
- Handle `MiniportSendNetBufferLists` in monitor mode: strip any
  radiotap prefix (check first byte == 0), forward the 802.11 frame
  to the TX ring with `VWIFI_TX_F_INJECT`. Complete the NBL
  immediately.

**Deliverables**

- Updated driver with real RX path and inject support.
- Updated README with Npcap install instructions.
- Small smoke-test script inside the Windows VM:
  `tshark -i vwifi0 -I -c 10` captures 10 frames.

**Exit criteria**

1. Install Npcap with the "Support raw 802.11" option checked.
2. Launch Wireshark as Administrator.
3. The vwifi adapter appears with a Monitor-mode checkbox in Capture
   Options.
4. Enable monitor mode, select channel 6 via WlanHelper.
5. With a Linux VM running hostapd on channel 6 against the same
   hub, Wireshark shows live beacon frames decoded with the 802.11
   dissector and a valid radiotap header (frequency 2437 MHz,
   RSSI -30 dBm).
6. `pcap_sendpacket` with a crafted probe request visible in `tcpdump`
   on the host Linux mac80211 interface (confirming injection
   round-trips through the hub).

**Risks**

- *Npcap's OID set differs slightly from what WDI expects.* Npcap
  uses the legacy Native 802.11 OIDs; the Microsoft WLAN component
  translates these to WDI tasks on our behalf, but monitor mode is
  one of the areas where the translation can be leaky. Expect some
  trial-and-error matching Npcap's exact expectations.
- *HT/VHT frames in radiotap.* Npcap builds radiotap from the
  `DOT11_EXTSTA_RECV_CONTEXT` fields; those fields don't carry MCS
  index, so HT frames get reported at a lower-rate-equivalent.
  Document and accept this for now.
- *Frame drops in high-rate captures.* The single-NBL-per-MPDU design
  has higher overhead than batching; acceptable for Phase 1.5,
  revisit if it becomes a bottleneck.

**Estimate:** 1–2 weeks.


## Phase 2 — Scan and BSS discovery

**Goal:** Windows sees real SSIDs on the medium. `netsh wlan show
networks mode=bssid` returns the networks being beaconed by other
VMs on the hub.

**Scope**

### QEMU side

- Implement `VWIFI_OP_SCAN`: on receipt,
  1. Iterate the requested channel set, setting each via
     `SET_CHANNEL`-equivalent internal state.
  2. For each channel, dwell for `dwell_ms`, collecting beacon and
     probe-response frames delivered by the medium into an internal
     BSS table.
  3. Optionally inject probe-request frames for directed scans (if
     `num_ssids > 0`).
  4. As BSSes are discovered, push `VWIFI_EV_BSS_FOUND` events with
     the parsed BSS entry (bssid, channel, rssi, capabilities, TSF,
     ssid_len, ssid, raw IEs).
  5. When scan is complete or aborted, push `VWIFI_EV_SCAN_COMPLETE`.
- Add internal BSS-table eviction (timeout after N seconds without
  re-observation).
- Honor `VWIFI_OP_SCAN_ABORT` mid-scan.

### Windows driver

- Wire up the OID dispatcher to handle `OID_WDI_TASK_SCAN`.
  Microsoft's WLAN component synthesizes this OID from higher-level
  scan requests.
- Parse the WDI scan parameters TLV, translate to a `VWIFI_OP_SCAN`
  payload, submit asynchronously (do **not** use `VwifiCtrlSendSync`
  here — tasks don't return inline; they complete later with
  indications).
- In `VwifiCtrlRspDrain`'s event path, handle `VWIFI_EV_BSS_FOUND`:
  build a `WDI_TLV_BSS_ENTRY` (or equivalent per WDI 1.x schema),
  chain multiple entries, indicate `NDIS_STATUS_WDI_INDICATION_BSS
  _ENTRY_LIST`.
- Handle `VWIFI_EV_SCAN_COMPLETE`: indicate `NDIS_STATUS_WDI_
  INDICATION_SCAN_COMPLETE` with the task completion status.
- Respect the live-update throttling rule (indicate in bursts of 3+
  entries or every 500 ms).
- Add `OID_WDI_GET_ADAPTER_CAPABILITIES` handler: report PHY types
  (DSSS+OFDM for 2.4 GHz), supported channels, supported ciphers
  (open, WPA2-CCMP for now), supported auth algos (Open, SAE in
  Phase 4). Derived from the `VWIFI_OP_GET_CAPS` reply + hardcoded
  tables for the parts the device doesn't report.
- Implement `OID_WDI_TASK_OPEN_ADAPTER` and `_CREATE_PORT` as
  success-returns; we only support one port.

**Deliverables**

- Driver with scan task wired end to end.
- QEMU device with BSS-table and event stream.
- Test scenario documented: one Linux VM hostapds `vwifi-test` on
  channel 6, one Windows VM scans and sees it.

**Exit criteria**

1. Inside Windows, `netsh wlan show networks mode=bssid` lists
   `vwifi-test` with correct BSSID, SSID, signal, channel.
2. The Wi-Fi flyout shows `vwifi-test` as an available network.
3. Repeated scans (e.g. every 10 seconds) don't leak resources or
   produce duplicate entries.
4. `netsh wlan show networks` on the host Linux via `wpa_cli scan`
   continues to work concurrently (hub fan-out is stable).

**Risks**

- *WDI TLV encoding.* Every WDI structure is TLV (tag-length-value)
  encoded. There is a WDI TLV parser/generator library provided by
  the WDK, but it's version-sensitive. Pin a WDK version; write a
  small helper module.
- *Microsoft WLAN component's scan ordering rules.* The component
  expects scan task completion to follow a strict sequence: start
  → task-started → (BSS entries streamed) → task-complete. Getting
  this wrong causes scan OIDs to hang. Test with a kernel debugger
  attached from day one.

**Estimate:** 2–3 weeks.


## Phase 3 — Associate, connect, data path

**Goal:** Windows connects to an open-network BSS on the medium,
gets an IPv4 address via DHCP, pings the Linux VM hosting the AP.

**Scope**

### QEMU side

- Implement `VWIFI_OP_CONNECT`: given bssid + ssid + channel + auth
  algo (open for Phase 3) + assoc IEs,
  1. Tune to the target channel.
  2. Build and inject an 802.11 Authentication Request (Open).
  3. Receive Authentication Response; if success, build and inject
     Association Request with provided IEs.
  4. Receive Association Response; extract AID and response IEs.
  5. Emit `VWIFI_EV_ASSOC_RESULT` with status_code, bssid, aid,
     assoc-response IEs.
  6. Transition internal state to "associated"; enable STA-mode
     BSSID filter on the RX path.
- Implement `VWIFI_OP_DISCONNECT`: send Disassociation frame, emit
  `VWIFI_EV_DISCONNECTED`.
- Extend RX filter so in STA mode only frames to/from our associated
  BSSID reach the guest (both directed and broadcast/multicast from
  that BSS).
- Add 802.3 → 802.11 framing for TX data path when in STA mode:
  the driver sends 802.11 data frames already (WDI path), but
  later we may accept 802.3 for simplicity; defer.

### Windows driver

- Implement `OID_WDI_TASK_CONNECT`: parse the WDI connect parameters,
  emit `VWIFI_OP_CONNECT`, and track the pending task.
- Handle `VWIFI_EV_ASSOC_RESULT`:
  indicate `NDIS_STATUS_WDI_INDICATION_ASSOCIATION_START` before
  the association attempt, `_ASSOCIATION_RESULT` with status after,
  and finally `_CONNECT_COMPLETE` to complete the connect task.
  **Important ordering: CONNECT_COMPLETE must be indicated before
  any EAPOL frames start flowing.** In open networks there's no
  EAPOL; in WPA2 (Phase 4) this becomes critical.
- Implement the real data path:
  - `MiniportWdiTxTalSend`: build 802.11 data frame from the 802.3
    frames the WLAN component hands us (header conversion: source
    MAC stays, dest MAC becomes RA=BSSID for STA mode, original
    dest becomes DA in the 802.11 address fields). Push to the
    TX ring. Complete via `MiniportWdiTxTalSendComplete`.
  - RX: in STA mode, strip the 802.11 header, convert to 802.3 /
    LLC/SNAP, indicate via the data path NBL pool. The WLAN
    component's receive path takes it from there.
- Implement `OID_WDI_TASK_DISCONNECT` and the `_DISASSOCIATION`
  indication.
- Add link-state indications: `NDIS_STATUS_LINK_STATE` with
  connected/disconnected and the current data rate.
- Implement `OID_WDI_GET_ASSOCIATION_STATE` and similar getter OIDs.

**Deliverables**

- Connect-to-open-network working.
- DHCP client inside Windows gets an IP from a DHCP server running
  on the Linux AP VM.
- `ping` and `iperf3` work across the medium.

**Exit criteria**

1. Click `vwifi-test` in Windows Wi-Fi flyout → Connect → no password
   → connection succeeds within 10 seconds.
2. `ipconfig` shows an IPv4 address assigned by the Linux DHCP server.
3. `ping <Linux AP IP>` succeeds with <5 ms latency.
4. `iperf3 -c <AP>` sustains > 50 Mbps over the virtual medium
   (more important: no frame drops, not peak throughput).
5. Disconnect via flyout → adapter returns to disconnected state,
   no kernel leaks (check pool usage via `!poolused` in WinDbg).

**Risks**

- *802.11 ↔ 802.3 header translation bugs.* Classic source of data-
  path breakage. Unit-test the translation logic separately.
- *WLAN component's expectations around rekey and roaming indications
  even in Phase 3 open networks.* May need to stub extra
  indications (`ROAM_COMPLETE`, etc.) even when no roam occurs.

**Estimate:** 2–3 weeks.


## Phase 4 — Security: WPA2 then WPA3

**Goal:** WPA2-PSK (CCMP-128) associate and data encrypted; WPA3-SAE
as a stretch goal.

**Scope**

### Phase 4a — WPA2-PSK (CCMP-128)

**QEMU side**

- Implement `VWIFI_OP_SET_KEY` and `VWIFI_OP_DEL_KEY`: store PTK
  (pairwise) and GTK (group) keyed by (MAC, index, pairwise-flag).
- Implement CCMP-128 encryption on TX data frames when a PTK is
  installed for the destination; decryption on RX data frames when
  a PTK/GTK is installed. Libraries: embed a minimal CCMP implementation
  or link against an existing one (CCMP is ~100 lines of code).
- Honor `VWIFI_TX_F_ENCRYPTED` flag on TX descriptors — if set, the
  driver has already encrypted (used for EAPOL frames that go out
  unencrypted, by clearing this bit explicitly).
- Deliver EAPOL frames (ethertype 0x888E in data payload) without
  decryption, even before keys are installed, so the 4-way handshake
  frames reach the driver / WLAN component.
- Emit `VWIFI_EV_KEY_INSTALLED` to confirm key application.

**Windows driver**

- Handle `OID_WDI_SET_ADD_CIPHER_KEYS` / `OID_WDI_SET_DELETE_CIPHER
  _KEYS`. Translate each key into a `VWIFI_OP_SET_KEY` request.
- Extend connect path to set `cipher_pairwise=CCMP128`,
  `cipher_group=CCMP128`, `akm_suite=PSK` when the WDI connect
  parameters indicate WPA2-PSK.
- **Critical ordering**: indicate `ASSOCIATION_RESULT` and
  `CONNECT_COMPLETE` to the WLAN component *before* EAPOL frames
  start flowing, even though the connection isn't really done
  until the 4-way handshake completes. If `CONNECT_COMPLETE` is
  withheld, WDI sends `OID_TASK_DISCONNECT` and tears down the
  connection. The 4-way handshake is driven by the WLAN component
  in user mode (via wlansvc and the authenticator/supplicant); the
  driver's role is purely to plumb EAPOL data frames through and
  install the resulting keys.

### Phase 4b — WPA3-SAE (stretch)

- Extend auth algo enumeration to include `VWIFI_AUTH_SAE`.
- QEMU device passes SAE authentication frames through unchanged
  (SAE handshake is EAPOL-like but occurs in authentication frames,
  not EAPOL data frames; the WLAN component handles the SAE math).
- Add GCMP-256 cipher support alongside CCMP-128.
- Verify PMF (Protected Management Frames) is enabled on the BSS
  and that the driver reports support correctly in the capabilities.

**Deliverables**

- WPA2-PSK connect works end-to-end to a Linux AP VM with
  `wpa=2 rsn_pairwise=CCMP`.
- Captured traffic in monitor mode (Phase 1.5) shows CCMP-encrypted
  frames — confirming encryption is actually happening.
- WPA3-SAE connect works if time permits; else documented as
  Phase 5 follow-up.

**Exit criteria**

1. Create `vwifi-wpa2` SSID on Linux hostapd with WPA2-PSK,
   passphrase `testpass123`.
2. From Windows, select the network in the flyout, enter password,
   connect succeeds.
3. `netsh wlan show interfaces` reports "Authentication: WPA2-Personal"
   and "Cipher: CCMP".
4. Ping works as in Phase 3.
5. Wireshark monitor capture on a third Windows VM (or Linux) shows
   the four EAPOL-Key messages of the 4-way handshake, followed by
   CCMP-encrypted data frames.

**Risks**

- *The 4-way handshake timing rule.* This is the #1 place WDI
  drivers fail. Budget a week just for debugging ordering.
- *Key material lifetime.* The WLAN component may send
  `SET_ADD_CIPHER_KEYS` with `bPairwise=TRUE` for the PTK separately
  from the GTK; confusing these causes the first data frame to
  decrypt incorrectly. Test both directions before declaring done.
- *Replay counter handling.* CCMP requires per-key replay counters;
  a wrong implementation will silently drop valid frames after
  rekey.

**Estimate:** 2–3 weeks for WPA2; +1 week for WPA3.


## Phase 5 — Polish and realism

This phase is open-ended. Items below are ordered by approximate
value; not all need to be done.

**Status.** SoftAP and 5 GHz are implemented and tested on the device
side (`tests/softap.c`, 11 assertions). Everything else below remains
open. Two items were closed out as part of that work:

- *Timer multiplexing*: the device now schedules its own deadlines
  (scan dwell / connect timeout / beacon interval) onto the backend's
  single one-shot timer, rather than widening the backend vtable —
  which every future backend, including the vfio-user port, would
  otherwise have had to implement.
- *Beacon drift*: periodic beacons advance from their scheduled
  deadline, not from wall-clock "now", so the host timer's
  millisecond granularity doesn't accumulate. A Time Unit is 1024us,
  not 1000us; the test asserts across 10 beacons because the
  difference is only visible in aggregate.

**Feature completeness**

- **Roaming**: `OID_WDI_TASK_ROAM`. Device picks a new BSS from a
  candidate list, tears down old association, establishes new one,
  emits `ROAM_COMPLETE`.
- **SoftAP**: Windows guest hosts an AP via mobile hotspot UI.
  Requires `VWIFI_OP_START_AP` / `STOP_AP`, beacon synthesis in the
  QEMU device, association request handling, 4-way handshake from
  the AP side.
- **Wi-Fi Direct**: additional operation mode;
  `DOT11_VWIFI_ATTRIBUTES` reports simultaneous ops.
- **Multiple ports**: more than one simultaneous MAC entity per
  adapter (STA + monitor, or two STAs).
- **HT/VHT rate reporting**: populate `ucDataRate` and phy IDs so
  Wireshark/`netsh wlan show interfaces` report 802.11n / 802.11ac
  connections accurately.
- **5 GHz support**: extend the supported-channels bitmask and
  channel-freq helpers.
- **Radio off / airplane mode**: honor `OID_WDI_SET_RADIO_STATE`;
  emit `NDIS_STATUS_WDI_INDICATION_RADIO_STATUS`.

**Robustness**

- **Live migration support**: drop the `vmstate.unmigratable` flag
  in QEMU; serialize ring state + medium reassembly buffer +
  association state.
- **Hot-reset recovery**: if the hub socket disconnects, reconnect
  and re-hello automatically; signal link state changes up.
- **Stress testing**: 10k scans, 1M data frames, 100 connect/
  disconnect cycles; watchdog for leaked NBLs.
- **SAL annotations**: add `_In_`, `_Out_`, `_Inout_` annotations
  across all driver functions to enable Static Driver Verifier.
- **Driver Verifier compliance**: enable full DV in a test VM,
  exercise all code paths without triggering any verifier bugchecks.

**Ecosystem**

- **virtio-win-style installer**: package `vwifi.sys`, `vwifi.inf`,
  the test cert, and a one-click installer.
- **EV-signed build pipeline**: CI that produces an attestation-
  signed binary for developers who want to avoid test-signing.
- **Multi-VM test harness**: automated smoke test that spins up
  3 Linux VMs + 2 Windows VMs + the hub, runs the test matrix
  (scan, open connect, WPA2 connect, monitor), reports pass/fail.

**Estimates:** each item 1–3 weeks. Plan 2–3 months total for a
robust Phase 5.


## Cross-cutting concerns

### Shared ABI

`vwifi_abi.h` is the contract. Rules:

- Never remove or reorder fields in an existing struct. Add new
  fields only at the end, in reserved-space holes, or behind a
  capability bit.
- New opcodes get fresh numbers. Deprecated opcodes stay allocated
  but return `-ENOSYS`.
- Bump `VWIFI_ABI_VERSION` when any existing layout changes. The
  driver refuses to bind on mismatch.

### Signing and install

Two personas:

- **Developer** (single Windows VM, frequent driver replacement):
  test-signing mode + self-signed cert. Documented in the driver
  README. HVCI must be off.
- **Distribution** (user installs driver on a Windows system they
  didn't build on): EV code-signing certificate + Microsoft
  attestation signing via Partner Center. Binary loads on any
  Windows 10/11 system without settings changes. Out of scope
  for the plan but budget $500/year if pursued.

### Kernel debugging

The driver is complex enough that printf-debugging via DbgPrint is
insufficient by Phase 3. Standard setup:

1. Host: `qemu-system-x86_64 ... -serial pipe:\\.\pipe\windbg-vwifi`
2. Guest: `bcdedit /debug on`, `bcdedit /dbgsettings SERIAL
   DEBUGPORT:1 BAUDRATE:115200`
3. Host: `windbg -k com:pipe,port=\\.\pipe\windbg-vwifi,resets=0,
   reconnect`

Breakpoints to set during bringup:
- `vwifi!VwifiMiniportInitializeEx` — device init
- `vwifi!VwifiCtrlSendSync` — every control request
- `vwifi!VwifiCtrlRspDrain` — every response/event
- `vwifi!VwifiMessageIsr` — interrupt delivery
- `vwifi!VwifiRxDrain` — RX indication

### Testing matrix

| Scenario | Phase | Automated? |
|---|---|---|
| Driver loads, device enumerates | 1 | Yes (pnputil + netsh check) |
| Signature register readable | 1 | Yes (reg query via devcon) |
| Wireshark captures beacons | 1.5 | Yes (`tshark -i vwifi0 -I -c 10`) |
| Wireshark inject round-trips | 1.5 | Manual |
| `netsh wlan show networks` sees Linux hostapd SSID | 2 | Yes |
| Open-network connect + DHCP + ping | 3 | Yes (scripted) |
| WPA2-PSK connect + ping | 4a | Yes (scripted) |
| WPA3-SAE connect + ping | 4b | Yes (scripted, optional) |
| Driver Verifier passes 24h soak | 5 | Yes |
| Migration stop/start | 5 | Manual |

### Build and CI

- QEMU: build on any Linux with standard QEMU build deps.
  Integrate as a patch series against upstream QEMU or as an
  out-of-tree device using a small integration script (mirrors
  qemu-ath9k's `scripts/integrate.sh`).
- Driver: build on Windows with EWDK or VS+WDK. Pin the WDK
  version in `README.md` and the project file. Provide a
  `build.cmd` that runs MSBuild with the pinned configuration.
- CI: a Windows runner with EWDK available, building the `.sys`
  and uploading the signed artifact. A Linux runner building
  patched QEMU.


## Risk register

| Risk | Phase | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| No public WDI sample exists | All | Certain | High | Lean on MSDN docs; build a spike repo of tiny WDI experiments; join WDK forum for questions |
| WDI header drift between WDK versions | 1+ | High | Medium | Pin WDK version; CI with exact pin; document the version in README |
| WDI is in maintenance mode; could break on future Windows | All | Low | Medium | Accept; plan for a future WiFiCx port if Windows 10 drops out of scope |
| 4-way handshake ordering bug | 4 | Certain | Medium | Schedule extra debug time; implement ordering tests before integrating with real WPA2 AP |
| TLV encoding bugs | 2+ | High | Medium | Use WDK's TLV helper lib; unit-test encoders separately |
| HVCI enabled by default on Windows 11 | 1 | Certain | Low | Document the disable step; don't treat as a driver bug |
| Npcap's OID expectations drift from WDI translation | 1.5 | Medium | Medium | Start with simple beacon capture; grow complexity one OID at a time |
| Data-path performance insufficient | 3+ | Medium | Low | Single-NBL-per-packet is fine for research; batch later if needed |
| QEMU upstream changes PCI/MSI-X APIs | Any | Low | Low | Pin QEMU release; CI builds against pinned version |
| Hub socket disconnect mid-session | All | Medium | Low | Phase 5 adds reconnect; during dev, just restart |

## References

- **Our repos**
  - [qemu-ath9k](https://github.com/bprochazka-bit/qemu-ath9k) — Phase 1 AR9285 QEMU device
  - [qemu-vwifi](https://github.com/bprochazka-bit/qemu-vwifi) — hub, wire protocol, Linux host radio

- **Wire protocol**
  - `ath9k_medium.h` (qemu-vwifi) — the unchanging contract with the hub

- **Windows WDI reference**
  - WDI Miniport Driver Design Guide (Microsoft Learn)
  - WDI IHV driver interfaces article
  - `dot11wdi.h` reference pages on Microsoft Learn
  - `OID_WDI_TASK_*` topic cluster
  - `NDIS_STATUS_WDI_INDICATION_*` topic cluster
  - Native 802.11 Miniport Drivers archive (still relevant for
    Monitor mode semantics and `DOT11_EXTSTA_RECV_CONTEXT`)

- **Npcap**
  - Npcap User's Guide (npcap.com)
  - Raw 802.11 capture feature documentation

- **Code-signing**
  - Microsoft Partner Center attestation signing overview
  - BCDEdit TESTSIGNING documentation
  - HVCI / Memory Integrity disable procedure

- **Related prior art**
  - `mac80211_hwsim` (Linux kernel) — reference virtual Wi-Fi driver
  - `wmediumd` — userspace virtual air medium
  - `virtio-win/kvm-guest-drivers-windows` — style and build reference
    for Windows drivers paired with QEMU devices
