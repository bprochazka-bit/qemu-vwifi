/*
 * vwifi — wdi_data.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * The WDI per-frame data path: pulling TX frames out of the WLAN
 * component and pushing RX frames back into it.
 *
 * Where the frames actually go
 * ----------------------------
 * Nowhere new. TX ends up in VwifiTxDataFrame and RX comes from the
 * same ring drain data.c already had; this file is the WDI-shaped
 * plumbing between the component and those.
 *
 * What shape those frames are is stated once, in the frame-format
 * contract next to the VwifiTxDataFrame declaration in vwifi_drv.h,
 * and is not restated here. In short: 802.11 MPDUs in both
 * directions, because wdiwifi.sys does the 802.3 conversion above
 * the miniport rather than leaving it to the device.
 *
 * This header used to claim the opposite -- "the device does the
 * 802.3 <-> 802.11 conversion, so what crosses this boundary is
 * ordinary Ethernet frames in both directions" -- and went on
 * claiming it after the code had been changed to do the other thing.
 * It is worth naming, because the cost was not one bug: RX was
 * debugged against it, then TX, then EAPOL, each time by reasoning
 * from a comment instead of from the wire. Hence one copy of the
 * contract, in a header both directions include.
 *
 * Why not MiniportSendNetBufferLists
 * ----------------------------------
 * Because in WDI it is not called for data. wdiwifi.sys owns the send
 * path: NDIS hands frames to IT, its TxMgr queues them per (port, peer,
 * TID), and the miniport is expected to come and take them through
 * NdisWdiTxDequeueIndication when it has room. The miniport's own
 * SendNetBufferLists handler stays registered and stays unused for
 * station data. That is why an implemented VwifiTxDataFrame and a
 * connected link still moved nothing: the frames were never offered to
 * it.
 *
 * The NBL <-> metadata contract, and how it was settled
 * -----------------------------------------------------
 * Every frame the component queues has a WDI_FRAME_METADATA. dot11wdi.h
 * defines it -- LIST_ENTRY at +0x00, a pNBL back-pointer at +0x10, a
 * WDI_FRAME_ID at +0x18 -- and defines the calls that allocate and free
 * it. What it never states is where on the NET_BUFFER_LIST the pointer
 * to it lives, and the miniport needs that in both directions:
 * TxSendCompleteIndication is fed FrameIDs, and an RX NBL this driver
 * builds has to carry a metadata the component can find.
 *
 * The RX direction settles it. The miniport creates those NBLs, so it
 * must attach the metadata somewhere the port driver will look -- and
 * the only field of a NET_BUFFER_LIST a miniport is permitted to write
 * is MiniportReserved. There is nowhere else it could be, and TX is the
 * same contract read the other way.
 *
 * That is an argument, and this driver has been wrong about arguments
 * before -- INDICATION_REQUIRED, and MaxCommandSize after it. So it is
 * not trusted, it is CHECKED: VwifiTxMetadata reads the candidate,
 * confirms the pointer is a plausible kernel address, and confirms that
 * following its pNBL leads back to the NBL it came from. A metadata
 * that fails the check is not used, and the frame is still sent and
 * still completed -- the transfer completion takes the NBL, not the
 * metadata, so TX degrades to "no send-complete" rather than to a
 * bugcheck or a leak.
 *
 * Both of these are honest about their own limits in the log rather
 * than silently doing half a job.
 */

#include "vwifi_drv.h"


/* Dequeue parameters. The component reads these as a budget: how many
 * bytes, how many frames, and how much credit the target has. */
#define VWIFI_TAL_DEQUEUE_QUANTUM   0xFFFFFFFFu
#define VWIFI_TAL_DEQUEUE_MAXFRAMES 32
#define VWIFI_TAL_DEQUEUE_CREDIT    0xFFFFu

/* Total dequeue rounds one pump may run, counting every time it is
 * re-entered and goes round again. A ceiling on time spent at
 * DISPATCH_LEVEL, which is the whole point of having one. */
#define VWIFI_TAL_PUMP_MAX_ROUNDS   16

/* ============================================================
 * The metadata attached to a frame
 * ============================================================ */

/* True if p could be a kernel-mode pointer to a structure.
 *
 * Not a guarantee -- nothing available here is -- but it rejects the
 * three things a stale MiniportReserved slot is actually likely to
 * hold: zero, a small integer, and a user-mode address. Combined with
 * the pNBL round-trip below, a value that passes both and is still
 * wrong would have to be a kernel object that happens to hold this
 * NBL's address at exactly +0x10. */
static BOOLEAN
VwifiPlausibleKernelPointer(_In_opt_ PVOID p)
{
    ULONG_PTR v = (ULONG_PTR)p;

    if (v == 0) return FALSE;
    if ((v & (sizeof(PVOID) - 1)) != 0) return FALSE;
#if defined(_WIN64)
    if (v < 0xFFFF800000000000ULL) return FALSE;
#else
    if (v < 0x80000000UL) return FALSE;
#endif
    return TRUE;
}

/* The WDI_FRAME_METADATA for this NBL, or NULL if it cannot be
 * established.
 *
 * NULL is a supported answer. The caller sends the frame either way;
 * all that is lost is the send-completion, and losing it visibly beats
 * inventing a FrameID. */
static PWDI_FRAME_METADATA
VwifiTxMetadata(_In_ PNET_BUFFER_LIST Nbl)
{
    PWDI_FRAME_METADATA md =
        (PWDI_FRAME_METADATA)NET_BUFFER_LIST_MINIPORT_RESERVED(Nbl)[0];

    if (!VwifiPlausibleKernelPointer(md)) return NULL;

    /* MmIsAddressValid before the read. It is racy in general and that
     * does not matter here: nothing can be paging out a non-paged
     * allocation the component is actively using, and the only thing
     * being defended against is the first probe of a field that might
     * never have been a pointer at all. */
    if (!MmIsAddressValid(md) ||
        !MmIsAddressValid((PUCHAR)md + FIELD_OFFSET(WDI_FRAME_METADATA, pNBL))) {
        return NULL;
    }

    /* The round trip. This is the whole check. */
    if (md->pNBL != Nbl) return NULL;

    return md;
}

/* ============================================================
 * TX
 * ============================================================ */

/* One NBL to the device. Returns what the component should be told
 * about the transfer. */
static WDI_TX_FRAME_STATUS
VwifiTalTxOneNbl(_Inout_ PVWIFI_ADAPTER Adapter, _In_ PNET_BUFFER_LIST Nbl)
{
    PNET_BUFFER nb;
    WDI_TX_FRAME_STATUS result = WDI_TxFrameStatus_Ok;

    for (nb = NET_BUFFER_LIST_FIRST_NB(Nbl); nb; nb = NET_BUFFER_NEXT_NB(nb)) {
        ULONG len = NET_BUFFER_DATA_LENGTH(nb);
        PUCHAR flat;
        PVOID alloc = NULL;
        NDIS_STATUS st;

        /* Contiguous if it already is, a bounce buffer if not -- the
         * same shape as the injection path in driver.c, and for the
         * same reason: the device wants one flat frame and an NBL is
         * under no obligation to be one. */
        flat = NdisGetDataBuffer(nb, len, NULL, 1, 0);
        if (flat == NULL) {
            alloc = NdisAllocateMemoryWithTagPriority(
                Adapter->MiniportAdapterHandle, len,
                VWIFI_POOL_TAG, NormalPoolPriority);
            if (alloc == NULL) {
                VWIFI_WARN("TAL tx: no bounce buffer for %u bytes", len);
                return WDI_TxFrameStatus_TransferFailed;
            }
            flat = NdisGetDataBuffer(nb, len, alloc, 1, 0);
        }

        /* What shape is a WDI TX frame?
         *
         * An MPDU, per the frame-format contract in vwifi_drv.h --
         * this dump is what established that, and it stays because a
         * silent change of shape underneath us is the one thing the
         * flag we pass below cannot survive.
         *
         * It is decoded AS an MPDU, which is a correction. The
         * original version tested bytes 12-13 for an EtherType first
         * and only fell through to the 802.11 test if that failed --
         * but in an MPDU bytes 12-13 are the middle of addr1, so the
         * "802.3, the device's assumption holds" branch was reachable
         * by coincidence on a frame that was nothing of the kind.
         * Reading it as 802.11 first and 802.3 only as the fallback
         * puts the burden of proof on the right side.
         *
         * Frame control is bytes 0-1: bits 2-3 of byte 0 are the type
         * (2 = data), bits 4-7 the subtype (bit 3 set = QoS, which
         * adds 2 bytes to the header). Then LLC/SNAP, whose last two
         * bytes are the EtherType the payload actually has. */
        if (flat != NULL && len >= 24) {
            ULONG type = (flat[0] >> 2) & 0x3;
            ULONG hlen = 24 + (((flat[0] & 0x80) && type == 2) ? 2u : 0u);
            ULONG et   = (len >= hlen + 8)
                             ? (((ULONG)flat[hlen + 6] << 8) | flat[hlen + 7])
                             : 0;

            VWIFI_TAL_ONCE(
                "TAL tx: first frame %u bytes: fc %02x %02x dur %02x %02x "
                "| a1 %02x:%02x:%02x:%02x:%02x:%02x "
                "| a2 %02x:%02x:%02x:%02x:%02x:%02x "
                "| a3 %02x:%02x:%02x:%02x:%02x:%02x | seq %02x %02x -- %s",
                len,
                flat[0], flat[1], flat[2], flat[3],
                flat[4], flat[5], flat[6], flat[7], flat[8], flat[9],
                flat[10], flat[11], flat[12], flat[13], flat[14], flat[15],
                flat[16], flat[17], flat[18], flat[19], flat[20], flat[21],
                flat[22], flat[23],
                (type == 2)
                    ? "802.11 DATA MPDU, as expected -- sent with "
                      "VWIFI_TX_F_80211 so the device passes it through "
                      "rather than encapsulating it again"
                    : (type == 0)
                        ? "802.11 MANAGEMENT frame on the data path -- "
                          "unexpected"
                        : "NOT an 802.11 data frame -- shape unknown, the "
                          "addresses above are not addresses");

            if (type == 2 && et != 0) {
                VWIFI_TAL_ONCE(
                    "TAL tx:   hdr %u bytes%s, SNAP EtherType 0x%04x (%s)",
                    hlen, (hlen == 26) ? " (QoS)" : "", et,
                    (et == 0x0800) ? "IPv4"
                                   : (et == 0x0806) ? "ARP"
                                   : (et == 0x86dd) ? "IPv6"
                                   : (et == 0x888e) ? "EAPOL"
                                                    : "other");
            }
        }

        /* The request side of the same exchange.
         *
         * Printed so the two transaction ids can be put next to each
         * other without a packet capture: an offer whose xid does not
         * match the discover that asked for it is a frame the stack is
         * right to drop, and that is indistinguishable from every other
         * failure at the level of "DHCP does not complete".
         *
         * This one is an MPDU, so the offsets are the 802.11 header
         * (24, plus 2 if QoS), then LLC/SNAP (8), then IP (20) and UDP
         * (8) before BOOTP starts. */
        if (flat != NULL && len >= 24 && ((flat[0] >> 2) & 0x3) == 2) {
            ULONG hdr = 24;

            if (flat[0] & 0x80) hdr += 2;   /* QoS control */
            {
                ULONG ip = hdr + 8;
                ULONG udp = ip + 20;
                ULONG bootp = udp + 8;

                if (len >= bootp + 44 &&
                    flat[hdr + 6] == 0x08 && flat[hdr + 7] == 0x00 &&
                    flat[ip + 9] == 17 &&
                    ((flat[udp] << 8) | flat[udp + 1]) == 68 &&
                    ((flat[udp + 2] << 8) | flat[udp + 3]) == 67) {
                    VWIFI_TAL_FIRST(3,
                        "TAL tx:   DHCP op %u xid %02x%02x%02x%02x "
                        "flags %02x%02x chaddr %02x:%02x:%02x:%02x:%02x:%02x",
                        flat[bootp],
                        flat[bootp + 4], flat[bootp + 5],
                        flat[bootp + 6], flat[bootp + 7],
                        flat[bootp + 10], flat[bootp + 11],
                        flat[bootp + 28], flat[bootp + 29], flat[bootp + 30],
                        flat[bootp + 31], flat[bootp + 32], flat[bootp + 33]);
                }
            }
        }

        /* VWIFI_TX_F_80211: what the component hands down is a
         * complete MPDU, header and all -- confirmed off the wire by
         * the dump above. The device must not encapsulate it again. */
        st = (flat != NULL)
                 ? VwifiTxDataFrame(Adapter, flat, len, VWIFI_TX_F_80211)
                 : NDIS_STATUS_FAILURE;

        if (alloc != NULL) {
            NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
                                          alloc, VWIFI_POOL_TAG);
        }

        if (st != NDIS_STATUS_SUCCESS) {
            VWIFI_WARN("TAL tx: frame of %u bytes failed 0x%08x %s",
                       len, st, VwifiNdisStatusName(st));
            /* Discard rather than TransferFailed: the frame was
             * understood and refused, not lost in transit. */
            result = WDI_TxFrameStatus_Discard;
            break;
        }
    }

    return result;
}

/* Lift the pause the component puts on a peer's TX when it is created.
 *
 * dot11wdi.h names the reason and nothing else in the API can clear it:
 *
 *     WDI_TX_PAUSE_REASON_PEER_CREATE = 0x00000002
 *
 * Pause and restart are the target's to declare -- both are
 * NDIS_WDI_TX_SEND_*_IND, miniport to component -- so a peer whose
 * creation paused TX stays paused until the miniport says the target is
 * ready for it. That is the state the trace was stuck in: peer created,
 * peer configured, TxTargetDescInit run, TxPeerBacklog(backlogged=1)
 * raised, and TxDataSend never once fired, because the component had
 * frames for a peer it had been told nothing about being ready.
 *
 * Called from the peer-config callback rather than straight after
 * PeerCreateIndication: config is the component saying it has finished
 * setting the peer up, and restarting a peer it has not finished with
 * would be answering a question it has not asked yet. */
VOID
VwifiTalTxRestartPeer(_Inout_ PVWIFI_ADAPTER Adapter,
                      _In_ WDI_PORT_ID PortId,
                      _In_ WDI_PEER_ID PeerId)
{
    PNDIS_WDI_DATA_API api = Adapter->DataPathApi;

    if (api == NULL || api->TxSendRestartIndication == NULL) {
        VWIFI_WARN("TAL tx: no TxSendRestartIndication -- peer %u stays "
                   "paused on WDI_TX_PAUSE_REASON_PEER_CREATE and nothing "
                   "will ever be sent to it", PeerId);
        return;
    }

    VWIFI_INFO("TAL tx: restarting peer %u on port %u (clearing the "
               "peer-create pause, all TIDs)", PeerId, PortId);
    api->TxSendRestartIndication(Adapter->DataPathHandle, PortId, PeerId,
                                 VWIFI_TAL_ALL_TIDS,
                                 WDI_TX_PAUSE_REASON_PEER_CREATE);

    /* Anything already queued for this peer is waiting right now. */
    VwifiTalTxPump(Adapter, PortId, PeerId, VWIFI_TAL_ALL_TIDS);
}

/* Drain whatever the component has queued, for as long as it keeps
 * giving us frames. */
VOID
VwifiTalTxPump(_Inout_ PVWIFI_ADAPTER Adapter,
               _In_ WDI_PORT_ID PortId,
               _In_ WDI_PEER_ID PeerId,
               _In_ UINT32 ExTidBitmask)
{
    PNDIS_WDI_DATA_API api = Adapter->DataPathApi;
    ULONG rounds = 0;

    if (api == NULL || api->TxDequeueIndication == NULL ||
        api->TxTransferCompleteIndication == NULL) {
        VWIFI_TAL_ONCE("TAL tx: no dequeue/transfer-complete in the data "
                         "API -- nothing can be sent");
        return;
    }

    /* One pump at a time, per adapter.
     *
     * Everything below re-enters the component, and the component is
     * entitled to call TxDataSend, TxTalSend or TxPeerBacklog back from
     * inside those calls -- the trace shows a TxPeerBacklog arriving
     * between one pump's first and last log line, at the same
     * timestamp, so it came from inside this function. Each such
     * re-entry was another pump frame on a 24 KB kernel stack with
     * nothing bounding the depth. Every captured hang has both
     * processors in ntoskrnl at IRQL 15, which is what a bugcheck's
     * freeze looks like; the cause of that bugcheck is not readable
     * from the registers, so treat unbounded depth here as a defect
     * worth fixing on its own terms rather than as a diagnosis.
     *
     * Nothing is lost by deferring: the loop below asks the component
     * what it has rather than being told what to send, so the running
     * pump picks up whatever the re-entrant call wanted. */
    if (InterlockedCompareExchange(&Adapter->TxPumpActive, 1, 0) != 0) {
        InterlockedExchange(&Adapter->TxPumpAgain, 1);
        VWIFI_TAL_FIRST(4, "TAL tx: pump re-entered for port %u peer %u -- "
                           "deferring to the one already running",
                        PortId, PeerId);
        return;
    }

claimed:
    for (;;) {
        PNET_BUFFER_LIST chain = NULL;
        WDI_FRAME_ID     frameIds[VWIFI_TAL_DEQUEUE_MAXFRAMES];
        UINT16           nFrameIds = 0;
        ULONG            nFrames = 0;
        ULONG            nFailed = 0;

        PCSTR via = "dequeue";

        /* NULL first. TxDequeueIndication is an _Out_ and a component
         * with nothing to give may simply not write it. */
        api->TxDequeueIndication(Adapter->DataPathHandle,
                                 VWIFI_TAL_DEQUEUE_QUANTUM,
                                 VWIFI_TAL_DEQUEUE_MAXFRAMES,
                                 VWIFI_TAL_DEQUEUE_CREDIT,
                                 &chain);

        /* Then the per-peer pull, if the general one gave nothing.
         *
         * These are two different questions and only the second names a
         * peer. TxDequeueIndication asks "what should the target send
         * next", scheduled by the component across everything it holds;
         * TxReleaseFrameIndication asks "release what is queued for
         * THIS peer and these TIDs". A backlog notification is about one
         * peer, so if the general form comes back empty for a peer the
         * component has just called backlogged, the specific form is the
         * question that was actually being asked.
         *
         * Which one produced the frames is logged, because it decides
         * what this loop should be doing and the answer is not in any
         * header. */
        if (chain == NULL && PeerId != WDI_PEER_ANY &&
            api->TxReleaseFrameIndication != NULL) {
            api->TxReleaseFrameIndication(Adapter->DataPathHandle,
                                          PortId, PeerId, ExTidBitmask,
                                          VWIFI_TAL_DEQUEUE_MAXFRAMES,
                                          VWIFI_TAL_DEQUEUE_CREDIT,
                                          &chain);
            via = "release";
        }

        if (chain == NULL) {
            /* The silent case, and it was invisible: this loop only
             * logged when it sent something, so a pump that dequeued
             * nothing looked exactly like a pump that was never
             * called. */
            VWIFI_TAL_FIRST(8, "TAL tx: nothing to send for port %u peer %u "
                               "tids 0x%x (neither dequeue nor release "
                               "returned a frame)",
                            PortId, PeerId, ExTidBitmask);
            break;
        }

        while (chain != NULL) {
            PNET_BUFFER_LIST      nbl  = chain;
            PWDI_FRAME_METADATA   md;
            WDI_TX_FRAME_STATUS   status;

            /* Unlink before touching it: the transfer completion hands
             * this NBL back and the component is free to reuse the Next
             * field the moment it does. */
            chain = NET_BUFFER_LIST_NEXT_NBL(nbl);
            NET_BUFFER_LIST_NEXT_NBL(nbl) = NULL;

            md     = VwifiTxMetadata(nbl);
            status = VwifiTalTxOneNbl(Adapter, nbl);
            nFrames++;
            /* Counted separately because "sent" was a lie: nFrames
             * counts frames this loop HANDLED, and a frame refused by
             * VwifiTxDataFrame -- not associated yet, bad length, ring
             * full -- incremented it just the same. The per-frame
             * warning inside VwifiTalTxOneNbl says which, but the
             * summary line has to stop claiming success it did not
             * check. */
            if (status != WDI_TxFrameStatus_Ok) nFailed++;

            if (md != NULL && nFrameIds < RTL_NUMBER_OF(frameIds)) {
                frameIds[nFrameIds++] = md->FrameID;
            } else if (md == NULL) {
                VWIFI_TAL_ONCE("TAL tx: no verifiable WDI_FRAME_METADATA on "
                                 "this NBL (MiniportReserved[0] did not lead "
                                 "back to it) -- sending anyway, without a "
                                 "send-completion");
            }

            /* The transfer is over the moment VwifiTxDataFrame returns:
             * the bytes are in the ring slot and the descriptor is
             * published, so the component's buffer is ours no longer. */
            api->TxTransferCompleteIndication(Adapter->DataPathHandle,
                                              status, nbl);
        }

        if (nFrameIds > 0 && api->TxSendCompleteIndication != NULL) {
            /* Ok for all of them: this device has no over-the-air
             * acknowledgement to report and the ring accepted them. A
             * per-frame status would be a fiction. */
            api->TxSendCompleteIndication(Adapter->DataPathHandle,
                                          WDI_TxFrameStatus_Ok,
                                          nFrameIds, frameIds, NULL);
        }

        VWIFI_TAL_FIRST(8, "TAL tx: %u frame(s) via %s, %u accepted by the "
                           "device, %u refused, %u with a frame id",
                        nFrames, via, nFrames - nFailed, nFailed, nFrameIds);

        /* A component that keeps handing back frames forever would spin
         * this loop at DISPATCH_LEVEL. Bounded, and the next
         * TxDataSend/TxPeerBacklog brings us straight back.
         *
         * `rounds` is deliberately NOT reset when the pump goes round
         * again below -- see the tail. */
        if (++rounds >= VWIFI_TAL_PUMP_MAX_ROUNDS) {
            VWIFI_WARN("TAL tx: %u dequeue rounds in one pump -- yielding",
                       rounds);
            break;
        }
    }

    /* Anything that arrived while this pump was running is answered by
     * going round once more. Cleared before the re-check so a
     * notification racing the release is not lost: worst case the loop
     * runs once with nothing to send, which is a logged no-op.
     *
     * A `goto`, not a call. This used to recurse into VwifiTalTxPump
     * here, and that made the bound above bound nothing: every re-entry
     * started a fresh 16-round budget, so a component that re-entered
     * the pump from inside TxTransferCompleteIndication -- which the
     * trace shows it does -- could keep the pair ping-ponging with no
     * ceiling at all. In a checked build MSVC does not fold that call
     * into a jump, so each lap also cost a stack frame carrying a
     * 32-entry frameIds array on a 24 KB kernel stack. Unbounded time at
     * DISPATCH_LEVEL and unbounded stack, from the one line that was
     * supposed to be the safety net.
     *
     * Now the budget spans the whole pump, however many times it is
     * re-entered, and running out of it ends the pump rather than
     * renewing it: the work is not dropped, because whatever set
     * TxPumpAgain will set it again through the next TxDataSend or
     * TxPeerBacklog. */
    InterlockedExchange(&Adapter->TxPumpActive, 0);
    if (InterlockedExchange(&Adapter->TxPumpAgain, 0) != 0) {
        if (rounds >= VWIFI_TAL_PUMP_MAX_ROUNDS) {
            VWIFI_WARN("TAL tx: pump re-entered again after %u rounds -- "
                       "stopping here and leaving the rest to the next "
                       "notification (port %u peer %u)",
                       rounds, PortId, PeerId);
            return;
        }
        /* Re-claim before going round. Losing the race means another
         * CPU owns the pump now and will do this work; hand the flag
         * back so its own tail check sees it. */
        if (InterlockedCompareExchange(&Adapter->TxPumpActive, 1, 0) != 0) {
            InterlockedExchange(&Adapter->TxPumpAgain, 1);
            return;
        }
        VWIFI_TAL_FIRST(4, "TAL tx: pump was re-entered while running -- "
                           "going round again for port %u peer %u "
                           "(%u round(s) used)",
                        PortId, PeerId, rounds);
        goto claimed;
    }
}

/* ============================================================
 * RX
 *
 * Two calls, not one. RxInorderDataIndication says frames exist for a
 * (peer, TID); the component decides when to take them and calls back
 * through RxGetMpdusHandler. Between those two moments the frames sit
 * on the adapter's RX queue, which is what this half of the file is.
 *
 * This is why VwifiRxDrainSta's NdisMIndicateReceiveNetBufferLists
 * moved nothing on an associated link: in WDI, NDIS is not the receiver
 * -- wdiwifi's RxMgr is, and it only learns about a frame through the
 * indication below.
 * ============================================================ */

VOID
VwifiTalRxIndicate(_Inout_ PVWIFI_ADAPTER Adapter,
                   _In_ PNET_BUFFER_LIST Nbl,
                   _In_ WDI_PEER_ID PeerId,
                   _In_ WDI_EXTENDED_TID ExTid)
{
    PNDIS_WDI_DATA_API api = Adapter->DataPathApi;
    PWDI_FRAME_METADATA md;
    NDIS_STATUS status = NDIS_STATUS_SUCCESS;
    KIRQL irql;

    /* Cleared FIRST, before any path that can bail out.
     *
     * NdisAllocateNetBufferAndNetBufferList does not promise a zeroed
     * MiniportReserved, and both failure exits below go to
     * VwifiTalRxReturn -- which reads that slot and frees whatever it
     * finds as a WDI_FRAME_METADATA. On a fresh NBL carrying pool
     * residue that is a free of a pointer nobody ever allocated. */
    NET_BUFFER_LIST_MINIPORT_RESERVED(Nbl)[0] = NULL;

    if (api == NULL || api->RxInorderDataIndication == NULL ||
        api->AllocateWiFiFrameMetaData == NULL) {
        VWIFI_TAL_ONCE("TAL rx: no indication/allocator in the data API -- "
                       "received frames cannot be delivered");
        VwifiTalRxReturn(Adapter, Nbl);
        return;
    }

    /* The metadata is the component's to allocate even on RX: it sizes
     * it, including whatever extra space the miniport asked for in
     * TalTxRxInitialize. Ours is only to fill in and attach. */
    md = api->AllocateWiFiFrameMetaData(Adapter->DataPathHandle);
    if (md == NULL) {
        VWIFI_WARN("TAL rx: frame metadata allocation failed -- dropping");
        VwifiTalRxReturn(Adapter, Nbl);
        return;
    }

    /* What the component handed back, before we write a single field.
     *
     * This driver fills in exactly two members of the structure below
     * and leaves the rest at whatever the allocator left there. That is
     * fine if the allocator zeroes it and not fine at all if it does
     * not: the RX metadata carries the frame's encryption and exemption
     * state, and residue that claims a frame arrived protected is a
     * frame the 802.11 layer discards above us -- on an open network,
     * silently, after answering SUCCESS to the indication. Which is
     * precisely the shape of the problem still outstanding.
     *
     * Raw bytes rather than named fields, because naming them would be
     * asserting a layout this driver has never verified. Once. */
    {
        const UCHAR *raw = (const UCHAR *)md;

        VWIFI_TAL_ONCE("TAL rx: fresh frame metadata at %p reads "
                       "%02x %02x %02x %02x %02x %02x %02x %02x "
                       "%02x %02x %02x %02x %02x %02x %02x %02x "
                       "%02x %02x %02x %02x %02x %02x %02x %02x "
                       "%02x %02x %02x %02x %02x %02x %02x %02x",
                       md,
                       raw[0],  raw[1],  raw[2],  raw[3],
                       raw[4],  raw[5],  raw[6],  raw[7],
                       raw[8],  raw[9],  raw[10], raw[11],
                       raw[12], raw[13], raw[14], raw[15],
                       raw[16], raw[17], raw[18], raw[19],
                       raw[20], raw[21], raw[22], raw[23],
                       raw[24], raw[25], raw[26], raw[27],
                       raw[28], raw[29], raw[30], raw[31]);
    }

    md->pNBL = Nbl;
    md->u.rxMetaData.PayloadType = WDI_FRAME_MSDU;

    /* MiniportReserved[0], the same place VwifiTxMetadata looks on the
     * way in. This direction is the reason to believe that IS the
     * contract: the NBL is ours, the component has to find the metadata
     * on it, and MiniportReserved is the only field of a
     * NET_BUFFER_LIST a miniport may write. */
    NET_BUFFER_LIST_MINIPORT_RESERVED(Nbl)[0] = md;

    NET_BUFFER_LIST_STATUS(Nbl) = NDIS_STATUS_SUCCESS;
    NET_BUFFER_LIST_NEXT_NBL(Nbl) = NULL;

    KeAcquireSpinLock(&Adapter->RxQueueLock, &irql);
    if (Adapter->RxQueueTail != NULL) {
        NET_BUFFER_LIST_NEXT_NBL(Adapter->RxQueueTail) = Nbl;
    } else {
        Adapter->RxQueueHead = Nbl;
    }
    Adapter->RxQueueTail = Nbl;
    Adapter->RxQueueCount++;
    KeReleaseSpinLock(&Adapter->RxQueueLock, irql);

    /* Not before the component is ready for this peer.
     *
     * TalTxRxPeerConfigHandler is what says the peer's data path is
     * built, and it arrives AFTER CONNECT_COMPLETE -- in the WPA2 trace
     * the gap was wide enough for the AP's EAPOL-Key message 1 to land
     * inside it. Indicating in that window is not harmless: the
     * component called RxGetMpdus, took the frame, answered PAUSED and
     * handed it straight back. The frame was consumed and discarded,
     * the four-way handshake never got its first message, and the OS
     * disconnected a second and a half later with a link that had
     * associated perfectly.
     *
     * Held instead, and announced by VwifiTalRxAnnounceHeld the moment
     * the peer is configured. Nothing is lost and nothing is indicated
     * to a peer that cannot yet receive it. */
    {
        PVWIFI_PEER peer = VwifiPeerFind(Adapter, PeerId);

        if (peer != NULL && !peer->Configured) {
            VWIFI_TAL_FIRST(4, "TAL rx: holding %u frame(s) for peer %u tid "
                               "%u -- the component has not configured this "
                               "peer for data yet",
                            Adapter->RxQueueCount, PeerId, ExTid);
            return;
        }
    }

    /* Nor while the component has said it is paused.
     *
     * Waiting for the peer to be configured was not enough: the
     * indication that followed it returned PAUSED anyway, and RxResume
     * came twenty-four milliseconds later. So the component is not
     * ready at peer-config time, it is ready when it says so, and
     * RxResume is the only thing that says so.
     *
     * This latch was tried once before and stalled forever because
     * RxResume never fired. It does now -- every trace since the peer
     * path started working has one. VwifiHeartbeatRxWatchdog is the
     * insurance against the old failure returning. */
    if (Adapter->RxPaused) {
        VWIFI_TAL_FIRST(4, "TAL rx: holding %u frame(s) for peer %u tid %u "
                           "-- the component paused RX and has not resumed",
                        Adapter->RxQueueCount, PeerId, ExTid);
        return;
    }

    /* Bracketed on purpose. The guest has hard-hung twice inside this
     * region, and "entering" without a matching "returned" is the
     * difference between the component wedging on a frame we handed it
     * and this driver wedging on its own. Neither was distinguishable
     * from the other in any trace so far. */
    VWIFI_TAL_FIRST(4, "TAL rx: entering indication for peer %u tid %u "
                       "(%u frame(s) queued)",
                    PeerId, ExTid, Adapter->RxQueueCount);

    /* Queued before the indication, because the component is entitled
     * to call RxGetMpdus from inside it. */
    api->RxInorderDataIndication(Adapter->DataPathHandle,
                                 WDI_RX_INDICATION_DISPATCH_GENERAL,
                                 PeerId, ExTid, NULL, &status);

    VWIFI_TAL_FIRST(4, "TAL rx: indication returned 0x%08x %s",
                    status, VwifiNdisStatusName(status));

    /* Counted, and that is all.
     *
     * A previous version latched on NDIS_STATUS_PAUSED and stopped
     * indicating until RxResumeHandler fired. RxResume never fired, so
     * the latch was permanent: four DHCP offers arrived after it and
     * every one of them was queued in silence. The link was up, the
     * frames were in the ring, and the driver had decided on its own
     * not to mention them.
     *
     * The trace also says the status does not mean what the latch
     * assumed. The component called RxGetMpdus from inside this very
     * indication and took the frame before answering PAUSED -- so
     * PAUSED is not "I did not receive this", and treating it as "do
     * not send me another" was reading a status as an instruction.
     *
     * So: report it, keep indicating. Every indication drains the whole
     * queue through RxGetMpdus whatever it returns afterwards, which is
     * the behaviour that actually moves frames. RxResumeHandler stays
     * wired for the case where the component does use it. */
    if (status == NDIS_STATUS_PAUSED) {
        /* Latched so the next frame is held rather than offered into
         * the same refusal -- and, more to the point, so
         * VwifiTalRxReturn knows the frames coming back were never
         * delivered. */
        InterlockedExchange(&Adapter->RxPaused, 1);
    }

    if (status == NDIS_STATUS_SUCCESS) {
        InterlockedIncrement(&Adapter->RxIndOk);
    } else {
        LONG n = InterlockedIncrement(&Adapter->RxIndNotOk);

        /* Powers of two: enough to see it start and to see whether it is
         * every frame or an occasional one, without a line per frame. */
        if ((n & (n - 1)) == 0) {
            VWIFI_WARN("TAL rx: indication %u for peer %u tid %u returned "
                       "0x%08x %s -- still indicating",
                       n, PeerId, ExTid, status, VwifiNdisStatusName(status));
        }
    }
}

/* Announce whatever is being held, if anything is.
 *
 * One indication is enough however many frames are queued: RxGetMpdus
 * drains the entire queue in one call. Called from the two places that
 * mean "the component can take frames now" -- the peer being configured
 * for data, and RxResumeHandler.
 *
 * The indication is made here rather than through VwifiTalRxIndicate,
 * which takes an NBL and attaches metadata to it. There is no new frame
 * at this point -- the frames are already queued and already carry
 * their metadata -- so there is nothing to pass it, and handing it NULL
 * would fault on the first line of that function. */
VOID
VwifiTalRxAnnounceHeld(_Inout_ PVWIFI_ADAPTER Adapter, _In_z_ PCSTR Why)
{
    PNDIS_WDI_DATA_API api = Adapter->DataPathApi;
    PVWIFI_PEER peer;
    NDIS_STATUS status = NDIS_STATUS_SUCCESS;
    ULONG held = Adapter->RxQueueCount;

    if (held == 0) return;

    peer = VwifiPeerFirstActive(Adapter);
    if (api == NULL || api->RxInorderDataIndication == NULL || peer == NULL) {
        return;
    }

    /* FROM_RX_RESUME_FRAMES, not DISPATCH_GENERAL.
     *
     * dot11wdi.h has a level for exactly this -- frames announced
     * because a pause ended rather than because they just arrived --
     * and this function is the only thing that makes that kind of
     * announcement. Saying GENERAL described the wrong situation to the
     * component. */
    VWIFI_INFO("TAL rx: %s -- announcing %u held frame(s)", Why, held);
    api->RxInorderDataIndication(Adapter->DataPathHandle,
                                 WDI_RX_INDICATION_FROM_RX_RESUME_FRAMES,
                                 peer->PeerId, 0, NULL, &status);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_WARN("TAL rx: announcing held frames returned 0x%08x %s",
                   status, VwifiNdisStatusName(status));
    }
}

/* The component saying it will take frames again. Unconditional: an
 * earlier version returned early unless a pause latch was set, so a
 * resume that arrived without one -- which is what actually happens --
 * left the held frames unmentioned. */
VOID
VwifiTalRxOnResume(_Inout_ PVWIFI_ADAPTER Adapter)
{
    InterlockedExchange(&Adapter->RxPaused, 0);
    VwifiTalRxAnnounceHeld(Adapter, "the component resumed RX");
}

/* Hand the whole queue over. Called from RxGetMpdusHandler.
 *
 * Everything, not a filtered subset: this device has one peer and one
 * traffic class, so a per-(peer, TID) split would be bookkeeping with
 * nothing to keep. If that stops being true, the queue grows a key
 * before it grows a filter. */
VOID
VwifiTalRxTakeQueued(_Inout_ PVWIFI_ADAPTER Adapter,
                     _Outptr_result_maybenull_ PNET_BUFFER_LIST *Out)
{
    KIRQL irql;

    KeAcquireSpinLock(&Adapter->RxQueueLock, &irql);
    *Out = Adapter->RxQueueHead;
    Adapter->RxQueueHead  = NULL;
    Adapter->RxQueueTail  = NULL;
    Adapter->RxQueueCount = 0;
    KeReleaseSpinLock(&Adapter->RxQueueLock, irql);
}

/* Give one RX NBL back: free its metadata, then release the NBL and
 * the ring slot behind it. */
VOID
VwifiTalRxReturn(_Inout_ PVWIFI_ADAPTER Adapter, _In_ PNET_BUFFER_LIST Nbl)
{
    PNDIS_WDI_DATA_API api = Adapter->DataPathApi;
    PWDI_FRAME_METADATA md =
        (PWDI_FRAME_METADATA)NET_BUFFER_LIST_MINIPORT_RESERVED(Nbl)[0];

    /* Same plausibility gate as the TX side. VwifiTalRxIndicate clears
     * this slot before it can fail, so a garbage value here should be
     * impossible -- and "should be impossible" is not a reason to hand
     * it to a deallocator. */
    if (!VwifiPlausibleKernelPointer(md)) md = NULL;

    if (md != NULL) {
        NET_BUFFER_LIST_MINIPORT_RESERVED(Nbl)[0] = NULL;
        if (api != NULL && api->FreeWiFiFrameMetaData != NULL) {
            api->FreeWiFiFrameMetaData(Adapter->DataPathHandle, md);
        }
    }

    /* One NBL at a time, so the chain the component returned cannot be
     * followed after its first element has been freed. */
    NET_BUFFER_LIST_NEXT_NBL(Nbl) = NULL;
    VwifiMiniportReturnNetBufferLists((NDIS_HANDLE)Adapter, Nbl, 0);
}

/* How many times one frame may be put back before it is given up on.
 * A frame the component keeps taking and refusing must not circulate
 * forever; two attempts is enough to cross a pause and cheap to lose. */
#define VWIFI_RX_REQUEUE_MAX 2

/* Give one RX NBL back, unless it was never delivered.
 *
 * The component collects through RxGetMpdus BEFORE it decides it is
 * paused: the trace shows it taking the AP's EAPOL-Key message 1,
 * answering PAUSED, and handing the frame straight back. Freeing it
 * there is how the four-way handshake lost its first message twice
 * over -- the frame was on the wire, in the ring, and correctly
 * indicated, and this function threw it away.
 *
 * So while the pause is latched, a returned frame goes back on the
 * queue instead, keeping its metadata, and is announced again when
 * RxResume arrives. Bounded, because a frame that is refused every time
 * would otherwise never leave. */
VOID
VwifiTalRxReturnOrRequeue(_Inout_ PVWIFI_ADAPTER Adapter,
                          _In_ PNET_BUFFER_LIST Nbl)
{
    ULONG_PTR tries;

    if (!Adapter->RxPaused) {
        VwifiTalRxReturn(Adapter, Nbl);
        return;
    }

    tries = (ULONG_PTR)NET_BUFFER_LIST_MINIPORT_RESERVED(Nbl)[2];
    if (tries >= VWIFI_RX_REQUEUE_MAX) {
        VWIFI_WARN("TAL rx: frame refused %u times while paused -- dropping",
                   (ULONG)tries);
        VwifiTalRxReturn(Adapter, Nbl);
        return;
    }
    NET_BUFFER_LIST_MINIPORT_RESERVED(Nbl)[2] = (PVOID)(tries + 1);

    {
        KIRQL irql;

        NET_BUFFER_LIST_NEXT_NBL(Nbl) = NULL;
        KeAcquireSpinLock(&Adapter->RxQueueLock, &irql);
        if (Adapter->RxQueueTail != NULL) {
            NET_BUFFER_LIST_NEXT_NBL(Adapter->RxQueueTail) = Nbl;
        } else {
            Adapter->RxQueueHead = Nbl;
        }
        Adapter->RxQueueTail = Nbl;
        Adapter->RxQueueCount++;
        KeReleaseSpinLock(&Adapter->RxQueueLock, irql);
    }

    VWIFI_TAL_FIRST(4, "TAL rx: the component returned a frame it took while "
                       "paused -- requeued, not freed");
}

/* Drop anything indicated and never collected.
 *
 * For RxFlush and for teardown. A queued NBL holds an RX ring slot, so
 * leaving them is not a leak that shows up later -- it is a receive
 * ring that shrinks every time a flush happens. */
VOID
VwifiTalRxFlushQueued(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PNET_BUFFER_LIST chain = NULL;
    ULONG n = 0;

    VwifiTalRxTakeQueued(Adapter, &chain);
    while (chain != NULL) {
        PNET_BUFFER_LIST next = NET_BUFFER_LIST_NEXT_NBL(chain);

        VwifiTalRxReturn(Adapter, chain);
        chain = next;
        n++;
    }
    if (n > 0) {
        VWIFI_INFO("TAL rx: discarded %u queued frame(s)", n);
    }
}
