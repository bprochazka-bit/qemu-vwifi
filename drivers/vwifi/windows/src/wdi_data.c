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
 * plumbing between the component and those. The device does the
 * 802.3 <-> 802.11 conversion, so what crosses this boundary is
 * ordinary Ethernet frames in both directions.
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
         * The device treats a non-injected STA frame as 802.3 and
         * builds the 802.11 header itself, so this only works if the
         * component hands down Ethernet. Nothing in the headers says
         * which it is, and the failure mode either way is silent: an
         * 802.11 MPDU fed to sta_tx_8023_to_80211 either fails its
         * length check or produces a frame with a nonsense EtherType,
         * and both look from here like a frame that was sent fine.
         *
         * So read it off the wire. Bytes 12-13 of an Ethernet II frame
         * are the EtherType -- 0x0800 for the DHCP that is failing,
         * 0x0806 for ARP, 0x86dd for v6. An 802.11 data MPDU has
         * frame-control in bytes 0-1 (0x08 0x0x for data) and its
         * addresses where Ethernet has none. */
        if (flat != NULL && len >= 24) {
            VWIFI_TAL_ONCE(
                "TAL tx: first frame %u bytes: "
                "%02x %02x %02x %02x %02x %02x | %02x %02x %02x %02x %02x %02x "
                "| %02x %02x | %02x %02x %02x %02x %02x %02x %02x %02x -- %s",
                len,
                flat[0], flat[1], flat[2], flat[3], flat[4], flat[5],
                flat[6], flat[7], flat[8], flat[9], flat[10], flat[11],
                flat[12], flat[13],
                flat[14], flat[15], flat[16], flat[17],
                flat[18], flat[19], flat[20], flat[21],
                (flat[12] == 0x08 && (flat[13] == 0x00 || flat[13] == 0x06))
                    ? "802.3: EtherType IPv4/ARP at 12 -- the device's "
                      "assumption holds"
                    : (flat[12] == 0x86 && flat[13] == 0xdd)
                        ? "802.3: EtherType IPv6 at 12 -- the device's "
                          "assumption holds"
                        : (((flat[0] >> 2) & 0x3) == 2)
                            ? "NOT 802.3: byte 0 says this is an 802.11 DATA "
                              "MPDU, and the device is building a second "
                              "802.11 header on top of it"
                            : "NOT 802.3 and not an 802.11 data frame either "
                              "-- shape unknown");
        }

        st = (flat != NULL) ? VwifiTxDataFrame(Adapter, flat, len)
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
         * TxDataSend/TxPeerBacklog brings us straight back. */
        if (++rounds >= 16) {
            VWIFI_WARN("TAL tx: 16 dequeue rounds in one pump -- yielding");
            break;
        }
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

    /* Queued before the indication, because the component is entitled
     * to call RxGetMpdus from inside it. */
    api->RxInorderDataIndication(Adapter->DataPathHandle,
                                 WDI_RX_INDICATION_DISPATCH_GENERAL,
                                 PeerId, ExTid, NULL, &status);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_TAL_FIRST(8, "TAL rx: indication for peer %u tid %u returned "
                           "0x%08x %s",
                        PeerId, ExTid, status, VwifiNdisStatusName(status));
    }
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
