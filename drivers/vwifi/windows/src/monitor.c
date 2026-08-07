/*
 * vwifi — monitor.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 1.5: monitor-mode receive and injection.
 *
 * RX: when the device delivers a raw 802.11 frame (VWIFI_RX_F_RAW),
 * we wrap it in a NET_BUFFER_LIST with a DOT11_EXTSTA_RECV_CONTEXT
 * attached as Native-802.11 OOB media-specific info. Npcap reads
 * that context, synthesizes a radiotap header, and hands it to
 * Wireshark as DLT_IEEE802_11_RADIOTAP.
 *
 * TX (injection): frames arriving from Npcap in monitor mode are
 * already 802.11 (possibly with a radiotap prefix we strip). We push
 * them to the TX ring with VWIFI_TX_F_INJECT so the device sources
 * the medium tx_mac from the frame's own addr2.
 *
 * The DOT11_EXTSTA_RECV_CONTEXT layout and the rule set for filling
 * it come from windot11.h; see the reference in development_plan.md.
 */

#include "vwifi_drv.h"
#include <windot11.h>

/* ============================================================
 * NBL pool setup — one pool for RX indications, allocated at
 * adapter init and torn down at halt.
 * ============================================================ */

NDIS_STATUS
VwifiRxNblPoolCreate(_Inout_ PVWIFI_ADAPTER Adapter)
{
    NET_BUFFER_LIST_POOL_PARAMETERS params = { 0 };

    params.Header.Type     = NDIS_OBJECT_TYPE_DEFAULT;
    params.Header.Revision = NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
    params.Header.Size     = NDIS_SIZEOF_NET_BUFFER_LIST_POOL_PARAMETERS_REVISION_1;
    params.ProtocolId      = NDIS_PROTOCOL_ID_DEFAULT;
    params.fAllocateNetBuffer = TRUE;
    params.PoolTag         = VWIFI_POOL_TAG;
    /* Sized here, not per allocation. Every NBL from this pool then
     * carries its context already, and the allocation calls ask for
     * none -- see VWIFI_RX_NBL_CONTEXT_SIZE for why asking per
     * allocation failed. */
    params.ContextSize     = VWIFI_RX_NBL_CONTEXT_SIZE;

    Adapter->RxNblPool = NdisAllocateNetBufferListPool(
        Adapter->MiniportAdapterHandle, &params);
    if (!Adapter->RxNblPool) {
        VWIFI_ERR("RX NBL pool allocation failed");
        return NDIS_STATUS_RESOURCES;
    }
    return NDIS_STATUS_SUCCESS;
}

VOID
VwifiRxNblPoolDestroy(_Inout_ PVWIFI_ADAPTER Adapter)
{
    if (Adapter->RxNblPool) {
        NdisFreeNetBufferListPool(Adapter->RxNblPool);
        Adapter->RxNblPool = NULL;
    }
}

/* ============================================================
 * Data-rate conversion: the medium uses ath9k rate codes; the
 * DOT11_EXTSTA_RECV_CONTEXT wants ucDataRate in 500 kbps units.
 * We keep a tiny lookup for the OFDM/DSSS rates we care about.
 *
 * Not static: the STA drain in data.c attaches the same receive
 * context now that VWIFI_CTRL_RX_80211 makes both paths carry MPDUs,
 * and it needs the same conversion. Declared in vwifi_drv.h.
 * ============================================================ */

UCHAR
VwifiRateCodeTo500Kbps(UCHAR rate_code)
{
    /* ath9k OFDM rate codes (see ath9k hw). Values in 500 kbps units:
     * 6 Mbps = 12, 9 = 18, 12 = 24, 18 = 36, 24 = 48, 36 = 72,
     * 48 = 96, 54 = 108. DSSS: 1 = 2, 2 = 4, 5.5 = 11, 11 = 22. */
    switch (rate_code) {
    case 0x0B: return 12;   /* 6 Mbps  */
    case 0x0F: return 18;   /* 9 Mbps  */
    case 0x0A: return 24;   /* 12 Mbps */
    case 0x0E: return 36;   /* 18 Mbps */
    case 0x09: return 48;   /* 24 Mbps */
    case 0x0D: return 72;   /* 36 Mbps */
    case 0x08: return 96;   /* 48 Mbps */
    case 0x0C: return 108;  /* 54 Mbps */
    case 0x1B: return 2;    /* 1 Mbps  */
    case 0x1A: return 4;    /* 2 Mbps  */
    case 0x19: return 11;   /* 5.5 Mbps*/
    case 0x18: return 22;   /* 11 Mbps */

    /*
     * Anything else has no legacy rate to report, and rate codes at
     * 0x80 and above are exactly that: HT/VHT MCS values, which this
     * field cannot express. Report 0 rather than "6 Mbps".
     *
     * A capture showing 0 Mb/s is obviously incomplete and gets
     * questioned; a capture showing 6 Mb/s for every 802.11n frame
     * looks right and gets believed. The Linux driver's monitor path
     * and medium/tools/vwifi_dump.py make the same choice.
     */
    default:   return 0;
    }
}

/* ============================================================
 * VwifiRxDrainMonitor — the real monitor-mode RX path.
 *
 * For each device-completed RX descriptor with VWIFI_RX_F_RAW set,
 * build an NBL over the slot's buffer, attach a
 * DOT11_EXTSTA_RECV_CONTEXT, and indicate up. NBLs are returned
 * via VwifiReturnNetBufferLists which re-arms the slot.
 * ============================================================ */

VOID
VwifiRxDrainMonitor(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_RING ring = &Adapter->RxRing;
    PNET_BUFFER_LIST indicate_head = NULL;
    PNET_BUFFER_LIST indicate_tail = NULL;
    ULONG indicated = 0;
    /* Two independent stops, because the loop's own exit condition is
     * not trustworthy on its own -- see VWIFI_ADAPTER::RxOutstanding.
     * `guard` bounds the pass to one lap of the ring no matter what the
     * descriptors say; the outstanding check keeps the lap from being
     * reachable in the first place. */
    ULONG guard = 0;

    for (;;) {
        ULONG idx = ring->NextIndex & ring->Mask;
        struct vwifi_rx_desc *d = (struct vwifi_rx_desc *)
            ((PUCHAR)ring->VirtualAddress + idx * ring->DescSize);

        if (++guard > ring->NumDescs) {
            VWIFI_WARN("%s: drained a full ring (%u descriptors) in one "
                       "pass without finding an armed slot -- stopping. "
                       "%d slot(s) outstanding",
                       "rx(mon)", ring->NumDescs, Adapter->RxOutstanding);
            break;
        }

        if (d->flags & VWIFI_DESC_F_OWN) {
            break;   /* still owned by device */
        }

        /* Only handle raw frames here; non-raw (STA-mode data) frames
         * are Phase 3's path. In Phase 1.5 the device only sets RAW
         * in monitor mode, so anything without it we just re-arm. */
        if (!(d->flags & VWIFI_RX_F_RAW) || d->frame_len == 0) {
            goto rearm;
        }

        /* Stop one slot short of owning the whole ring. The frame is
         * dropped and its slot re-armed -- which is safe precisely
         * because no NBL has taken it yet -- so the device keeps a
         * place to write and the drain keeps making progress. Losing
         * frames while the returns catch up beats lapping. */
        if (Adapter->RxOutstanding >= (LONG)(ring->NumDescs - 1)) {
            VWIFI_TAL_ONCE("rx(mon): all but one RX slot outstanding -- "
                           "dropping frames until the component returns "
                           "some");
            goto rearm;
        }

        {
            /* The RX slot's DMA buffer holds the raw 802.11 frame. */
            PUCHAR frame_va = (PUCHAR)Adapter->RxBufferPoolVa
                            + (SIZE_T)idx * VWIFI_RX_BUFFER_SIZE;
            NDIS_PHYSICAL_ADDRESS frame_pa;
            frame_pa.QuadPart = Adapter->RxBufferPoolPa.QuadPart
                              + (LONGLONG)idx * VWIFI_RX_BUFFER_SIZE;

            /* Build an MDL over the frame bytes. */
            PMDL mdl = NdisAllocateMdl(Adapter->MiniportAdapterHandle,
                                       frame_va, d->frame_len);
            if (!mdl) {
                VWIFI_WARN("rx: MDL alloc failed, dropping frame");
                goto rearm;
            }

            PNET_BUFFER_LIST nbl = NdisAllocateNetBufferAndNetBufferList(
                Adapter->RxNblPool, 0, 0,
                mdl, 0, d->frame_len);
            if (!nbl) {
                VWIFI_WARN("rx: NBL alloc failed, dropping frame");
                NdisFreeMdl(mdl);
                goto rearm;
            }

            /* Remember which slot backs this NBL so we can re-arm it
             * on return. */
            VwifiRxNblSetSlot(nbl, idx);

            /* Build and attach the DOT11_EXTSTA_RECV_CONTEXT. This is
             * the OOB info Npcap turns into radiotap. */
            PDOT11_EXTSTA_RECV_CONTEXT rc = &Adapter->RxRecvContext[idx];
            RtlZeroMemory(rc, sizeof(*rc));
            rc->Header.Type     = NDIS_OBJECT_TYPE_DEFAULT;
            rc->Header.Revision = DOT11_EXTSTA_RECV_CONTEXT_REVISION_1;
            rc->Header.Size     = sizeof(DOT11_EXTSTA_RECV_CONTEXT);

            rc->uReceiveFlags = DOT11_RECV_FLAG_RAW_PACKET;
            /* The medium never carries an FCS and we don't synthesize
             * one, so we do NOT set _FCS_FAILURE (frame is "good", just
             * FCS-absent). We do have a TSF, so mark timestamp valid. */
            rc->uReceiveFlags |= DOT11_RECV_FLAG_RAW_PACKET_TIMESTAMP;

            rc->uPhyId = dot11_phy_type_erp;   /* 802.11g / 2.4 GHz ERP */
            rc->uChCenterFrequency = d->channel_freq;
            rc->usNumberOfMPDUsReceived = 1;
            rc->lRSSI = d->rssi;
            rc->ucDataRate = VwifiRateCodeTo500Kbps(d->rate_code);
            rc->uSizeMediaSpecificInfo = 0;
            rc->pvMediaSpecificInfo = NULL;
            rc->ullTimestamp = d->tsf;

            NET_BUFFER_LIST_INFO(nbl, MediaSpecificInformation) = rc;

            NET_BUFFER_LIST_STATUS(nbl) = NDIS_STATUS_SUCCESS;

            /* Chain for a batched indication. */
            NET_BUFFER_LIST_NEXT_NBL(nbl) = NULL;
            if (indicate_tail) {
                NET_BUFFER_LIST_NEXT_NBL(indicate_tail) = nbl;
            } else {
                indicate_head = nbl;
            }
            indicate_tail = nbl;
            indicated++;
            /* The slot now belongs to this NBL and stays un-armed until
             * VwifiMiniportReturnNetBufferLists gives it back. */
            InterlockedIncrement(&Adapter->RxOutstanding);

            /* NOTE: we do NOT re-arm this slot here — it stays owned by
             * the NBL until VwifiReturnNetBufferLists reclaims it. We
             * still advance NextIndex so we can process later slots. */
            ring->NextIndex = (ring->NextIndex + 1) & ring->Mask;
            continue;
        }

    rearm:
        d->flags      = VWIFI_DESC_F_OWN;
        d->frame_len  = 0;
        d->buffer_len = VWIFI_RX_BUFFER_SIZE;
        ring->NextIndex = (ring->NextIndex + 1) & ring->Mask;
    }

    /* Ack the vector: tell the device how far we've consumed. */
    VwifiWrite32(Adapter, ring->RegHead, ring->NextIndex);

    if (indicated) {
        /* ReceiveFlags = 0: we hand NBL ownership to NDIS and reclaim
         * each NBL (re-arming its ring slot) in VwifiMiniportReturn
         * NetBufferLists. Do NOT pass NDIS_RECEIVE_FLAGS_RESOURCES —
         * that would demand synchronous return and we'd have to re-arm
         * inline, defeating the batched indication. */
        NdisMIndicateReceiveNetBufferLists(
            Adapter->MiniportAdapterHandle, indicate_head,
            NDIS_DEFAULT_PORT_NUMBER, indicated, 0);
    }
}

/* ============================================================
 * VwifiReturnNetBufferLists — NDIS returns RX NBLs here after the
 * stack (and Npcap) is done. Free the NBL/MDL and re-arm the slot.
 * ============================================================ */

_Use_decl_annotations_
VOID
VwifiMiniportReturnNetBufferLists(
    NDIS_HANDLE MiniportAdapterContext,
    PNET_BUFFER_LIST NetBufferLists,
    ULONG ReturnFlags)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportAdapterContext;
    PNET_BUFFER_LIST nbl, next;
    UNREFERENCED_PARAMETER(ReturnFlags);

    for (nbl = NetBufferLists; nbl; nbl = next) {
        next = NET_BUFFER_LIST_NEXT_NBL(nbl);

        ULONG slot = VwifiRxNblGetSlot(nbl);

        /* Free the MDL we allocated, then the NBL. */
        PNET_BUFFER nb = NET_BUFFER_LIST_FIRST_NB(nbl);
        PMDL mdl = NET_BUFFER_FIRST_MDL(nb);
        if (mdl) {
            NdisFreeMdl(mdl);
        }
        NdisFreeNetBufferList(nbl);

        InterlockedDecrement(&adapter->RxOutstanding);

        /* Checked before it is used as an index, because the three
         * writes below go through whatever this produces.
         *
         * There is no defensible way for a returned NBL to name a slot
         * outside the ring, and that is exactly why it has to be
         * tested: the failure is silent and unbounded. An unchecked
         * index here does not corrupt a frame, it writes six bytes to
         * an arbitrary kernel address, and the machine dies later
         * somewhere with no connection to this driver at all.
         *
         * Leaking the slot is the right answer when it happens. The
         * ring loses one descriptor; the drain already copes with slots
         * that never come back, and it beats the alternative by a
         * distance that does not need arguing. */
        if (slot >= adapter->RxRing.NumDescs) {
            VWIFI_ERR("rx return: NBL %p names ring slot %u, but the ring "
                      "has %u -- refusing to re-arm it. The slot is lost; "
                      "writing through this index would not have been.",
                      nbl, slot, adapter->RxRing.NumDescs);
            continue;
        }

        /* Re-arm the RX slot so the device can reuse it, and account
         * for it: this is the moment the slot stops being outstanding,
         * and the drain's lap guard reads that count. */
        struct vwifi_rx_desc *d = (struct vwifi_rx_desc *)
            ((PUCHAR)adapter->RxRing.VirtualAddress
             + slot * adapter->RxRing.DescSize);
        d->frame_len  = 0;
        d->buffer_len = VWIFI_RX_BUFFER_SIZE;
        /* Memory barrier so buffer_len is visible before OWN. */
        KeMemoryBarrier();
        d->flags = VWIFI_DESC_F_OWN;
    }
}

/* ============================================================
 * Injection TX — called from the send path when in monitor mode.
 *
 * Npcap sends raw 802.11 frames (Wireshark's injection). If a
 * radiotap header is present (first byte is radiotap version 0),
 * strip it. Push the 802.11 frame to the TX ring with INJECT set.
 * ============================================================ */

static ULONG
VwifiStripRadiotap(_In_reads_bytes_(len) PUCHAR data, _In_ ULONG len,
                   _Out_ PULONG offset)
{
    /* Radiotap header: u8 version(0), u8 pad, u16 length (LE). */
    if (len >= 4 && data[0] == 0x00) {
        ULONG rtlen = data[2] | ((ULONG)data[3] << 8);
        if (rtlen < len) {
            *offset = rtlen;
            return len - rtlen;
        }
    }
    *offset = 0;
    return len;
}

/* ============================================================
 * Control helpers — push mode/channel/filter to the device.
 * Called at PASSIVE_LEVEL from OID handlers.
 * ============================================================ */

NDIS_STATUS
VwifiSetOpMode(_Inout_ PVWIFI_ADAPTER Adapter, ULONG Mode)
{
    struct vwifi_op_mode m = { 0 };
    ULONG out_len = 0;
    NDIS_STATUS st;

    m.mode = Mode;
    st = VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_OP_MODE,
                           &m, sizeof(m), NULL, &out_len);
    if (st == NDIS_STATUS_SUCCESS) {
        Adapter->OpMode = Mode;
        VWIFI_INFO("op mode -> %u", Mode);
    }
    return st;
}

NDIS_STATUS
VwifiSetChannel(_Inout_ PVWIFI_ADAPTER Adapter, USHORT FreqMhz)
{
    struct vwifi_channel c = { 0 };
    ULONG out_len = 0;
    NDIS_STATUS st;

    c.primary_freq = FreqMhz;
    /* 2.4 GHz band flag; HT20 by default. Values come from
     * ath9k_medium.h channel flags. */
    c.flags = 0;   /* leave phy-specific flags default for Phase 1.5 */
    st = VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_CHANNEL,
                           &c, sizeof(c), NULL, &out_len);
    if (st == NDIS_STATUS_SUCCESS) {
        Adapter->CurrentFreq = FreqMhz;
        VWIFI_INFO("channel -> %u MHz", FreqMhz);
    }
    return st;
}

NDIS_STATUS
VwifiSetRawFilter(_Inout_ PVWIFI_ADAPTER Adapter, ULONG Mask)
{
    struct vwifi_raw_filter f = { 0 };
    ULONG out_len = 0;
    NDIS_STATUS st;

    f.mask = Mask;
    st = VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_RAW_FILTER,
                           &f, sizeof(f), NULL, &out_len);
    if (st == NDIS_STATUS_SUCCESS) {
        Adapter->RawFilter = Mask;
        VWIFI_INFO("raw filter -> 0x%08x", Mask);
    }
    return st;
}

NDIS_STATUS
VwifiInjectFrame(_Inout_ PVWIFI_ADAPTER Adapter,
                 _In_reads_bytes_(FrameLen) PUCHAR Frame, _In_ ULONG FrameLen)
{
    PVWIFI_RING ring = &Adapter->TxRing;
    ULONG slot;
    ULONG rt_offset;
    ULONG len_802_11;
    struct vwifi_tx_desc *desc;
    PUCHAR tx_buf;

    len_802_11 = VwifiStripRadiotap(Frame, FrameLen, &rt_offset);
    if (len_802_11 == 0 || len_802_11 > VWIFI_RX_BUFFER_SIZE) {
        return NDIS_STATUS_INVALID_LENGTH;
    }

    slot = InterlockedIncrement((volatile LONG *)&ring->NextIndex) - 1;
    slot &= ring->Mask;

    /* Copy the 802.11 frame into the per-slot TX buffer. */
    tx_buf = (PUCHAR)Adapter->TxBufferPoolVa + (SIZE_T)slot * VWIFI_RX_BUFFER_SIZE;
    RtlCopyMemory(tx_buf, Frame + rt_offset, len_802_11);

    desc = (struct vwifi_tx_desc *)
        ((PUCHAR)ring->VirtualAddress + slot * ring->DescSize);
    RtlZeroMemory(desc, sizeof(*desc));
    desc->frame_addr = Adapter->TxBufferPoolPa.QuadPart
                     + (LONGLONG)slot * VWIFI_RX_BUFFER_SIZE;
    desc->frame_len  = (USHORT)len_802_11;
    desc->flags      = VWIFI_DESC_F_OWN | VWIFI_TX_F_INJECT;
    KeMemoryBarrier();

    VwifiWrite32(Adapter, VWIFI_REG_TX_RING_DOORBELL, slot + 1);
    return NDIS_STATUS_SUCCESS;
}
