/*
 * vwifi — data.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 3: STA-mode data path.
 *
 * What shape the frames are is stated once, in the frame-format
 * contract next to the VwifiTxDataFrame declaration in vwifi_drv.h.
 * It is not restated here. In short: in WDI mode both directions
 * carry 802.11 MPDUs, because wdiwifi.sys does the 802.3 conversion
 * above the miniport.
 *
 * This file is deliberately thin:
 *
 *   TX: stage the frame into a TX slot, publish the descriptor, ring
 *       the doorbell. ExtraFlags decides whether the device treats it
 *       as an MPDU to pass through or as 802.3 to encapsulate.
 *   RX: the device hands us MPDUs with VWIFI_RX_F_RAW set (it was
 *       asked for them with VWIFI_CTRL_RX_80211); wrap each in an NBL
 *       and indicate it up.
 *
 * Monitor-mode RX (raw 802.11, flagged as a capture) lives in
 * monitor.c; the RX DPC routes to one or the other by op mode.
 */

#include "vwifi_drv.h"

/* ============================================================
 * TX — stage a frame for the device
 * ============================================================ */

NDIS_STATUS
VwifiTxDataFrame(_Inout_ PVWIFI_ADAPTER Adapter,
                 _In_reads_bytes_(FrameLen) PUCHAR Frame,
                 _In_ ULONG FrameLen,
                 _In_ USHORT ExtraFlags)
{
    PVWIFI_RING ring = &Adapter->TxRing;
    ULONG slot;
    ULONG minLen;
    struct vwifi_tx_desc *desc;
    PUCHAR tx_buf;

    /* The minimum length is a property of the shape, not of the ring:
     * 24 is the shortest 802.11 MPDU header, 14 the shortest 802.3
     * one. This used to be a flat 14, which on the MPDU path let a
     * 14-to-23-byte runt reach a device that would then read an
     * addr3 that was never sent. */
    minLen = (ExtraFlags & VWIFI_TX_F_80211) ? 24u : 14u;

    if (FrameLen < minLen || FrameLen > VWIFI_RX_BUFFER_SIZE) {
        return NDIS_STATUS_INVALID_LENGTH;
    }
    if (!Adapter->Associated) {
        return NDIS_STATUS_PAUSED;
    }

    slot = InterlockedIncrement((volatile LONG *)&ring->NextIndex) - 1;
    slot &= ring->Mask;

    tx_buf = (PUCHAR)Adapter->TxBufferPoolVa
           + (SIZE_T)slot * VWIFI_RX_BUFFER_SIZE;
    RtlCopyMemory(tx_buf, Frame, FrameLen);

    desc = (struct vwifi_tx_desc *)
        ((PUCHAR)ring->VirtualAddress + slot * ring->DescSize);
    RtlZeroMemory(desc, sizeof(*desc));
    desc->frame_addr = Adapter->TxBufferPoolPa.QuadPart
                     + (LONGLONG)slot * VWIFI_RX_BUFFER_SIZE;
    desc->frame_len  = (USHORT)FrameLen;
    /* No INJECT flag. Without VWIFI_TX_F_80211 the device treats this
     * as 802.3 and builds the 802.11 data frame itself; with it, the
     * frame already has its own header and is sent as it stands --
     * still association-checked, still encrypted if a key is set. */
    desc->flags      = (USHORT)(VWIFI_DESC_F_OWN | ExtraFlags);
    KeMemoryBarrier();

    VwifiWrite32(Adapter, VWIFI_REG_TX_RING_DOORBELL, slot + 1);
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * RX — indicate received frames from the device
 *
 * Mirrors VwifiRxDrainMonitor's NBL lifetime: the ring slot stays
 * owned by the NBL until VwifiMiniportReturnNetBufferLists reclaims
 * it. Since VWIFI_CTRL_RX_80211 the two paths differ less than the
 * split suggests -- both carry 802.11 MPDUs and both attach a
 * DOT11_EXTSTA_RECV_CONTEXT; what monitor mode adds is the raw-capture
 * flag on the indication. (This comment previously said the STA path
 * indicated 802.3 frames. It did not, and had not for some time -- see
 * the frame-format contract in vwifi_drv.h.)
 * ============================================================ */

VOID
VwifiRxDrainSta(_Inout_ PVWIFI_ADAPTER Adapter)
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
                       "rx(sta)", ring->NumDescs, Adapter->RxOutstanding);
            break;
        }

        if (d->flags & VWIFI_DESC_F_OWN) break;

        VWIFI_TAL_FIRST(8, "rx(sta): descriptor %u: %u bytes flags=0x%04x "
                           "freq=%u rssi=%d",
                        idx, d->frame_len, d->flags, d->channel_freq, d->rssi);

        /* RAW is what we asked for.
         *
         * This used to drop raw frames as something that could not
         * happen in STA mode, back when the device converted to 802.3
         * for us. VWIFI_CTRL_RX_80211 turns that conversion off,
         * because wdiwifi wants MPDUs on receive exactly as it hands
         * them down on transmit, so RAW is now the normal case and its
         * absence is the surprising one. */
        if (d->frame_len == 0) {
            goto rearm;
        }
        if (!(d->flags & VWIFI_RX_F_RAW)) {
            VWIFI_TAL_ONCE("rx(sta): device delivered an 802.3 frame despite "
                           "VWIFI_CTRL_RX_80211 -- passing it up anyway, but "
                           "the component expects an 802.11 MPDU and will "
                           "discard it in silence");
        }

        /* Stop one slot short of owning the whole ring. The frame is
         * dropped and its slot re-armed -- which is safe precisely
         * because no NBL has taken it yet -- so the device keeps a
         * place to write and the drain keeps making progress. Losing
         * frames while the returns catch up beats lapping. */
        if (Adapter->RxOutstanding >= (LONG)(ring->NumDescs - 1)) {
            VWIFI_TAL_ONCE("rx(sta): all but one RX slot outstanding -- "
                           "dropping frames until the component returns "
                           "some");
            goto rearm;
        }

        {
            PUCHAR frame_va = (PUCHAR)Adapter->RxBufferPoolVa
                            + (SIZE_T)idx * VWIFI_RX_BUFFER_SIZE;

            PMDL mdl = NdisAllocateMdl(Adapter->MiniportAdapterHandle,
                                       frame_va, d->frame_len);
            if (!mdl) {
                VWIFI_WARN("rx(sta): MDL alloc failed");
                goto rearm;
            }

            PNET_BUFFER_LIST nbl = NdisAllocateNetBufferAndNetBufferList(
                Adapter->RxNblPool, 0, 0,
                mdl, 0, d->frame_len);
            if (!nbl) {
                VWIFI_WARN("rx(sta): NBL alloc failed for %u bytes "
                           "(pool context %u bytes)",
                           d->frame_len, VWIFI_RX_NBL_CONTEXT_SIZE);
                NdisFreeMdl(mdl);
                goto rearm;
            }

            VwifiRxNblSetSlot(nbl, idx);

            /* Every frame is indicated on VWIFI_WDI_RX_TID_NON_QOS,
             * and no per-frame TID is kept.
             *
             * A QoS data frame (subtype bit 3, i.e. 0x80 of frame
             * control byte 0) carries a QoS control field at offset 24
             * whose low four bits would be its TID; nothing this
             * station receives is QoS, so the indication below and the
             * held-frame announcement in wdi_data.c both name the same
             * constant. Read the comment on it before changing the
             * value -- what the header documents bugchecks NDIS.
             *
             * The TID was briefly stored in MiniportReserved[3] so the
             * announcement could recover it. The component overwrites
             * that slot while it holds the frame. If QoS receive ever
             * becomes real, that is the moment to find somewhere safe
             * to keep a per-frame TID -- until then there is nothing
             * to keep. */

            /* No DOT11_EXTSTA_RECV_CONTEXT here, deliberately.
             *
             * This path briefly attached one, by analogy with monitor.c
             * and on the argument that both directions now carry 802.11
             * MPDUs so both should describe the reception the same way.
             * The argument was wrong about which interface it was
             * talking to.
             *
             * DOT11_EXTSTA_RECV_CONTEXT belongs to the NATIVE 802.11
             * receive path, where a miniport calls
             * NdisMIndicateReceiveNetBufferLists and the layer above
             * reads MediaSpecificInformation off the NBL. That is what
             * monitor.c does, and it is right to attach one there.
             *
             * A WDI station reception is not that path. The NBL is
             * handed to wdiwifi through the TAL -- RxInorderDataIndication
             * then RxGetMpdus -- and every scrap of per-frame metadata
             * WDI defines for receive lives in WDI_RX_METADATA, which
             * dot11wdi.h gives exactly one member: PayloadType. There is
             * no second channel. Writing a native-802.11 structure into
             * an OOB slot on an NBL that is about to travel a different
             * interface is not extra information, it is a pointer the
             * component did not put there.
             *
             * And it is not needed: this link carries traffic, WPA2
             * included, without one.
             *
             * The per-slot RxRecvContext array stays; monitor.c uses
             * it on the path where the structure does belong. */

            NET_BUFFER_LIST_STATUS(nbl) = NDIS_STATUS_SUCCESS;
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

            ring->NextIndex = (ring->NextIndex + 1) & ring->Mask;
            continue;
        }

    rearm:
        d->flags      = VWIFI_DESC_F_OWN;
        d->frame_len  = 0;
        d->buffer_len = VWIFI_RX_BUFFER_SIZE;
        ring->NextIndex = (ring->NextIndex + 1) & ring->Mask;
    }

    VwifiWrite32(Adapter, ring->RegHead, ring->NextIndex);

    if (!indicated) return;

    /* Through the TAL when there is a peer to attribute the frames to,
     * and only then.
     *
     * NdisMIndicateReceiveNetBufferLists is the plain-NDIS receive and
     * in WDI it is not the receive path for station data: wdiwifi's
     * RxMgr is, and it learns about a frame only through
     * NdisWdiRxInorderDataIndication. Indicating up the NDIS way on an
     * associated link put frames somewhere nothing was listening --
     * which is why an associated adapter still could not complete DHCP.
     *
     * The fallback is not dead code. Monitor mode has no peer and no
     * WDI port to receive on, and the frames still have to go
     * somewhere; VwifiRxDrainMonitor uses the same call for the same
     * reason. */
    {
        PVWIFI_PEER peer = VwifiPeerFirstActive(Adapter);

        if (peer != NULL) {
            while (indicate_head != NULL) {
                PNET_BUFFER_LIST nbl = indicate_head;

                indicate_head = NET_BUFFER_LIST_NEXT_NBL(nbl);
                NET_BUFFER_LIST_NEXT_NBL(nbl) = NULL;
                /* VWIFI_WDI_RX_TID_NON_QOS, which is 0 -- and 0 for a
                 * documented reason now rather than by accident. The
                 * held-frame announcement in wdi_data.c names the same
                 * constant, so the two can never disagree. */
                VwifiTalRxIndicate(Adapter, nbl, peer->PeerId,
                                   VWIFI_WDI_RX_TID_NON_QOS);
            }
            return;
        }
    }

    NdisMIndicateReceiveNetBufferLists(
        Adapter->MiniportAdapterHandle, indicate_head,
        NDIS_DEFAULT_PORT_NUMBER, indicated, 0);
}
