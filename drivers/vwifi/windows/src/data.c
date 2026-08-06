/*
 * vwifi — data.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Phase 3: STA-mode data path.
 *
 * The device does the 802.3 <-> 802.11 conversion (it owns the
 * association state, BSSID, and sequence counter), so this file is
 * deliberately thin:
 *
 *   TX: stage the 802.3 frame into a TX slot, publish the descriptor,
 *       ring the doorbell.
 *   RX: the device hands us 802.3 frames with VWIFI_RX_F_RAW clear;
 *       wrap each in an NBL and indicate it up.
 *
 * Monitor-mode RX (raw 802.11 + DOT11_EXTSTA_RECV_CONTEXT) lives in
 * monitor.c; the RX DPC routes to one or the other by op mode.
 */

#include "vwifi_drv.h"

/* ============================================================
 * TX — stage an 802.3 frame for the device
 * ============================================================ */

NDIS_STATUS
VwifiTxDataFrame(_Inout_ PVWIFI_ADAPTER Adapter,
                 _In_reads_bytes_(FrameLen) PUCHAR Frame,
                 _In_ ULONG FrameLen,
                 _In_ USHORT ExtraFlags)
{
    PVWIFI_RING ring = &Adapter->TxRing;
    ULONG slot;
    struct vwifi_tx_desc *desc;
    PUCHAR tx_buf;

    if (FrameLen < 14 || FrameLen > VWIFI_RX_BUFFER_SIZE) {
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
 * RX — indicate 802.3 frames from the device
 *
 * Mirrors VwifiRxDrainMonitor's NBL lifetime: the ring slot stays
 * owned by the NBL until VwifiMiniportReturnNetBufferLists reclaims
 * it. The only differences are that there's no
 * DOT11_EXTSTA_RECV_CONTEXT to attach and the payload is 802.3.
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

    /* An associated link that carries no traffic gives two very
     * different silences, and until now they looked identical in the
     * trace: the RX interrupt never firing at all, and it firing with
     * an empty ring every time. The first means nothing is arriving
     * from the medium; the second means frames are arriving and being
     * dropped somewhere between here and the component. */
    VWIFI_TAL_ONCE("rx(sta): the RX DPC has fired at least once");

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

        /* In STA mode the device delivers 802.3 with RAW clear. If we
         * somehow got a raw frame here, skip it. */
        if ((d->flags & VWIFI_RX_F_RAW) || d->frame_len == 0) {
            goto rearm;
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

            /* What is actually arriving, and in what shape.
             *
             * TX turned out to be the opposite of what this driver
             * assumed -- the component hands down 802.11 MPDUs, not
             * 802.3 -- and RX is the same contract read the other way,
             * so "the device converts to 802.3 and we pass it up" is an
             * assumption of exactly the kind that was wrong last time.
             * The dump settles it instead of arguing it.
             *
             * It also says what the frame IS. An associated link that
             * cannot complete DHCP has two very different explanations
             * -- the offer never arrives, or it arrives and the
             * component discards it -- and "UDP 67->68" in this line
             * tells them apart in one look. */
            if (d->frame_len >= 42) {
                VWIFI_TAL_FIRST(4,
                    "rx(sta): frame %u bytes: dst %02x:%02x:%02x:%02x:%02x:%02x "
                    "src %02x:%02x:%02x:%02x:%02x:%02x type %02x%02x -- %s",
                    d->frame_len,
                    frame_va[0], frame_va[1], frame_va[2],
                    frame_va[3], frame_va[4], frame_va[5],
                    frame_va[6], frame_va[7], frame_va[8],
                    frame_va[9], frame_va[10], frame_va[11],
                    frame_va[12], frame_va[13],
                    (frame_va[12] == 0x08 && frame_va[13] == 0x06)
                        ? "802.3 ARP"
                        : (frame_va[12] == 0x08 && frame_va[13] == 0x00)
                            ? ((frame_va[23] == 17) ? "802.3 IPv4/UDP"
                                                    : "802.3 IPv4")
                            : (frame_va[12] == 0x86 && frame_va[13] == 0xdd)
                                ? "802.3 IPv6"
                                : (((frame_va[0] >> 2) & 0x3) == 2)
                                    ? "NOT 802.3 -- this is an 802.11 DATA "
                                      "MPDU and is being passed up as an MSDU"
                                    : "802.3 with an unrecognised EtherType");
                /* Ports second, and only when they exist, rather than
                 * widening the line above for every frame. 67 -> 68 is
                 * the DHCP offer this link keeps not getting. */
                if (frame_va[12] == 0x08 && frame_va[13] == 0x00 &&
                    frame_va[23] == 17 && d->frame_len >= 38) {
                    VWIFI_TAL_FIRST(4, "rx(sta):   UDP %u -> %u",
                                    (ULONG)((frame_va[34] << 8) | frame_va[35]),
                                    (ULONG)((frame_va[36] << 8) | frame_va[37]));
                }
            }

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
                /* Best-effort TID. This device does no QoS and the
                 * component told us nothing per-frame, so every MSDU is
                 * best-effort -- which is what TID 0 means. */
                VwifiTalRxIndicate(Adapter, nbl, peer->PeerId, 0);
            }
            return;
        }
    }

    NdisMIndicateReceiveNetBufferLists(
        Adapter->MiniportAdapterHandle, indicate_head,
        NDIS_DEFAULT_PORT_NUMBER, indicated, 0);
}
