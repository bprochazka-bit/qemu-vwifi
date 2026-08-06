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

            /* What is actually arriving, and in what shape.
             *
             * An 802.11 MPDU now, not 802.3: addr1 at 4, addr2 at 10,
             * addr3 at 16, then LLC/SNAP and the EtherType. pktmon
             * settled the contract -- it showed wdiwifi converting
             * 802.3 to 802.11 on the way down and showed not one
             * received packet reaching NDIS while this driver was
             * handing up Ethernet.
             *
             * The offsets below are computed rather than written out
             * because a QoS data frame carries two more header bytes
             * and getting that wrong silently shifts every field. */
            if (d->frame_len >= 34) {
                ULONG hdr = 24;
                ULONG snap, etype;

                if ((frame_va[0] >> 4) & 0x08) hdr += 2;   /* QoS control */
                snap  = hdr + 8;                            /* LLC/SNAP */
                etype = hdr + 6;

                VWIFI_TAL_FIRST(4,
                    "rx(sta): frame %u bytes: a1 %02x:%02x:%02x:%02x:%02x:%02x "
                    "a2 %02x:%02x:%02x:%02x:%02x:%02x "
                    "a3 %02x:%02x:%02x:%02x:%02x:%02x fc %02x%02x type %02x%02x",
                    d->frame_len,
                    frame_va[4], frame_va[5], frame_va[6],
                    frame_va[7], frame_va[8], frame_va[9],
                    frame_va[10], frame_va[11], frame_va[12],
                    frame_va[13], frame_va[14], frame_va[15],
                    frame_va[16], frame_va[17], frame_va[18],
                    frame_va[19], frame_va[20], frame_va[21],
                    frame_va[0], frame_va[1],
                    frame_va[etype], frame_va[etype + 1]);

                /* Ports second, and only when they exist. 67 -> 68 is
                 * the DHCP offer this link keeps not getting. */
                if (frame_va[etype] == 0x08 && frame_va[etype + 1] == 0x00 &&
                    d->frame_len >= snap + 28 &&
                    frame_va[snap + 9] == 17) {
                    ULONG udp   = snap + 20;
                    ULONG sport = (ULONG)((frame_va[udp] << 8) | frame_va[udp + 1]);
                    ULONG dport = (ULONG)((frame_va[udp + 2] << 8) | frame_va[udp + 3]);

                    VWIFI_TAL_FIRST(4, "rx(sta):   UDP %u -> %u", sport, dport);

                    /* The offer, in full, on its own counter: is it
                     * addressed to this station, does its transaction id
                     * match what this station asked, and is the hardware
                     * address in the payload ours. Any one of those being
                     * wrong is a frame Windows is right to ignore, and
                     * none of them show up in "UDP 67 -> 68". */
                    if (sport == 67 && dport == 68 &&
                        d->frame_len >= udp + 8 + 44) {
                        ULONG b = udp + 8;   /* BOOTP */

                        VWIFI_TAL_FIRST(3,
                            "rx(sta):   DHCP op %u xid %02x%02x%02x%02x "
                            "flags %02x%02x yiaddr %u.%u.%u.%u "
                            "chaddr %02x:%02x:%02x:%02x:%02x:%02x",
                            frame_va[b],
                            frame_va[b + 4], frame_va[b + 5],
                            frame_va[b + 6], frame_va[b + 7],
                            frame_va[b + 10], frame_va[b + 11],
                            frame_va[b + 16], frame_va[b + 17],
                            frame_va[b + 18], frame_va[b + 19],
                            frame_va[b + 28], frame_va[b + 29], frame_va[b + 30],
                            frame_va[b + 31], frame_va[b + 32], frame_va[b + 33]);
                    }
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
