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
                 _In_ ULONG FrameLen)
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
    /* No INJECT flag: in STA mode the device treats this as 802.3 and
     * builds the 802.11 data frame itself. */
    desc->flags      = VWIFI_DESC_F_OWN;
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

    for (;;) {
        ULONG idx = ring->NextIndex & ring->Mask;
        struct vwifi_rx_desc *d = (struct vwifi_rx_desc *)
            ((PUCHAR)ring->VirtualAddress + idx * ring->DescSize);

        if (d->flags & VWIFI_DESC_F_OWN) break;

        /* In STA mode the device delivers 802.3 with RAW clear. If we
         * somehow got a raw frame here, skip it. */
        if ((d->flags & VWIFI_RX_F_RAW) || d->frame_len == 0) {
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
                Adapter->RxNblPool, sizeof(VWIFI_RX_NBL_CONTEXT), 0,
                mdl, 0, d->frame_len);
            if (!nbl) {
                VWIFI_WARN("rx(sta): NBL alloc failed");
                NdisFreeMdl(mdl);
                goto rearm;
            }

            PVWIFI_RX_NBL_CONTEXT ctx =
                (PVWIFI_RX_NBL_CONTEXT)NET_BUFFER_LIST_CONTEXT_DATA_START(nbl);
            ctx->SlotIndex = idx;

            NET_BUFFER_LIST_STATUS(nbl) = NDIS_STATUS_SUCCESS;
            NET_BUFFER_LIST_NEXT_NBL(nbl) = NULL;
            if (indicate_tail) {
                NET_BUFFER_LIST_NEXT_NBL(indicate_tail) = nbl;
            } else {
                indicate_head = nbl;
            }
            indicate_tail = nbl;
            indicated++;

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
