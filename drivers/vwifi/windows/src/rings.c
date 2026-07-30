/*
 * vwifi — rings.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Four descriptor rings, DMA-coherent allocation via NdisMAllocate
 * SharedMemory, the synchronous control-send primitive, and the
 * DPC-context drain functions for ctrl-rsp and rx rings.
 *
 * Ownership model (mirrors QEMU side):
 *   - Producer rings (ctrl-req, tx): driver writes desc with OWN=1,
 *     increments its NextIndex, writes to doorbell MMIO to notify.
 *     Device clears OWN when done.
 *   - Consumer rings (ctrl-rsp, rx): device writes desc with OWN=1,
 *     driver observes via ISR+DPC, processes, clears OWN,
 *     increments NextIndex, writes it to RING_HEAD MMIO to ack.
 */

#include "vwifi_drv.h"

/* ============================================================
 * DMA-coherent allocation helpers
 * ============================================================ */

static NDIS_STATUS
VwifiRingAlloc(
    _Inout_ PVWIFI_ADAPTER Adapter,
    _Inout_ PVWIFI_RING Ring,
    _In_ PCSTR Name,
    _In_ ULONG NumDescs,
    _In_ ULONG DescSize,
    _In_ ULONG RegBaseLo, _In_ ULONG RegBaseHi,
    _In_ ULONG RegSize,
    _In_ ULONG RegDoorbell, _In_ ULONG RegHead)
{
    ULONG size = NumDescs * DescSize;

    RtlZeroMemory(Ring, sizeof(*Ring));
    Ring->Name        = Name;
    Ring->NumDescs    = NumDescs;
    Ring->DescSize    = DescSize;
    Ring->Mask        = NumDescs - 1;
    Ring->SizeBytes   = size;
    Ring->RegBaseLo   = RegBaseLo;
    Ring->RegBaseHi   = RegBaseHi;
    Ring->RegSize     = RegSize;
    Ring->RegDoorbell = RegDoorbell;
    Ring->RegHead     = RegHead;
    Ring->NextIndex   = 0;

    NdisMAllocateSharedMemory(
        Adapter->MiniportAdapterHandle, size, TRUE /* cached = no */,
        &Ring->VirtualAddress, &Ring->PhysicalAddress);
    if (!Ring->VirtualAddress) {
        VWIFI_ERR("ring %s: shared memory alloc failed (%u bytes)",
                  Name, size);
        return NDIS_STATUS_RESOURCES;
    }
    RtlZeroMemory(Ring->VirtualAddress, size);

    VWIFI_INFO("ring %s: %u descs × %u B = %u B @ phys 0x%llx",
               Name, NumDescs, DescSize, size,
               Ring->PhysicalAddress.QuadPart);
    return NDIS_STATUS_SUCCESS;
}

static VOID
VwifiRingFreeOne(_Inout_ PVWIFI_ADAPTER Adapter, _Inout_ PVWIFI_RING Ring)
{
    if (Ring->VirtualAddress) {
        NdisMFreeSharedMemory(Adapter->MiniportAdapterHandle,
                              Ring->SizeBytes, TRUE,
                              Ring->VirtualAddress, Ring->PhysicalAddress);
        Ring->VirtualAddress = NULL;
    }
}

/* ============================================================
 * VwifiRingsAllocate — all four rings + buffer pools
 * ============================================================ */
NDIS_STATUS
VwifiRingsAllocate(_Inout_ PVWIFI_ADAPTER Adapter)
{
    NDIS_STATUS st;

    st = VwifiRingAlloc(Adapter, &Adapter->CtrlReqRing, "ctrl-req",
        VWIFI_CTRL_REQ_RING_SIZE, sizeof(struct vwifi_ctrl_req_desc),
        VWIFI_REG_CTRL_REQ_RING_BASE_LO, VWIFI_REG_CTRL_REQ_RING_BASE_HI,
        VWIFI_REG_CTRL_REQ_RING_SIZE, VWIFI_REG_CTRL_REQ_RING_DOORBELL, 0);
    if (st != NDIS_STATUS_SUCCESS) goto fail;

    st = VwifiRingAlloc(Adapter, &Adapter->CtrlRspRing, "ctrl-rsp",
        VWIFI_CTRL_RSP_RING_SIZE, sizeof(struct vwifi_ctrl_rsp_desc),
        VWIFI_REG_CTRL_RSP_RING_BASE_LO, VWIFI_REG_CTRL_RSP_RING_BASE_HI,
        VWIFI_REG_CTRL_RSP_RING_SIZE, 0, VWIFI_REG_CTRL_RSP_RING_HEAD);
    if (st != NDIS_STATUS_SUCCESS) goto fail;

    st = VwifiRingAlloc(Adapter, &Adapter->TxRing, "tx",
        VWIFI_TX_RING_SIZE, sizeof(struct vwifi_tx_desc),
        VWIFI_REG_TX_RING_BASE_LO, VWIFI_REG_TX_RING_BASE_HI,
        VWIFI_REG_TX_RING_SIZE, VWIFI_REG_TX_RING_DOORBELL, 0);
    if (st != NDIS_STATUS_SUCCESS) goto fail;

    st = VwifiRingAlloc(Adapter, &Adapter->RxRing, "rx",
        VWIFI_RX_RING_SIZE, sizeof(struct vwifi_rx_desc),
        VWIFI_REG_RX_RING_BASE_LO, VWIFI_REG_RX_RING_BASE_HI,
        VWIFI_REG_RX_RING_SIZE, 0, VWIFI_REG_RX_RING_HEAD);
    if (st != NDIS_STATUS_SUCCESS) goto fail;

    /* RX buffer pool: one contiguous DMA region, divided into
     * VWIFI_RX_RING_SIZE slots of VWIFI_RX_BUFFER_SIZE bytes each. */
    Adapter->RxBufferPoolSize = VWIFI_RX_RING_SIZE * VWIFI_RX_BUFFER_SIZE;
    NdisMAllocateSharedMemory(Adapter->MiniportAdapterHandle,
        Adapter->RxBufferPoolSize, FALSE,
        &Adapter->RxBufferPoolVa, &Adapter->RxBufferPoolPa);
    if (!Adapter->RxBufferPoolVa) {
        st = NDIS_STATUS_RESOURCES;
        goto fail;
    }

    /* TX injection buffer pool: same geometry as RX, used to stage
     * injected 802.11 frames before DMA. */
    Adapter->TxBufferPoolSize = VWIFI_TX_RING_SIZE * VWIFI_RX_BUFFER_SIZE;
    NdisMAllocateSharedMemory(Adapter->MiniportAdapterHandle,
        Adapter->TxBufferPoolSize, FALSE,
        &Adapter->TxBufferPoolVa, &Adapter->TxBufferPoolPa);
    if (!Adapter->TxBufferPoolVa) {
        st = NDIS_STATUS_RESOURCES;
        goto fail;
    }

    /* Per-slot DOT11_EXTSTA_RECV_CONTEXT array — non-DMA, just kernel
     * memory referenced by RX NBLs while in flight. */
    Adapter->RxRecvContext = NdisAllocateMemoryWithTagPriority(
        Adapter->MiniportAdapterHandle,
        VWIFI_RX_RING_SIZE * sizeof(struct DOT11_EXTSTA_RECV_CONTEXT),
        VWIFI_POOL_TAG, NormalPoolPriority);
    if (!Adapter->RxRecvContext) {
        st = NDIS_STATUS_RESOURCES;
        goto fail;
    }

    /* Ctrl-request payload scratch: one VWIFI_CTRL_PAYLOAD_SIZE slot
     * per ring entry, so concurrent requests don't collide. */
    NdisMAllocateSharedMemory(Adapter->MiniportAdapterHandle,
        VWIFI_CTRL_REQ_RING_SIZE * VWIFI_CTRL_PAYLOAD_SIZE, FALSE,
        &Adapter->CtrlReqPayloadVa, &Adapter->CtrlReqPayloadPa);
    NdisMAllocateSharedMemory(Adapter->MiniportAdapterHandle,
        VWIFI_CTRL_RSP_RING_SIZE * VWIFI_CTRL_PAYLOAD_SIZE, FALSE,
        &Adapter->CtrlRspPayloadVa, &Adapter->CtrlRspPayloadPa);
    if (!Adapter->CtrlReqPayloadVa || !Adapter->CtrlRspPayloadVa) {
        st = NDIS_STATUS_RESOURCES;
        goto fail;
    }

    return NDIS_STATUS_SUCCESS;

fail:
    VwifiRingsFree(Adapter);
    return st;
}

VOID
VwifiRingsFree(_Inout_ PVWIFI_ADAPTER Adapter)
{
    VwifiRingFreeOne(Adapter, &Adapter->CtrlReqRing);
    VwifiRingFreeOne(Adapter, &Adapter->CtrlRspRing);
    VwifiRingFreeOne(Adapter, &Adapter->TxRing);
    VwifiRingFreeOne(Adapter, &Adapter->RxRing);

    if (Adapter->RxBufferPoolVa) {
        NdisMFreeSharedMemory(Adapter->MiniportAdapterHandle,
            Adapter->RxBufferPoolSize, FALSE,
            Adapter->RxBufferPoolVa, Adapter->RxBufferPoolPa);
        Adapter->RxBufferPoolVa = NULL;
    }
    if (Adapter->TxBufferPoolVa) {
        NdisMFreeSharedMemory(Adapter->MiniportAdapterHandle,
            Adapter->TxBufferPoolSize, FALSE,
            Adapter->TxBufferPoolVa, Adapter->TxBufferPoolPa);
        Adapter->TxBufferPoolVa = NULL;
    }
    if (Adapter->RxRecvContext) {
        NdisFreeMemoryWithTagPriority(Adapter->MiniportAdapterHandle,
            Adapter->RxRecvContext, VWIFI_POOL_TAG);
        Adapter->RxRecvContext = NULL;
    }
    if (Adapter->CtrlReqPayloadVa) {
        NdisMFreeSharedMemory(Adapter->MiniportAdapterHandle,
            VWIFI_CTRL_REQ_RING_SIZE * VWIFI_CTRL_PAYLOAD_SIZE, FALSE,
            Adapter->CtrlReqPayloadVa, Adapter->CtrlReqPayloadPa);
        Adapter->CtrlReqPayloadVa = NULL;
    }
    if (Adapter->CtrlRspPayloadVa) {
        NdisMFreeSharedMemory(Adapter->MiniportAdapterHandle,
            VWIFI_CTRL_RSP_RING_SIZE * VWIFI_CTRL_PAYLOAD_SIZE, FALSE,
            Adapter->CtrlRspPayloadVa, Adapter->CtrlRspPayloadPa);
        Adapter->CtrlRspPayloadVa = NULL;
    }
}

/* ============================================================
 * VwifiRingsProgramMmio — write each ring's physical address and
 * size into the corresponding MMIO registers. Must be done BEFORE
 * writing VWIFI_CTRL_ENABLE.
 * ============================================================ */
VOID
VwifiRingsProgramMmio(_In_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_RING rings[] = {
        &Adapter->CtrlReqRing, &Adapter->CtrlRspRing,
        &Adapter->TxRing,      &Adapter->RxRing,
    };
    for (ULONG i = 0; i < ARRAYSIZE(rings); i++) {
        PVWIFI_RING r = rings[i];
        ULONGLONG pa = r->PhysicalAddress.QuadPart;
        VwifiWrite32(Adapter, r->RegBaseLo, (ULONG)(pa & 0xFFFFFFFF));
        VwifiWrite32(Adapter, r->RegBaseHi, (ULONG)(pa >> 32));
        VwifiWrite32(Adapter, r->RegSize,   r->NumDescs);
        r->NextIndex = 0;
    }
}

/* ============================================================
 * Arm every control-response descriptor.
 *
 * The device picks the next response slot with an index of its own,
 * advanced by asynchronous EVENTS as well as by replies to requests --
 * BSS_FOUND, SCAN_COMPLETE, ASSOC_RESULT and LINK_STATE all consume a
 * slot. It refuses to post into a slot whose OWN bit is clear, and
 * drops the event when that happens.
 *
 * So the whole ring has to be armed up front, exactly as the Linux
 * driver does it. Arming slots lazily, one per outgoing request, breaks
 * the moment an event is posted: the two indices drift apart, leaving
 * some slots unarmed when the device needs them and letting a preload
 * overwrite a slot the device has already filled. Both failures are
 * silent -- an event simply never arrives, and a scan reports fewer
 * networks or never completes.
 * ============================================================ */
VOID
VwifiRingsArmCtrlRsp(_Inout_ PVWIFI_ADAPTER Adapter)
{
    struct vwifi_ctrl_rsp_desc *descs = Adapter->CtrlRspRing.VirtualAddress;

    for (ULONG i = 0; i < Adapter->CtrlRspRing.NumDescs; i++) {
        struct vwifi_ctrl_rsp_desc *d = &descs[i];
        RtlZeroMemory(d, sizeof(*d));
        d->payload_addr = Adapter->CtrlRspPayloadPa.QuadPart
                        + (ULONGLONG)i * VWIFI_CTRL_PAYLOAD_SIZE;
        d->payload_len  = VWIFI_CTRL_PAYLOAD_SIZE;
        d->flags        = VWIFI_DESC_F_OWN;
    }
    Adapter->CtrlRspRing.NextIndex = 0;
}

/* ============================================================
 * Post initial RX buffers so the device has somewhere to deliver
 * frames when the medium starts flowing.
 * ============================================================ */
VOID
VwifiRingsPostRxBuffers(_Inout_ PVWIFI_ADAPTER Adapter)
{
    struct vwifi_rx_desc *descs = Adapter->RxRing.VirtualAddress;

    for (ULONG i = 0; i < Adapter->RxRing.NumDescs; i++) {
        struct vwifi_rx_desc *d = &descs[i];
        ULONGLONG buf_pa = Adapter->RxBufferPoolPa.QuadPart
                         + (ULONGLONG)i * VWIFI_RX_BUFFER_SIZE;
        RtlZeroMemory(d, sizeof(*d));
        d->frame_addr = buf_pa;
        d->buffer_len = VWIFI_RX_BUFFER_SIZE;
        d->flags      = VWIFI_DESC_F_OWN;   /* buffer is owned by device */
    }
}

/* ============================================================
 * Pending-request list helpers
 * ============================================================ */

static VOID
VwifiEnqueuePendingReq(_Inout_ PVWIFI_ADAPTER Adapter,
                      _Inout_ PVWIFI_PENDING_REQ Req)
{
    KIRQL old;
    KeAcquireSpinLock(&Adapter->PendingReqLock, &old);
    InsertTailList(&Adapter->PendingReqList, &Req->Link);
    KeReleaseSpinLock(&Adapter->PendingReqLock, old);
}

static PVWIFI_PENDING_REQ
VwifiDequeuePendingReqLocked(_Inout_ PVWIFI_ADAPTER Adapter,
                             _In_ ULONG ReqId)
{
    PLIST_ENTRY entry;
    for (entry = Adapter->PendingReqList.Flink;
         entry != &Adapter->PendingReqList;
         entry = entry->Flink) {
        PVWIFI_PENDING_REQ r = CONTAINING_RECORD(entry, VWIFI_PENDING_REQ, Link);
        if (r->ReqId == ReqId) {
            RemoveEntryList(&r->Link);
            return r;
        }
    }
    return NULL;
}

/* ============================================================
 * VwifiCtrlSendSync — post a control request and block until the
 * response arrives or the adapter halts. Called only at PASSIVE
 * IRQL (from OID handlers or MiniportInitializeEx).
 * ============================================================ */
NDIS_STATUS
VwifiCtrlSendSync(
    _Inout_ PVWIFI_ADAPTER Adapter,
    _In_ USHORT Opcode,
    _In_reads_bytes_opt_(InLen) const VOID *InPayload,
    _In_ ULONG InLen,
    _Out_writes_bytes_to_opt_(*OutLen, *OutLen) VOID *OutPayload,
    _Inout_ ULONG *OutLen)
{
    ULONG slot;
    struct vwifi_ctrl_req_desc *desc;
    VWIFI_PENDING_REQ req = { 0 };
    LARGE_INTEGER timeout;

    if (InLen > VWIFI_CTRL_PAYLOAD_SIZE) {
        return NDIS_STATUS_INVALID_LENGTH;
    }

    /* Allocate a request id and a pending record. */
    req.ReqId  = (ULONG)InterlockedIncrement(&Adapter->NextReqId);
    req.Opcode = Opcode;
    KeInitializeEvent(&req.CompletionEvent, NotificationEvent, FALSE);
    if (OutPayload && OutLen && *OutLen) {
        req.ReplyPayload    = OutPayload;
        req.ReplyPayloadLen = *OutLen;
    }
    VwifiEnqueuePendingReq(Adapter, &req);

    /* Pick the next free request slot (simple round-robin; relies on
     * ring depth > outstanding request count — true in practice). */
    slot = InterlockedIncrement((volatile LONG *)&Adapter->CtrlReqRing.NextIndex) - 1;
    slot &= Adapter->CtrlReqRing.Mask;

    desc = (struct vwifi_ctrl_req_desc *)
        ((PUCHAR)Adapter->CtrlReqRing.VirtualAddress +
         slot * sizeof(struct vwifi_ctrl_req_desc));

    /* Copy input payload into the per-slot scratch buffer. */
    if (InPayload && InLen) {
        RtlCopyMemory(
            (PUCHAR)Adapter->CtrlReqPayloadVa + slot * VWIFI_CTRL_PAYLOAD_SIZE,
            InPayload, InLen);
    }

    /*
     * No response slot is preloaded here on purpose.
     *
     * The response ring is armed in full by VwifiRingsArmCtrlRsp() and
     * re-armed slot by slot as the DPC drains it. Arming one here, at
     * the index of the REQUEST slot, assumed the device answers on the
     * matching index -- it does not. It uses a producer index of its
     * own that also advances for every asynchronous event, so the two
     * drift apart as soon as a single BSS_FOUND is posted, and this
     * store would then land on a slot the device had already filled and
     * the DPC had not yet read.
     */

    /* Build and publish the request descriptor. */
    RtlZeroMemory(desc, sizeof(*desc));
    desc->opcode       = Opcode;
    desc->req_id       = req.ReqId;
    desc->payload_addr = Adapter->CtrlReqPayloadPa.QuadPart
                       + slot * VWIFI_CTRL_PAYLOAD_SIZE;
    desc->payload_len  = InLen;
    desc->flags        = VWIFI_DESC_F_OWN;

    /* Doorbell. */
    VwifiWrite32(Adapter, VWIFI_REG_CTRL_REQ_RING_DOORBELL, slot + 1);

    /* Wait up to 5 seconds for the DPC to signal us. */
    timeout.QuadPart = -(LONGLONG)(5LL * 10LL * 1000LL * 1000LL);
    NTSTATUS nt = KeWaitForSingleObject(&req.CompletionEvent, Executive,
                                        KernelMode, FALSE, &timeout);
    if (nt == STATUS_TIMEOUT) {
        /* Pull from pending list to avoid double-completion. */
        KIRQL old;
        KeAcquireSpinLock(&Adapter->PendingReqLock, &old);
        (void)VwifiDequeuePendingReqLocked(Adapter, req.ReqId);
        KeReleaseSpinLock(&Adapter->PendingReqLock, old);
        VWIFI_WARN("ctrl send opcode 0x%04x timed out", Opcode);
        return NDIS_STATUS_REQUEST_ABORTED;
    }

    if (OutLen) *OutLen = req.ReplyPayloadLen;
    return req.Status;
}

/* ============================================================
 * VwifiCtrlRspDrain — DPC-context: consume every descriptor on
 * the ctrl-rsp ring, match to pending request (by req_id), or
 * dispatch as an event.
 * ============================================================ */
VOID
VwifiCtrlRspDrain(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_RING ring = &Adapter->CtrlRspRing;

    for (;;) {
        ULONG idx = ring->NextIndex & ring->Mask;
        struct vwifi_ctrl_rsp_desc *d = (struct vwifi_ctrl_rsp_desc *)
            ((PUCHAR)ring->VirtualAddress + idx * ring->DescSize);

        /* Device transfers OWN to driver by CLEARING the bit;
         * see vwifi_post_rsp on the QEMU side. */
        if (d->flags & VWIFI_DESC_F_OWN) {
            break;   /* still owned by device */
        }

        if (d->flags & VWIFI_RSP_F_EVENT) {
            /* Unsolicited device event. The payload lives in the
             * shared-memory slot this descriptor points at. */
            const VOID *payload = (PUCHAR)Adapter->CtrlRspPayloadVa
                                + idx * VWIFI_CTRL_PAYLOAD_SIZE;

            switch (d->opcode) {
            case VWIFI_EV_BSS_FOUND:
                VwifiScanOnBssFound(Adapter, payload, d->payload_len);
                break;

            case VWIFI_EV_SCAN_COMPLETE:
                VwifiScanOnComplete(Adapter, payload, d->payload_len);
                break;

            case VWIFI_EV_LINK_STATE:
                if (d->payload_len >= sizeof(ULONG)) {
                    Adapter->MediumLinkUp = (*(const ULONG *)payload) != 0;
                    VWIFI_INFO("medium link %s",
                               Adapter->MediumLinkUp ? "up" : "down");
                }
                break;

            case VWIFI_EV_ASSOC_RESULT:
                VwifiConnectOnAssocResult(Adapter, payload, d->payload_len);
                break;

            case VWIFI_EV_DISCONNECTED:
                VwifiConnectOnDisconnected(Adapter, payload, d->payload_len);
                break;

            case VWIFI_EV_KEY_INSTALLED:
                VwifiKeysOnInstalled(Adapter, payload, d->payload_len);
                break;

            /* Phase 5+ events. */
            case VWIFI_EV_MGMT_RX:
            default:
                VWIFI_INFO("unhandled event 0x%04x status=%d payload=%u",
                           d->opcode, d->status, d->payload_len);
                break;
            }
        } else {
            /* Match to pending request by req_id. Already at
             * DISPATCH_LEVEL in the DPC, hence the *AtDpcLevel forms
             * and no saved KIRQL. */
            PVWIFI_PENDING_REQ req;
            KeAcquireSpinLockAtDpcLevel(&Adapter->PendingReqLock);
            req = VwifiDequeuePendingReqLocked(Adapter, d->req_id);
            KeReleaseSpinLockFromDpcLevel(&Adapter->PendingReqLock);

            if (req) {
                req->Status = (d->status == 0)
                    ? NDIS_STATUS_SUCCESS
                    : NDIS_STATUS_FAILURE;

                if (req->ReplyPayload && d->payload_len) {
                    /* Copy reply from shared-mem into caller's buf. */
                    ULONG n = d->payload_len;
                    if (n > req->ReplyPayloadLen) n = req->ReplyPayloadLen;
                    RtlCopyMemory(
                        req->ReplyPayload,
                        (PUCHAR)Adapter->CtrlRspPayloadVa
                          + idx * VWIFI_CTRL_PAYLOAD_SIZE,
                        n);
                    req->ReplyPayloadLen = n;
                }
                KeSetEvent(&req->CompletionEvent, 0, FALSE);
            } else {
                VWIFI_WARN("orphan ctrl-rsp req_id=%u opcode=0x%04x",
                           d->req_id, d->opcode);
            }
        }

        /* Re-arm the slot for next use: point it at the same payload
         * buffer and transfer ownership back to device. */
        d->payload_addr = Adapter->CtrlRspPayloadPa.QuadPart
                        + idx * VWIFI_CTRL_PAYLOAD_SIZE;
        d->payload_len  = VWIFI_CTRL_PAYLOAD_SIZE;
        d->flags        = VWIFI_DESC_F_OWN;

        ring->NextIndex = (ring->NextIndex + 1) & ring->Mask;
    }

    /* Ack the vector by writing head back. */
    VwifiWrite32(Adapter, ring->RegHead, ring->NextIndex);
}

/* ============================================================
 * VwifiRxDrain — DPC-context: consume RX descriptors. Phase 1:
 * we don't yet have an NBL pool wired up, so we just re-arm the
 * slots. Phase 1.5 will convert this into proper NBL indications
 * with DOT11_EXTSTA_RECV_CONTEXT attached.
 * ============================================================ */
VOID
VwifiRxDrain(_Inout_ PVWIFI_ADAPTER Adapter)
{
    PVWIFI_RING ring = &Adapter->RxRing;

    for (;;) {
        ULONG idx = ring->NextIndex & ring->Mask;
        struct vwifi_rx_desc *d = (struct vwifi_rx_desc *)
            ((PUCHAR)ring->VirtualAddress + idx * ring->DescSize);

        if (d->flags & VWIFI_DESC_F_OWN) {
            break;
        }

        VWIFI_INFO("rx: %u bytes freq=%u MHz rssi=%d rate=0x%02x flags=0x%04x",
                   d->frame_len, d->channel_freq, d->rssi,
                   d->rate_code, d->flags);

        /* Re-arm the slot with the same buffer. */
        d->flags      = VWIFI_DESC_F_OWN;
        d->frame_len  = 0;
        d->buffer_len = VWIFI_RX_BUFFER_SIZE;

        ring->NextIndex = (ring->NextIndex + 1) & ring->Mask;
    }

    VwifiWrite32(Adapter, ring->RegHead, ring->NextIndex);
}
