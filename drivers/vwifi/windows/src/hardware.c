/*
 * vwifi — hardware.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * PCI resource parsing, MMIO mapping, MSI-X interrupt setup,
 * device reset, and top-level enable/disable sequence.
 */

#include "vwifi_drv.h"

MINIPORT_MESSAGE_INTERRUPT    VwifiMessageIsr;
MINIPORT_MESSAGE_INTERRUPT_DPC VwifiMessageDpc;

/* ============================================================
 * Parse the assigned PCI resources list to find BAR0 (MMIO).
 * ============================================================ */
static NDIS_STATUS
VwifiParseResources(
    _Inout_ PVWIFI_ADAPTER Adapter,
    _In_ PCM_PARTIAL_RESOURCE_LIST List)
{
    ULONG i;
    BOOLEAN found_mmio = FALSE;

    for (i = 0; i < List->Count; i++) {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR d = &List->PartialDescriptors[i];
        switch (d->Type) {
        case CmResourceTypeMemory:
            if (!found_mmio) {
                Adapter->MmioPhysicalAddress = d->u.Memory.Start;
                Adapter->MmioLength          = d->u.Memory.Length;
                found_mmio = TRUE;
                VWIFI_INFO("BAR0 mmio phys=0x%llx len=%u",
                           Adapter->MmioPhysicalAddress.QuadPart,
                           Adapter->MmioLength);
            }
            break;
        default:
            /* MSI-X table/PBA, interrupt messages — NDIS handles
             * these via NdisMRegisterInterruptEx below. */
            break;
        }
    }

    if (!found_mmio || Adapter->MmioLength < VWIFI_MMIO_SIZE) {
        VWIFI_ERR("BAR0 missing or too small");
        return NDIS_STATUS_RESOURCE_CONFLICT;
    }
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * MSI-X ISR / DPC
 *
 * The ISR runs at DEVICE IRQL; it should be short. We check the
 * per-vector status bit and schedule the DPC, which will drain
 * ctrl-rsp and rx rings.
 * ============================================================ */
_Use_decl_annotations_
BOOLEAN
VwifiMessageIsr(
    NDIS_HANDLE MiniportInterruptContext,
    ULONG MessageId,
    PBOOLEAN QueueDefaultInterruptDpc,
    PULONG TargetProcessors)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportInterruptContext;
    ULONG irq_status;

    UNREFERENCED_PARAMETER(TargetProcessors);

    if (MessageId >= VWIFI_NUM_VECTORS) {
        *QueueDefaultInterruptDpc = FALSE;
        return FALSE;
    }

    /* Read & check which vectors are pending. */
    irq_status = VwifiRead32(adapter, VWIFI_REG_IRQ_STATUS);
    if (!(irq_status & (1u << MessageId))) {
        *QueueDefaultInterruptDpc = FALSE;
        return FALSE;
    }

    /* Mask this vector until the DPC has serviced it — avoids
     * IRQ storms while the DPC is still running. */
    ULONG mask = VwifiRead32(adapter, VWIFI_REG_IRQ_MASK);
    mask |= (1u << MessageId);
    VwifiWrite32(adapter, VWIFI_REG_IRQ_MASK, mask);

    *QueueDefaultInterruptDpc = TRUE;
    return TRUE;
}

_Use_decl_annotations_
VOID
VwifiMessageDpc(
    NDIS_HANDLE MiniportInterruptContext,
    ULONG MessageId,
    PVOID MiniportDpcContext,
    PULONG NdisReserved1,
    PULONG NdisReserved2)
{
    PVWIFI_ADAPTER adapter = (PVWIFI_ADAPTER)MiniportInterruptContext;
    ULONG mask;

    UNREFERENCED_PARAMETER(MiniportDpcContext);
    UNREFERENCED_PARAMETER(NdisReserved1);
    UNREFERENCED_PARAMETER(NdisReserved2);

    switch (MessageId) {
    case VWIFI_VEC_CTRL_RSP:
        VwifiCtrlRspDrain(adapter);
        break;
    case VWIFI_VEC_RX:
        if (adapter->OpMode == VWIFI_MODE_MONITOR) {
            VwifiRxDrainMonitor(adapter);
        } else if (adapter->OpMode == VWIFI_MODE_STA) {
            VwifiRxDrainSta(adapter);
        } else {
            VwifiRxDrain(adapter);
        }
        break;
    case VWIFI_VEC_TX_COMPLETE:
        /* Phase 1: no separate TX completion processing; the TX
         * ring's OWN bit is the only signal and we reclaim lazily. */
        break;
    case VWIFI_VEC_EVENT:
        /* Link-state changes etc. come through ctrl-rsp as EV_*
         * events today; this vector is reserved. */
        break;
    default:
        break;
    }

    /* Unmask the vector we just serviced. */
    mask = VwifiRead32(adapter, VWIFI_REG_IRQ_MASK);
    mask &= ~(1u << MessageId);
    VwifiWrite32(adapter, VWIFI_REG_IRQ_MASK, mask);
}

/* ============================================================
 * Bring-up is in two halves, and the split is not cosmetic.
 *
 * VwifiHwInitialize runs inside MiniportWdiAllocateAdapter. At that
 * point the adapter is NOT yet a registered NDIS miniport: we are
 * filling in the registration attributes that the WLAN component will
 * apply *after* we return. So nothing here may call an NDIS routine
 * that needs a registered adapter — above all NdisMAllocateSharedMemory,
 * which needs NDIS_MINIPORT_ATTRIBUTES_BUS_MASTER to be in effect and
 * otherwise fails with NDIS_STATUS_RESOURCES no matter how small the
 * request. A 1536-byte ring allocation failing on an idle machine is
 * what that looks like from the outside.
 *
 * VwifiHwStart runs from MiniportWdiOpenAdapter, by which time the
 * attributes are live. Everything that touches NDIS-managed
 * resources — DMA rings, NBL pools, interrupts — belongs there.
 * ============================================================ */

NDIS_STATUS
VwifiHwInitialize(
    _Inout_ PVWIFI_ADAPTER Adapter,
    _In_ PNDIS_MINIPORT_INIT_PARAMETERS InitParams)
{
    NDIS_STATUS status;
    ULONG caps, sig, ver;

    status = VwifiParseResources(Adapter,
                                 InitParams->AllocatedResources);
    if (status != NDIS_STATUS_SUCCESS) return status;

    /* Map BAR0 — MmNonCached because it's MMIO. MmMapIoSpace is a
     * memory-manager call, not an NDIS one, so it is safe this early. */
    Adapter->MmioVirtualAddress = MmMapIoSpace(
        Adapter->MmioPhysicalAddress, Adapter->MmioLength, MmNonCached);
    if (!Adapter->MmioVirtualAddress) {
        VWIFI_ERR("MmMapIoSpace BAR0 failed");
        return NDIS_STATUS_RESOURCES;
    }

    /* Verify signature and ABI version. Doing this here means a wrong
     * or mismatched device is rejected before the WLAN component has
     * committed to us. */
    sig = VwifiRead32(Adapter, VWIFI_REG_SIGNATURE);
    ver = VwifiRead32(Adapter, VWIFI_REG_ABI_VERSION);
    caps = VwifiRead32(Adapter, VWIFI_REG_CAPS);
    VWIFI_INFO("device signature=0x%08x abi_version=%u caps=0x%08x",
               sig, ver, caps);
    if (sig != VWIFI_SIGNATURE) {
        VWIFI_ERR("signature mismatch — wrong device?");
        status = NDIS_STATUS_NOT_SUPPORTED;
        goto fail_mmio;
    }
    if (ver != VWIFI_ABI_VERSION) {
        VWIFI_ERR("ABI version mismatch: device=%u driver=%u",
                  ver, VWIFI_ABI_VERSION);
        status = NDIS_STATUS_NOT_SUPPORTED;
        goto fail_mmio;
    }

    /* Assert reset to start from a known state. */
    VwifiWrite32(Adapter, VWIFI_REG_RESET, 1);

    VWIFI_INFO("hardware probed; rings deferred to OpenAdapter");
    return NDIS_STATUS_SUCCESS;

fail_mmio:
    MmUnmapIoSpace(Adapter->MmioVirtualAddress, Adapter->MmioLength);
    Adapter->MmioVirtualAddress = NULL;
    return status;
}

/* ============================================================
 * VwifiHwStart — called from MiniportWdiOpenAdapter, once the
 * registration attributes are in effect.
 * ============================================================ */
NDIS_STATUS
VwifiHwStart(_Inout_ PVWIFI_ADAPTER Adapter)
{
    NDIS_STATUS status;
    NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS irq_chars = { 0 };

    if (Adapter->Started) {
        return NDIS_STATUS_SUCCESS;
    }

    /* Allocate and program the four rings. */
    status = VwifiRingsAllocate(Adapter);
    if (status != NDIS_STATUS_SUCCESS) return status;
    VwifiRingsProgramMmio(Adapter);
    VwifiRingsArmCtrlRsp(Adapter);
    VwifiRingsPostRxBuffers(Adapter);

    /* RX NBL pool for monitor-mode indications. */
    status = VwifiRxNblPoolCreate(Adapter);
    if (status != NDIS_STATUS_SUCCESS) goto fail_rings;

    /* Phase 2 scan task state. */
    status = VwifiScanTaskCreate(Adapter);
    if (status != NDIS_STATUS_SUCCESS) goto fail_nbl;

    /* Phase 3 connect task state. */
    status = VwifiConnectTaskCreate(Adapter);
    if (status != NDIS_STATUS_SUCCESS) goto fail_scan;

    /* Connect MSI-X interrupts. NDIS walks the resource list to find
     * the message table and wires up the callbacks. */
    irq_chars.Header.Type     = NDIS_OBJECT_TYPE_MINIPORT_INTERRUPT;
    irq_chars.Header.Revision = NDIS_MINIPORT_INTERRUPT_REVISION_1;
    irq_chars.Header.Size     = NDIS_SIZEOF_MINIPORT_INTERRUPT_CHARACTERISTICS_REVISION_1;
    irq_chars.MessageInterruptHandler   = VwifiMessageIsr;
    irq_chars.MessageInterruptDpcHandler = VwifiMessageDpc;

    status = NdisMRegisterInterruptEx(
        Adapter->MiniportAdapterHandle, Adapter, &irq_chars,
        &Adapter->InterruptHandle);
    if (status != NDIS_STATUS_SUCCESS) {
        VWIFI_ERR("NdisMRegisterInterruptEx failed 0x%x", status);
        goto fail_connect;
    }
    Adapter->MessageInfo = irq_chars.MessageInfoTable;

    /* Enable device: IRQs + ring processing. */
    VwifiWrite32(Adapter, VWIFI_REG_IRQ_MASK, 0);
    VwifiWrite32(Adapter, VWIFI_REG_CTRL,
                 VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);

    /* GET_CAPS synchronously. Gives us the default MAC and feature
     * bits. This is the first round trip over the rings, so it is also
     * the first proof the device is answering at all. */
    {
        ULONG out_len = sizeof(Adapter->Caps);
        status = VwifiCtrlSendSync(Adapter, VWIFI_OP_GET_CAPS,
                                   NULL, 0,
                                   &Adapter->Caps, &out_len);
        if (status != NDIS_STATUS_SUCCESS) {
            VWIFI_ERR("GET_CAPS failed 0x%x — is the medium hub running?",
                      status);
            goto fail_irq;
        }
        Adapter->CapsValid = TRUE;
        RtlCopyMemory(Adapter->PermanentMac, Adapter->Caps.default_mac, 6);
        RtlCopyMemory(Adapter->CurrentMac,   Adapter->Caps.default_mac, 6);
        VWIFI_INFO("device caps=0x%08x mac=%02x:%02x:%02x:%02x:%02x:%02x",
                   Adapter->Caps.caps,
                   Adapter->CurrentMac[0], Adapter->CurrentMac[1],
                   Adapter->CurrentMac[2], Adapter->CurrentMac[3],
                   Adapter->CurrentMac[4], Adapter->CurrentMac[5]);
    }

    /* Push our MAC to the device so it stamps TX frames correctly. */
    {
        ULONG out_len = 0;
        (void)VwifiCtrlSendSync(Adapter, VWIFI_OP_SET_STA_MAC,
                                Adapter->CurrentMac, 6, NULL, &out_len);
    }

    Adapter->Started = TRUE;
    VWIFI_INFO("adapter started");
    return NDIS_STATUS_SUCCESS;

fail_irq:
    NdisMDeregisterInterruptEx(Adapter->InterruptHandle);
    Adapter->InterruptHandle = NULL;
fail_connect:
    VwifiConnectTaskDestroy(Adapter);
fail_scan:
    VwifiScanTaskDestroy(Adapter);
fail_nbl:
    VwifiRxNblPoolDestroy(Adapter);
fail_rings:
    VwifiRingsFree(Adapter);
    return status;
}

/* ============================================================
 * VwifiHwStop — the mirror of VwifiHwStart, from CloseAdapter.
 * Leaves the MMIO mapping alone; that belongs to Initialize.
 * ============================================================ */
VOID
VwifiHwStop(_Inout_ PVWIFI_ADAPTER Adapter)
{
    if (!Adapter->Started) {
        return;
    }
    Adapter->Started = FALSE;

    if (Adapter->MmioVirtualAddress) {
        VwifiWrite32(Adapter, VWIFI_REG_CTRL, 0);
        VwifiWrite32(Adapter, VWIFI_REG_RESET, 1);
    }

    if (Adapter->InterruptHandle) {
        NdisMDeregisterInterruptEx(Adapter->InterruptHandle);
        Adapter->InterruptHandle = NULL;
    }

    VwifiConnectTaskDestroy(Adapter);
    VwifiScanTaskDestroy(Adapter);
    VwifiRxNblPoolDestroy(Adapter);
    VwifiRingsFree(Adapter);
}

/* ============================================================
 * VwifiHwShutdown
 * ============================================================ */
VOID
VwifiHwShutdown(_Inout_ PVWIFI_ADAPTER Adapter)
{
    /* Safe whether or not OpenAdapter ever ran: HwStop is a no-op when
     * the adapter was never started, which is exactly the path taken
     * when AllocateAdapter succeeded but the start failed. */
    VwifiHwStop(Adapter);

    if (Adapter->MmioVirtualAddress) {
        MmUnmapIoSpace(Adapter->MmioVirtualAddress, Adapter->MmioLength);
        Adapter->MmioVirtualAddress = NULL;
    }
}

/* ============================================================
 * VwifiHwReset
 * ============================================================ */
NDIS_STATUS
VwifiHwReset(_Inout_ PVWIFI_ADAPTER Adapter)
{
    VWIFI_INFO("HwReset");
    VwifiWrite32(Adapter, VWIFI_REG_RESET, 1);
    /* Re-enable. Rings' base addresses are still valid — the
     * device zeros only the status bits, not the ring MMIO regs. */
    VwifiRingsProgramMmio(Adapter);
    VwifiRingsArmCtrlRsp(Adapter);
    VwifiRingsPostRxBuffers(Adapter);
    VwifiWrite32(Adapter, VWIFI_REG_CTRL,
                 VWIFI_CTRL_ENABLE | VWIFI_CTRL_IRQ_ENABLE);
    return NDIS_STATUS_SUCCESS;
}
