/*
 * vwifi — tlv_shim.h
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * C-callable boundary over the WDI TLV generator/parser library.
 *
 * Why a shim rather than calling the library directly from the WDI
 * handlers: the library ships as a C++ header (TlvGeneratorParser.hpp)
 * and requires C++ new/delete. Rather than convert every wdi_*.c to
 * C++ (which means auditing every implicit void* conversion), we keep
 * the C++ contained in tlv_shim.cpp and expose plain C entry points.
 *
 * The shim's job is translation in both directions:
 *
 *   WDI TLV byte stream  <->  our vwifi_* device ABI structures
 *
 * That happens to be exactly the boundary we want anyway: WDI's model
 * on one side, the device contract on the other, nothing leaking
 * across.
 *
 * PeerVersion: every entry point takes it. The caller supplies
 * Adapter->WdiPeerVersion, captured at init. The library uses it to
 * emit/consume a byte stream matching whatever WDI version the running
 * OS has — that's what lets one binary serve Windows 10 (WDI 1.1.9)
 * and Windows 11.
 */

#pragma once

#include <ndis.h>
#include "vwifi_abi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Scan
 * ============================================================ */

/* Parse an OID_WDI_TASK_SCAN M1 into a device scan request.
 *
 * `ReqBuf` must be at least sizeof(struct vwifi_scan_req) + room for
 * the trailing SSID list (num_ssids * 34 bytes). *ReqLen receives the
 * total bytes written.
 */
NDIS_STATUS VwifiTlvParseScanRequest(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_writes_bytes_to_(ReqCap, *ReqLen) struct vwifi_scan_req *ReqBuf,
    _In_ ULONG ReqCap,
    _Out_ PULONG ReqLen);

/* One BSS to report. Points into the device's event payload; the shim
 * copies what it needs during generation.
 *
 * `Frame` is the WHOLE beacon or probe-response frame as seen on the
 * air, NOT just the IE tail. WABIModel's BSSEntryContainer carries
 * WDI_TLV_BEACON_FRAME / WDI_TLV_PROBE_RESPONSE_FRAME as raw byte
 * blobs and the OS parses the IEs itself. Entry->ie_len is the frame
 * length; Entry->capability_info bit 16 (VWIFI_BSS_F_BEACON) says
 * which of the two TLVs to emit. */
typedef struct _VWIFI_TLV_BSS_ITEM
{
    const struct vwifi_bss_entry *Entry;
    const UCHAR                  *Frame;    /* Entry->ie_len bytes */
} VWIFI_TLV_BSS_ITEM, *PVWIFI_TLV_BSS_ITEM;

/* Generate an NDIS_STATUS_WDI_INDICATION_BSS_ENTRY_LIST payload from
 * a batch of BSS entries. On success *Buffer holds a library-allocated
 * blob that MUST be released with VwifiTlvFreeGenerated. */
NDIS_STATUS VwifiTlvGenerateBssEntryList(
    _In_ ULONG PeerVersion,
    _In_reads_(Count) const VWIFI_TLV_BSS_ITEM *Items,
    _In_ ULONG Count,
    _Outptr_result_bytebuffer_(*BufferLen) VOID **Buffer,
    _Out_ PULONG BufferLen);

/* No generator for SCAN_COMPLETE or CONNECT_COMPLETE: both are empty
 * messages (EmptyMessageStructureType) that carry no TLVs. Their status
 * goes in WDI_MESSAGE_HEADER.Status — see VwifiSendWdiIndication. */

/* ============================================================
 * Connect
 * ============================================================ */

/* Parse an OID_WDI_TASK_CONNECT M1 into a device connect request.
 *
 * `ReqBuf` must have room for sizeof(struct vwifi_connect_req) plus any
 * association IEs. Note that WDI supplies only vendor-specific elements
 * here — there is no RSN element in a connect request and nowhere to put
 * one. The OS states the security it wants through the auth and cipher
 * algorithm lists, and the device builds RSN from those. An empty
 * assoc_ie_len is the normal case, not a parse failure. */
NDIS_STATUS VwifiTlvParseConnectRequest(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_writes_bytes_to_(ReqCap, *ReqLen) struct vwifi_connect_req *ReqBuf,
    _In_ ULONG ReqCap,
    _Out_ PULONG ReqLen);

NDIS_STATUS VwifiTlvGenerateAssociationResult(
    _In_ ULONG PeerVersion,
    _In_ const struct vwifi_assoc_result *Result,
    _In_reads_bytes_opt_(Result->ie_len) const UCHAR *Ies,
    _Outptr_result_bytebuffer_(*BufferLen) VOID **Buffer,
    _Out_ PULONG BufferLen);

/* `Local` is vwifi_disconnect_ev.local: WDI wants to know who initiated
 * the disconnect, not the 802.11 reason code, which has no counterpart
 * in WDI_ASSOC_STATUS. */
NDIS_STATUS VwifiTlvGenerateDisassociation(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(6) const UCHAR *Bssid,
    _In_ BOOLEAN Local,
    _Outptr_result_bytebuffer_(*BufferLen) VOID **Buffer,
    _Out_ PULONG BufferLen);

/* ============================================================
 * Cipher keys
 * ============================================================ */

/* One key extracted from an OID_WDI_SET_ADD_CIPHER_KEYS M1. */
typedef struct _VWIFI_TLV_KEY
{
    BOOLEAN Pairwise;
    UCHAR   KeyIndex;
    UCHAR   PeerMac[6];
    USHORT  Cipher;         /* VWIFI_CIPHER_* */
    UCHAR   KeyValue[32];
    ULONG   KeyLength;
} VWIFI_TLV_KEY, *PVWIFI_TLV_KEY;

/* Parse the key list. *Count receives how many were written to Keys. */
NDIS_STATUS VwifiTlvParseAddCipherKeys(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_writes_to_(KeysCap, *Count) VWIFI_TLV_KEY *Keys,
    _In_ ULONG KeysCap,
    _Out_ PULONG Count);

NDIS_STATUS VwifiTlvParseDeleteCipherKeys(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_writes_to_(KeysCap, *Count) VWIFI_TLV_KEY *Keys,
    _In_ ULONG KeysCap,
    _Out_ PULONG Count);

/* ============================================================
 * Capabilities
 * ============================================================ */

NDIS_STATUS VwifiTlvGenerateAdapterCapabilities(
    _In_ ULONG PeerVersion,
    _In_ const struct vwifi_caps *Caps,
    _In_reads_bytes_(6) const UCHAR *Mac,
    _Outptr_result_bytebuffer_(*BufferLen) VOID **Buffer,
    _Out_ PULONG BufferLen);

/* ============================================================
 * Adapter configuration
 * ============================================================ */

/* Parse a WDI_SET_ADAPTER_CONFIGURATION and hand back the MAC address
 * the OS wants used, if it supplied one. *MacPresent is FALSE when the
 * optional container is absent, which means "keep the current address"
 * and is not an error. */
NDIS_STATUS VwifiTlvParseAdapterConfiguration(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_writes_bytes_(6) UCHAR *MacOut,
    _Out_ BOOLEAN *MacPresent);

/* ============================================================
 * Cleanup
 *
 * MUST be called for every successful Generate*. Microsoft is blunt
 * about this: "Although you may be tempted to skip calling the
 * library's cleanup routines (such as FreeParsed, CleanupParsed, and
 * FreeGenerated), do not skip calling them! It might work on some
 * code paths, but will lead to hard-to-diagnose memory leaks."
 * ============================================================ */

VOID VwifiTlvFreeGenerated(_In_opt_ VOID *Buffer);

/* ============================================================
 * Operation mode
 * ============================================================ */

/* Parse an OID_WDI_TASK_CHANGE_OPERATION_MODE M1 and hand back the
 * device-side mode (VWIFI_MODE_*) rather than the WDI enum.
 *
 * Only VWIFI_MODE_STA can come out of this: WDI_OPERATION_MODE covers
 * STA and the three P2P roles and nothing else. Monitor mode has no WDI
 * representation, so it cannot be requested through this task. */
NDIS_STATUS VwifiTlvParseOperationMode(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_ PULONG DeviceMode);

#ifdef __cplusplus
}
#endif
