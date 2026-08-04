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
 * `Beacon` and `Probe` are WHOLE frames as seen on the air, NOT just
 * the IE tails. WABIModel's BSSEntryContainer carries
 * WDI_TLV_BEACON_FRAME and WDI_TLV_PROBE_RESPONSE_FRAME as two
 * separate raw byte blobs and the OS parses the IEs itself.
 *
 * Two pointers, not one plus a discriminator. This was a single
 * `Frame` with the kind taken from Entry->capability_info, which made
 * the two TLVs mutually exclusive -- an entry could report a beacon or
 * a probe response but never both, and since the probe response is
 * always the one that arrives during a scan, BEACON_FRAME was in
 * practice never emitted at all. Either may be NULL / zero-length;
 * whichever is present is emitted, and an entry may carry both.
 *
 * Lengths are explicit rather than read from Entry->ie_len: a merged
 * entry holds two frames of two different lengths, and the device's
 * ie_len describes only whichever event happened to arrive last. */
typedef struct _VWIFI_TLV_BSS_ITEM
{
    const struct vwifi_bss_entry *Entry;

    const UCHAR                  *Beacon;     /* BeaconLen bytes, or NULL */
    ULONG                         BeaconLen;
    const UCHAR                  *Probe;      /* ProbeLen bytes, or NULL */
    ULONG                         ProbeLen;

    /* When this entry was discovered, in *host system time*.
     *
     * WDI_TLV_BSS_ENTRY_AGE_INFO's first field is documented down to
     * the API that must produce it: "The timestamp should be obtained
     * with NdisGetCurrentSystemTime or KeQuerySystemTime." That is
     * 100-nanosecond intervals since 1601, and the OS subtracts it from
     * its own clock to age the entry out.
     *
     * The BSS entry's `tsf` is not that and cannot be substituted for
     * it: it is the AP's own microsecond counter since the AP came up,
     * a number some fifteen orders of magnitude smaller. Reported as a
     * host timestamp it makes every BSS four centuries old on arrival,
     * so the entry is stale the moment it lands -- visible for an
     * instant, then aged out of the OS's list, with no fresh entry for
     * a connect to be built from.
     *
     * The caller records it when the frame arrives, not when the
     * indication is generated, because the cache replays entries long
     * after the scan that found them. */
    ULONGLONG                     HostTimeStamp;

    /* 0 (live) if this entry was found by a scan that is running now,
     * 1 (cached) if it is being replayed out of the adapter's own BSS
     * list -- the second field of the same TLV. */
    BOOLEAN                       Cached;
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
 * Ports
 * ============================================================ */

/* Parse an OID_WDI_TASK_CREATE_PORT. The port id itself is in the
 * request's WDI_MESSAGE_HEADER, not here; this is what the port is for.
 * *MacPresent is FALSE when the optional address is absent, meaning
 * "use the adapter's own". */
NDIS_STATUS VwifiTlvParseCreatePort(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_ PULONG OpModeMask,
    _Out_ PULONG NdisPortNumber,
    _Out_writes_bytes_(6) UCHAR *MacOut,
    _Out_ BOOLEAN *MacPresent);

/* Build the M3 for a create-port task. Its PortAttributes container is
 * mandatory -- unlike the delete-port completion, which really is
 * header-only. Release with VwifiTlvFreeGenerated. */
NDIS_STATUS VwifiTlvGenerateCreatePortComplete(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(6) const UCHAR *Mac,
    _In_ ULONG PortNumber,
    _Outptr_result_bytebuffer_(*BufferLen) VOID **Buffer,
    _Out_ PULONG BufferLen);

NDIS_STATUS VwifiTlvParseDeletePort(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_ PULONG PortNumber);

/* ============================================================
 * Statistics
 * ============================================================ */

/* Build a WDI_GET_STATISTICS reply. Both of its containers are
 * mandatory, so there is no such thing as an empty answer. Release with
 * VwifiTlvFreeGenerated. */
NDIS_STATUS VwifiTlvGenerateStatistics(
    _In_ ULONG PeerVersion,
    _Outptr_result_bytebuffer_(*BufferLen) VOID **Buffer,
    _Out_ PULONG BufferLen);

/* ============================================================
 * Dot11 reset / receive packet filter
 * ============================================================ */

NDIS_STATUS VwifiTlvParseDot11Reset(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_ BOOLEAN *SetDefaultMib,
    _Out_writes_bytes_(6) UCHAR *MacOut,
    _Out_ BOOLEAN *MacPresent);

NDIS_STATUS VwifiTlvParseReceivePacketFilter(
    _In_ ULONG PeerVersion,
    _In_reads_bytes_(BufferLen) const VOID *Buffer,
    _In_ ULONG BufferLen,
    _Out_ PULONG Filter);

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
