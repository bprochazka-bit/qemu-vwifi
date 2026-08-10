/*
 * vwifi — wdi_peer.c
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * WDI peers: the thing the data path is addressed by, and the thing
 * this driver has never had.
 *
 * Why a peer exists
 * -----------------
 * WDI does not send data to a port. It sends data to a (port, peer,
 * TID) triple: MiniportWdiTxDataSend takes all three, RxInorderData
 * indications carry the peer id, and WDI_TX_METADATA names the peer
 * on every frame. A port with no peers has nowhere to send anything.
 *
 * A station has exactly one peer -- the AP it is associated to -- and
 * it comes into existence at association and goes away at disconnect.
 * The miniport is what says so: NdisWdiPeerCreateIndication tells the
 * WLAN component that a peer now exists, what its MAC is, and which
 * id the miniport will use for it from then on.
 *
 * Until that call is made, TX is paused. dot11wdi.h says so directly:
 *
 *     WDI_TX_PAUSE_REASON_PEER_CREATE = 0x00000002
 *
 * -- a pause reason whose whole meaning is "no peer yet". That is why
 * MiniportWdiTxDataSend has never once fired in this driver's traces:
 * not because the component had nothing to send, but because it had
 * nobody to send it to.
 *
 * Who assigns the id
 * ------------------
 * We do. The component takes WDI_PEER_ID as an input on the create
 * indication and uses it verbatim afterwards, so the id is the
 * miniport's to choose and its only real constraint is that it stays
 * below the MaxNumPeers the component states in TalTxRxStart -- eight,
 * on this device. Slot index doubles as id, which keeps the lookup a
 * bounds check rather than a search.
 *
 * Delete is asynchronous, create is not
 * -------------------------------------
 * NdisWdiPeerCreateIndication returns void and takes no status: once
 * it returns, the peer is created. NdisWdiPeerDeleteIndication has an
 * _Out_ NDIS_STATUS *and* a matching callback,
 * TalTxRxPeerDeleteConfirmHandler, which is how the component says it
 * has finished tearing down its own per-peer state. The slot is not
 * reusable until that confirmation arrives, so a peer being deleted
 * stays in the table with DeletePending set rather than being freed on
 * the spot.
 *
 * Nothing here talks to the device. The vwifi device tracks its own
 * association and has no concept of a WDI peer id; this table exists
 * purely to hold the mapping the WDI data path is addressed through.
 */

#include "vwifi_drv.h"

/* ============================================================
 * Table
 * ============================================================ */

static PVWIFI_PEER
VwifiPeerSlot(_In_ PVWIFI_ADAPTER Adapter, _In_ WDI_PEER_ID PeerId)
{
    if (PeerId >= VWIFI_MAX_PEERS) return NULL;
    return &Adapter->Peers[PeerId];
}

PVWIFI_PEER
VwifiPeerFind(_In_ PVWIFI_ADAPTER Adapter, _In_ WDI_PEER_ID PeerId)
{
    PVWIFI_PEER p = VwifiPeerSlot(Adapter, PeerId);

    return (p != NULL && p->InUse) ? p : NULL;
}

/* The peer a STA data frame belongs to.
 *
 * One association means one peer, so this does not need to match on a
 * MAC address: the first live, fully configured peer is the AP. It is
 * written as a search anyway because the alternative -- caching "the"
 * peer id in the adapter -- goes stale in exactly the window where it
 * matters, between PeerDeleteIndication and its confirmation. */
PVWIFI_PEER
VwifiPeerFirstActive(_In_ PVWIFI_ADAPTER Adapter)
{
    for (ULONG i = 0; i < VWIFI_MAX_PEERS; i++) {
        PVWIFI_PEER p = &Adapter->Peers[i];

        if (p->InUse && !p->DeletePending) return p;
    }
    return NULL;
}

/* ============================================================
 * Create
 * ============================================================ */

NDIS_STATUS
VwifiPeerCreate(_Inout_ PVWIFI_ADAPTER Adapter,
                _In_ WDI_PORT_ID WdiPortId,
                _In_reads_bytes_(6) const UCHAR *Mac,
                _Out_ WDI_PEER_ID *PeerIdOut)
{
    PVWIFI_PEER p = NULL;
    ULONG limit = Adapter->MaxPeers ? Adapter->MaxPeers : VWIFI_MAX_PEERS;
    WDI_MAC_ADDRESS addr;

    *PeerIdOut = WDI_PEER_ANY;

    if (Adapter->DataPathApi == NULL ||
        Adapter->DataPathApi->PeerCreateIndication == NULL) {
        VWIFI_ERR("peer create: no PeerCreateIndication in the data API -- "
                  "the component cannot be told this peer exists, and TX "
                  "will stay paused on WDI_TX_PAUSE_REASON_PEER_CREATE");
        return NDIS_STATUS_NOT_SUPPORTED;
    }

    /* Clamped to whatever TalTxRxStart said, not to the array bound.
     * The component sizes its own per-peer state from MaxNumPeers and
     * an id past it is one it has nowhere to put. */
    if (limit > VWIFI_MAX_PEERS) limit = VWIFI_MAX_PEERS;

    for (ULONG i = 0; i < limit; i++) {
        if (!Adapter->Peers[i].InUse) {
            p = &Adapter->Peers[i];
            p->PeerId = (WDI_PEER_ID)i;
            break;
        }
    }
    if (p == NULL) {
        VWIFI_ERR("peer create: all %u peer slots in use", limit);
        return NDIS_STATUS_RESOURCES;
    }

    p->InUse         = TRUE;
    p->DeletePending = FALSE;
    p->Configured    = FALSE;
    p->WdiPortId     = WdiPortId;
    RtlCopyMemory(p->Mac, Mac, 6);

    /* By value, not by pointer. NDIS_WDI_PEER_CREATE_IND takes
     * WDI_MAC_ADDRESS itself, which is a six-byte struct -- passing the
     * address of our copy would compile only by accident. */
    RtlCopyMemory(addr.Address, Mac, 6);

    VWIFI_INFO("peer create: id %u port %u %02x:%02x:%02x:%02x:%02x:%02x",
               p->PeerId, WdiPortId,
               Mac[0], Mac[1], Mac[2], Mac[3], Mac[4], Mac[5]);

    /* Published before the indication, not after: the component may
     * call TalTxRxPeerConfigHandler -- or start dequeuing TX -- from
     * inside this call, and both look the peer up in this table. */
    Adapter->DataPathApi->PeerCreateIndication(Adapter->DataPathHandle,
                                               WdiPortId, p->PeerId, addr);

    *PeerIdOut = p->PeerId;
    return NDIS_STATUS_SUCCESS;
}

/* ============================================================
 * Delete
 * ============================================================ */

VOID
VwifiPeerDelete(_Inout_ PVWIFI_ADAPTER Adapter, _In_ WDI_PEER_ID PeerId)
{
    PVWIFI_PEER p = VwifiPeerFind(Adapter, PeerId);
    NDIS_STATUS status = NDIS_STATUS_SUCCESS;

    if (p == NULL || p->DeletePending) return;

    if (Adapter->DataPathApi == NULL ||
        Adapter->DataPathApi->PeerDeleteIndication == NULL) {
        /* Nothing to tell, so nothing to wait for. Drop the slot rather
         * than leak it -- a peer the component was never told about
         * cannot have state to tear down. */
        VWIFI_WARN("peer delete: no PeerDeleteIndication; releasing slot %u",
                   PeerId);
        RtlZeroMemory(p, sizeof(*p));
        return;
    }

    VWIFI_INFO("peer delete: id %u port %u", PeerId, p->WdiPortId);
    p->DeletePending = TRUE;

    Adapter->DataPathApi->PeerDeleteIndication(Adapter->DataPathHandle,
                                               p->WdiPortId, PeerId,
                                               &status);
    if (status != NDIS_STATUS_SUCCESS) {
        /* The component refused or failed the delete. The slot cannot
         * be reused -- the component still believes the peer exists --
         * so it stays occupied and says why. */
        VWIFI_ERR("peer delete: id %u refused 0x%08x %s; slot stays in use",
                  PeerId, status, VwifiNdisStatusName(status));
        return;
    }

    /* Success here means "accepted", not "finished". The slot is
     * released by VwifiPeerOnDeleteConfirm. */
}

VOID
VwifiPeerDeleteAll(_Inout_ PVWIFI_ADAPTER Adapter)
{
    for (ULONG i = 0; i < VWIFI_MAX_PEERS; i++) {
        if (Adapter->Peers[i].InUse) {
            VwifiPeerDelete(Adapter, (WDI_PEER_ID)i);
        }
    }
}

/* Release every slot without telling anyone.
 *
 * For teardown after the data path is already gone -- TalTxRxDeinitialize
 * has run, or the adapter is being freed -- where an indication would
 * be made through an API table that is no longer valid. */
VOID
VwifiPeerForgetAll(_Inout_ PVWIFI_ADAPTER Adapter)
{
    for (ULONG i = 0; i < VWIFI_MAX_PEERS; i++) {
        if (Adapter->Peers[i].InUse) {
            VWIFI_INFO("peer forget: id %u (no data path left to tell)",
                       Adapter->Peers[i].PeerId);
        }
        RtlZeroMemory(&Adapter->Peers[i], sizeof(Adapter->Peers[i]));
    }
}

/* ============================================================
 * Callbacks from the TAL
 * ============================================================ */

VOID
VwifiPeerOnConfig(_Inout_ PVWIFI_ADAPTER Adapter,
                  _In_ WDI_PORT_ID PortId,
                  _In_ WDI_PEER_ID PeerId,
                  _In_opt_ const WDI_TXRX_PEER_CFG *Cfg)
{
    PVWIFI_PEER p = VwifiPeerFind(Adapter, PeerId);

    /* What the component says about this peer, which this driver used
     * to throw away with an UNREFERENCED_PARAMETER.
     *
     * WDI_TXRX_PEER_CFG::PeerQoSConfig is WDI_TXRX_PeerCfgQosNone (0),
     * QosCapable (1) or UapsdTids (2). It is the component's answer to
     * "may I address this peer by QoS TID", and the receive path has
     * been answering that question for itself -- announcing every frame
     * on TID 0 regardless. Logged rather than acted on: the TID now
     * comes off the frame, and this is here to show whether the two
     * agree. A peer the component calls QosNone that is nevertheless
     * being handed QoS TIDs is a bug the log should name out loud. */
    if (Cfg != NULL) {
        VWIFI_INFO("peer config: peer %u QoS config %u (%s)",
                   PeerId, (ULONG)Cfg->PeerQoSConfig,
                   (Cfg->PeerQoSConfig == WDI_TXRX_PeerCfgQosNone)
                       ? "none -- non-QoS TIDs only"
                       : (Cfg->PeerQoSConfig == WDI_TXRX_PeerCfgQosCapable)
                           ? "QoS capable"
                           : "U-APSD TIDs");
    } else {
        VWIFI_WARN("peer config: peer %u arrived with no WDI_TXRX_PEER_CFG",
                   PeerId);
    }

    if (p == NULL) {
        /* Worth an error rather than a shrug: it means the component
         * and this table disagree about which peers exist, and every
         * frame for this peer afterwards has nowhere to go. */
        VWIFI_ERR("peer config for unknown peer %u on port %u", PeerId, PortId);
        return;
    }

    p->Configured = TRUE;
    VWIFI_INFO("peer config: id %u port %u -- peer is now configured for data",
               PeerId, PortId);

    /* Anything that arrived before this moment is still queued, and this
     * is the event that makes it deliverable. In the WPA2 trace the AP's
     * EAPOL-Key message 1 landed in the gap between CONNECT_COMPLETE and
     * this callback; unannounced, it sits in the queue while the OS
     * times the four-way handshake out and disconnects. */
    VwifiTalRxAnnounceHeld(Adapter, "the peer is configured for data");

    /* And now it may actually be sent to. Creating a peer pauses its TX
     * -- WDI_TX_PAUSE_REASON_PEER_CREATE -- and only the miniport can
     * lift that. This is the first moment it is true to do so: the
     * component has just said it has finished setting the peer up. */
    VwifiTalTxRestartPeer(Adapter, PortId, PeerId);
}

VOID
VwifiPeerOnDeleteConfirm(_Inout_ PVWIFI_ADAPTER Adapter,
                         _In_ WDI_PORT_ID PortId,
                         _In_ WDI_PEER_ID PeerId)
{
    PVWIFI_PEER p = VwifiPeerSlot(Adapter, PeerId);

    if (p == NULL || !p->InUse) {
        VWIFI_WARN("peer delete confirm for unknown peer %u on port %u",
                   PeerId, PortId);
        return;
    }

    VWIFI_INFO("peer delete confirm: id %u port %u -- slot released",
               PeerId, PortId);
    RtlZeroMemory(p, sizeof(*p));
}
