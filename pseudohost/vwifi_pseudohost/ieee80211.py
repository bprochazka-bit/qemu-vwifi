#
# vwifi-pseudohost — 802.11 frame and information-element helpers
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Just enough of 802.11 to be a station: build/parse the management
# frames of an association (probe, auth, assoc), read the information
# elements out of a beacon (SSID, DS channel, RSN), and move data frames
# to and from Ethernet across the LLC/SNAP boundary.
#
# The frame formats here match what devices/vwifi/src/vwifi_device.c
# constructs, because the same hostapd sits on the other side of both.
#
import struct

# ---- frame control -------------------------------------------------------
FTYPE_MGMT = 0
FTYPE_CTRL = 1
FTYPE_DATA = 2

# management subtypes
STYPE_ASSOC_REQ = 0
STYPE_ASSOC_RESP = 1
STYPE_PROBE_REQ = 4
STYPE_PROBE_RESP = 5
STYPE_BEACON = 8
STYPE_DISASSOC = 10
STYPE_AUTH = 11
STYPE_DEAUTH = 12

# frame-control byte-1 bits
FC1_TODS = 0x01
FC1_FROMDS = 0x02
FC1_PROTECTED = 0x40

# information element IDs
EID_SSID = 0
EID_SUPP_RATES = 1
EID_DS_PARAMS = 3
EID_EXT_SUPP_RATES = 50
EID_RSN = 48

# auth algorithms / status / reason
AUTH_ALG_OPEN = 0
STATUS_SUCCESS = 0

RSN_OUI = bytes([0x00, 0x0F, 0xAC])
# RSN suite selectors (last byte of the OUI+type)
CIPHER_CCMP = 4
AKM_PSK = 2

# LLC/SNAP header that prefixes an Ethernet payload inside an 802.11 MSDU.
LLC_SNAP = bytes([0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00])
ETH_P_IP = 0x0800
ETH_P_ARP = 0x0806
ETH_P_PAE = 0x888E          # EAPOL

BROADCAST = b"\xff\xff\xff\xff\xff\xff"


def mac_str(b):
    return ":".join("%02x" % x for x in b)


def mac_bytes(s):
    if isinstance(s, (bytes, bytearray)):
        return bytes(s)
    return bytes(int(x, 16) for x in s.split(":"))


def _u16(p, o):
    return p[o] | (p[o + 1] << 8)


# ---- channel <-> frequency ----------------------------------------------

def chan_to_freq(ch):
    if 1 <= ch <= 13:
        return 2407 + ch * 5
    if ch == 14:
        return 2484
    if 36 <= ch <= 165:
        return 5000 + ch * 5
    return 0


def freq_to_chan(freq):
    if freq == 0:
        return 0
    if freq == 2484:
        return 14
    if 2412 <= freq <= 2472:
        return (freq - 2407) // 5
    if 5160 <= freq <= 5885:
        return (freq - 5000) // 5
    return 0


# ---- information elements ------------------------------------------------

def ie(eid, body):
    return bytes([eid, len(body)]) + bytes(body)


def parse_ies(buf):
    """Return {eid: body_bytes} for the first occurrence of each element."""
    out = {}
    i = 0
    n = len(buf)
    while i + 2 <= n:
        eid = buf[i]
        ln = buf[i + 1]
        if i + 2 + ln > n:
            break
        out.setdefault(eid, buf[i + 2 : i + 2 + ln])
        i += 2 + ln
    return out


# The default legacy 802.11g supported-rate set, matching what the device
# advertises.  Basic-rate bit (0x80) set on 1/2/5.5/11.  Rates in units of
# 500 kbps.
_SUPP_RATES = bytes([0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24])
_EXT_RATES = bytes([0x30, 0x48, 0x60, 0x6C])


def supp_rate_ies():
    return ie(EID_SUPP_RATES, _SUPP_RATES) + ie(EID_EXT_SUPP_RATES, _EXT_RATES)


def rsn_ie_ccmp_psk():
    """A minimal RSN element: CCMP group + pairwise, PSK AKM.

    Byte-identical to what a WPA2-PSK/CCMP hostapd expects to see echoed
    back in the association request, and to ie_put_rsn() in the device.
    """
    body = bytearray()
    body += struct.pack("<H", 1)                     # version
    body += RSN_OUI + bytes([CIPHER_CCMP])           # group cipher
    body += struct.pack("<H", 1) + RSN_OUI + bytes([CIPHER_CCMP])   # pairwise
    body += struct.pack("<H", 1) + RSN_OUI + bytes([AKM_PSK])       # AKM
    body += struct.pack("<H", 0)                     # RSN capabilities
    return ie(EID_RSN, bytes(body))


def parse_rsn(body):
    """Very small RSN parser: is it PSK, and does it name CCMP pairwise?

    Returns a dict {group, pairwise:[...], akms:[...]} of suite last-bytes,
    or None if the element is malformed.
    """
    try:
        o = 2                                        # skip version
        group = body[o + 3]
        o += 4
        pc = _u16(body, o)
        o += 2
        pairwise = [body[o + 4 * i + 3] for i in range(pc)]
        o += 4 * pc
        ac = _u16(body, o)
        o += 2
        akms = [body[o + 4 * i + 3] for i in range(ac)]
        return {"group": group, "pairwise": pairwise, "akms": akms}
    except IndexError:
        return None


# ---- management frames ---------------------------------------------------

class SeqCounter:
    def __init__(self):
        self._n = 0

    def next(self):
        v = self._n & 0x0FFF
        self._n += 1
        return v << 4


def mgmt_header(subtype, da, sa, bssid, seq):
    b = bytearray(24)
    b[0] = subtype << 4                              # type=0 (mgmt)
    b[1] = 0
    b[4:10] = da
    b[10:16] = sa
    b[16:22] = bssid
    struct.pack_into("<H", b, 22, seq)
    return b


def build_probe_req(sa, ssid, seq):
    b = mgmt_header(STYPE_PROBE_REQ, BROADCAST, sa, BROADCAST, seq)
    b += ie(EID_SSID, ssid)
    b += supp_rate_ies()
    return bytes(b)


def build_auth(sa, bssid, seq):
    b = mgmt_header(STYPE_AUTH, bssid, sa, bssid, seq)
    b += struct.pack("<HHH", AUTH_ALG_OPEN, 1, STATUS_SUCCESS)
    return bytes(b)


def build_assoc_req(sa, bssid, ssid, freq, seq, protected=False):
    b = mgmt_header(STYPE_ASSOC_REQ, bssid, sa, bssid, seq)
    cap = 0x0431 | (0x0010 if protected else 0)      # ESS + Privacy(if RSN)
    b += struct.pack("<HH", cap, 10)                 # capability, listen int
    b += ie(EID_SSID, ssid)
    b += supp_rate_ies()
    b += ie(EID_DS_PARAMS, bytes([freq_to_chan(freq)]))
    if protected:
        b += rsn_ie_ccmp_psk()
    return bytes(b)


def build_deauth(sa, bssid, reason, seq):
    b = mgmt_header(STYPE_DEAUTH, bssid, sa, bssid, seq)
    b += struct.pack("<H", reason)
    return bytes(b)


# ---- AP-side management frames -------------------------------------------

def build_beacon(bssid, ssid, freq, privacy, seq, da=BROADCAST,
                 subtype=STYPE_BEACON):
    """A beacon (or, with subtype=ProbeResp and a unicast da, a probe
    response).  Capability carries the Privacy bit and an RSN element
    when the network is secured."""
    b = mgmt_header(subtype, da, bssid, bssid, seq)
    cap = 0x0421 | (0x0010 if privacy else 0)      # ESS + short pre/slot
    b += struct.pack("<QHH", 0, 100, cap)          # tsf, beacon int, cap
    b += ie(EID_SSID, ssid)
    b += supp_rate_ies()
    b += ie(EID_DS_PARAMS, bytes([freq_to_chan(freq)]))
    if privacy:
        b += rsn_ie_ccmp_psk()
    return bytes(b)


def build_auth_resp(bssid, da, seq, status=STATUS_SUCCESS):
    b = mgmt_header(STYPE_AUTH, da, bssid, bssid, seq)
    b += struct.pack("<HHH", AUTH_ALG_OPEN, 2, status)   # alg, seq 2, status
    return bytes(b)


def build_assoc_resp(bssid, da, aid, seq, status=STATUS_SUCCESS):
    b = mgmt_header(STYPE_ASSOC_RESP, da, bssid, bssid, seq)
    b += struct.pack("<HHH", 0x0421, status, aid)        # cap, status, AID
    b += supp_rate_ies()
    return bytes(b)


def build_data_fromds(bssid, sa, da, eth_payload, protected=False, seq=0):
    """A FromDS non-QoS data frame (AP -> station).

    addr1=RA=DA (the station), addr2=TA=BSSID, addr3=SA (original sender:
    the AP itself, or another station when bridging).
    """
    ethertype, sdu = eth_payload
    b = bytearray(24)
    b[0] = FTYPE_DATA << 2
    b[1] = FC1_FROMDS | (FC1_PROTECTED if protected else 0)
    b[4:10] = da
    b[10:16] = bssid
    b[16:22] = sa
    struct.pack_into("<H", b, 22, (seq & 0x0FFF) << 4)
    b += LLC_SNAP + struct.pack(">H", ethertype) + sdu
    return bytes(b)


def frame_type_subtype(frame):
    fc = frame[0]
    return (fc >> 2) & 0x3, (fc >> 4) & 0xF


def is_beacon_or_probe_resp(frame):
    if len(frame) < 24:
        return False
    t, s = frame_type_subtype(frame)
    return t == FTYPE_MGMT and s in (STYPE_BEACON, STYPE_PROBE_RESP)


class Beacon:
    """A parsed beacon / probe response: the fields a station acts on."""

    __slots__ = ("bssid", "ssid", "channel_freq", "capability", "rsn",
                 "privacy")

    def __init__(self, frame, medium_freq):
        self.bssid = bytes(frame[16:22])             # addr3
        self.capability = _u16(frame, 34)
        self.privacy = bool(self.capability & 0x0010)
        ies = parse_ies(frame[36:])
        self.ssid = bytes(ies.get(EID_SSID, b""))
        ds = ies.get(EID_DS_PARAMS)
        if ds:
            self.channel_freq = chan_to_freq(ds[0])
        else:
            self.channel_freq = medium_freq
        rsn_body = ies.get(EID_RSN)
        self.rsn = parse_rsn(rsn_body) if rsn_body else None


# ---- data frames <-> Ethernet -------------------------------------------

def build_data_frame(bssid, sa, da, eth_payload, protected=False, seq=0):
    """A ToDS non-QoS data frame carrying an Ethernet payload as LLC/SNAP.

    `eth_payload` is (ethertype, sdu_bytes).  For the station-to-AP path
    addr1=BSSID, addr2=SA(us), addr3=DA.
    """
    ethertype, sdu = eth_payload
    b = bytearray(24)
    b[0] = FTYPE_DATA << 2                            # data, subtype 0
    b[1] = FC1_TODS | (FC1_PROTECTED if protected else 0)
    b[4:10] = bssid                                  # addr1 = RA = BSSID
    b[10:16] = sa                                    # addr2 = TA = us
    b[16:22] = da                                    # addr3 = DA
    struct.pack_into("<H", b, 22, (seq & 0x0FFF) << 4)
    b += LLC_SNAP + struct.pack(">H", ethertype) + sdu
    return bytes(b)


def parse_data_frame(frame):
    """Decode a received (already-decrypted) data frame.

    Returns (src_mac, dst_mac, ethertype, sdu) or None if it isn't a
    data frame we can turn into Ethernet.
    """
    if len(frame) < 24:
        return None
    t, s = frame_type_subtype(frame)
    if t != FTYPE_DATA:
        return None
    if s & 0x04:                                     # Null-func / no-data
        return None
    tods = bool(frame[1] & FC1_TODS)
    fromds = bool(frame[1] & FC1_FROMDS)
    hlen = 24
    if tods and fromds:
        hlen += 6                                    # addr4 (mesh/WDS)
    if s & 0x08:                                     # QoS data
        qos_off = hlen
        hlen += 2
        _ = qos_off
    addr1 = bytes(frame[4:10])
    addr2 = bytes(frame[10:16])
    addr3 = bytes(frame[16:22])
    if fromds and not tods:
        da, sa = addr1, addr3                        # AP -> STA
    elif tods and not fromds:
        da, sa = addr3, addr2                        # STA -> AP
    else:
        da, sa = addr1, addr2
    body = frame[hlen:]
    if len(body) < 8 or body[:6] != LLC_SNAP:
        return None
    ethertype = (body[6] << 8) | body[7]
    return sa, da, ethertype, bytes(body[8:])
