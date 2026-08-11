#
# vwifi-pseudohost — a minimal SNMP agent (printer identity)
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# When Windows adds a "Standard TCP/IP Port" it doesn't just open a raw
# socket — its port monitor first SNMP-walks the device (community
# "public", UDP 161) to decide whether it is a printer and what to call
# it.  A device that stays silent fails detection.  This agent answers
# the OIDs that detection reads: system group, the Host Resources device
# table (marking the device a printer, Idle), and the Printer MIB name —
# enough for Windows to identify "HP OfficeJet Pro 9015" and finish the
# TCP/IP port as a raw:9100 printer, which then prints to the sink.
#
# SNMP v1 and v2c GetRequest / GetNextRequest only (that is all the port
# monitor and a `snmpwalk` need); no Set, no traps.  Just enough BER to
# speak it.
#
from .services import UDPService

# BER tags
T_INT = 0x02
T_OCTSTR = 0x04
T_NULL = 0x05
T_OID = 0x06
T_SEQ = 0x30
T_IPADDR = 0x40
T_COUNTER = 0x41
T_GAUGE = 0x42
T_TIMETICKS = 0x43
PDU_GET = 0xA0
PDU_GETNEXT = 0xA1
PDU_RESPONSE = 0xA2

NOSUCHNAME = 2


# ---- BER encode ----------------------------------------------------------
def _enc_len(n):
    if n < 0x80:
        return bytes([n])
    b = b""
    while n:
        b = bytes([n & 0xFF]) + b
        n >>= 8
    return bytes([0x80 | len(b)]) + b


def _tlv(tag, val):
    return bytes([tag]) + _enc_len(len(val)) + val


def _enc_int(n):
    if n == 0:
        return _tlv(T_INT, b"\x00")
    neg = n < 0
    b = b""
    v = n if not neg else (-n - 1)
    while v > 0:
        b = bytes([v & 0xFF]) + b
        v >>= 8
    if not b:
        b = b"\x00"
    if not neg and (b[0] & 0x80):
        b = b"\x00" + b
    return _tlv(T_INT, b)


def _enc_uint(tag, n):
    b = b""
    while n:
        b = bytes([n & 0xFF]) + b
        n >>= 8
    if not b:
        b = b"\x00"
    if b[0] & 0x80:
        b = b"\x00" + b
    return _tlv(tag, b)


def _enc_oid(oid):
    first = 40 * oid[0] + oid[1]
    body = bytes([first])
    for sub in oid[2:]:
        if sub < 0x80:
            body += bytes([sub])
        else:
            stack = []
            stack.append(sub & 0x7F)
            sub >>= 7
            while sub:
                stack.append((sub & 0x7F) | 0x80)
                sub >>= 7
            body += bytes(reversed(stack))
    return _tlv(T_OID, body)


# ---- BER decode ----------------------------------------------------------
def _rd_len(buf, i):
    n = buf[i]
    i += 1
    if n < 0x80:
        return n, i
    cnt = n & 0x7F
    val = 0
    for _ in range(cnt):
        val = (val << 8) | buf[i]
        i += 1
    return val, i


def _rd_tlv(buf, i):
    tag = buf[i]
    ln, i = _rd_len(buf, i + 1)
    return tag, buf[i:i + ln], i + ln


def _dec_int(b):
    if not b:
        return 0
    n = 0
    for x in b:
        n = (n << 8) | x
    if b[0] & 0x80:
        n -= 1 << (8 * len(b))
    return n


def _dec_oid(b):
    if not b:
        return ()
    out = [b[0] // 40, b[0] % 40]
    val = 0
    for x in b[1:]:
        val = (val << 7) | (x & 0x7F)
        if not (x & 0x80):
            out.append(val)
            val = 0
    return tuple(out)


class SNMPAgent(UDPService):
    name = "snmp"
    port = 161

    def on_start(self):
        self._oids = self._build_table()
        self._sorted = sorted(self._oids)

    # -- the MIB the port monitor reads -----------------------------------
    def _build_table(self):
        h = self.host
        model = getattr(h, "wsd_model", getattr(h, "hostname", "Printer"))
        name = getattr(h, "hostname", "Printer")
        mac = getattr(h, "mac", b"\x00" * 6)
        # (oid): (ber_tag, encoded_value_bytes)
        def s(text):
            return (T_OCTSTR, text.encode() if isinstance(text, str) else text)
        return {
            (1, 3, 6, 1, 2, 1, 1, 1, 0): s(model),                # sysDescr
            (1, 3, 6, 1, 2, 1, 1, 2, 0):
                (T_OID, (1, 3, 6, 1, 4, 1, 11, 2, 3, 9, 1)),       # sysObjectID (HP)
            (1, 3, 6, 1, 2, 1, 1, 3, 0): (T_TIMETICKS, 100000),   # sysUpTime
            (1, 3, 6, 1, 2, 1, 1, 4, 0): s("admin"),              # sysContact
            (1, 3, 6, 1, 2, 1, 1, 5, 0): s(name),                 # sysName
            (1, 3, 6, 1, 2, 1, 1, 6, 0): s("Front Office"),       # sysLocation
            (1, 3, 6, 1, 2, 1, 1, 7, 0): (T_INT, 64),             # sysServices
            (1, 3, 6, 1, 2, 1, 2, 2, 1, 6, 1): (T_OCTSTR, bytes(mac)),  # ifPhysAddress
            # Host Resources device table: this device is a printer, running.
            (1, 3, 6, 1, 2, 1, 25, 3, 2, 1, 2, 1):
                (T_OID, (1, 3, 6, 1, 2, 1, 25, 3, 1, 5)),         # hrDeviceType=printer
            (1, 3, 6, 1, 2, 1, 25, 3, 2, 1, 3, 1): s(model),      # hrDeviceDescr
            (1, 3, 6, 1, 2, 1, 25, 3, 2, 1, 5, 1): (T_INT, 2),    # hrDeviceStatus=running
            # Printer group
            (1, 3, 6, 1, 2, 1, 25, 3, 5, 1, 1, 1): (T_INT, 3),    # hrPrinterStatus=idle
            (1, 3, 6, 1, 2, 1, 25, 3, 5, 1, 2, 1):
                (T_OCTSTR, b"\x00"),                              # detectedErrorState=none
            (1, 3, 6, 1, 2, 1, 43, 5, 1, 1, 16, 1): s(name),      # prtGeneralPrinterName
            (1, 3, 6, 1, 2, 1, 43, 5, 1, 1, 17, 1): s(mac.hex()),  # prtGeneralSerialNumber
        }

    def _encode_value(self, entry):
        tag, val = entry
        if tag == T_OID:
            return _enc_oid(val)
        if tag in (T_TIMETICKS, T_COUNTER, T_GAUGE):
            return _enc_uint(tag, val)
        if tag == T_INT:
            return _enc_int(val)
        return _tlv(T_OCTSTR, val)

    # -- request handling --------------------------------------------------
    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        resp = self._respond(payload)
        if resp is not None:
            self.log("SNMP query from %d.%d.%d.%d -> response" % tuple(src_ip))
            self.reply_udp(src_ip, src_port, resp, src_port=dst_port)

    def _respond(self, msg):
        try:
            tag, body, _ = _rd_tlv(msg, 0)
            if tag != T_SEQ:
                return None
            i = 0
            vt, ver, i = _rd_tlv(body, i)
            version = _dec_int(ver)
            ct, community, i = _rd_tlv(body, i)
            ptag, pdu, i = _rd_tlv(body, i)
        except (IndexError, ValueError):
            return None
        if ptag not in (PDU_GET, PDU_GETNEXT):
            return None
        try:
            j = 0
            _t, reqid, j = _rd_tlv(pdu, j)
            _t, _es, j = _rd_tlv(pdu, j)
            _t, _ei, j = _rd_tlv(pdu, j)
            _t, vbl, j = _rd_tlv(pdu, j)          # variable-bindings SEQUENCE
        except (IndexError, ValueError):
            return None

        out_vbs = []
        err_status, err_index = 0, 0
        k = 0
        idx = 0
        while k < len(vbl):
            idx += 1
            try:
                _t, vb, k = _rd_tlv(vbl, k)
                ot, oidb, _ = _rd_tlv(vb, 0)
                oid = _dec_oid(oidb)
            except (IndexError, ValueError):
                break
            roid, entry = self._lookup(oid, ptag == PDU_GETNEXT)
            if entry is None:
                if version == 0:                  # v1: whole-PDU error
                    err_status, err_index = NOSUCHNAME, idx
                    out_vbs.append(_tlv(T_SEQ,
                                        _enc_oid(oid) + _tlv(T_NULL, b"")))
                else:                             # v2c: endOfMibView marker
                    out_vbs.append(_tlv(T_SEQ,
                                        _enc_oid(oid) + _tlv(0x82, b"")))
            else:
                out_vbs.append(_tlv(T_SEQ,
                                    _enc_oid(roid) + self._encode_value(entry)))

        pdu_out = (_tlv(T_INT, reqid)
                   + _enc_int(err_status) + _enc_int(err_index)
                   + _tlv(T_SEQ, b"".join(out_vbs)))
        resp_pdu = _tlv(PDU_RESPONSE, pdu_out)
        return _tlv(T_SEQ, _enc_int(version) + _tlv(T_OCTSTR, community)
                    + resp_pdu)

    def _lookup(self, oid, is_next):
        if not is_next:
            entry = self._oids.get(oid)
            return oid, entry
        # GetNext: the smallest OID strictly greater than the requested one.
        for cand in self._sorted:
            if cand > oid:
                return cand, self._oids[cand]
        return oid, None
