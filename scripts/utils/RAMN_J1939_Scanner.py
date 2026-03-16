#!/usr/bin/env python3
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION. ALL RIGHTS RESERVED.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
J1939 CA (Controller Application) scanner.

Probes a CAN bus for J1939 ECUs using five techniques and returns every
source address (SA) discovered, together with **all** methods that
detected it.

Techniques
----------
addr_claim   Broadcast Request (PF=0xEA, DA=0xFF) for PGN 60928
             → Address Claimed (PF=0xEE, SA=ECU-SA)

ecu_id       Broadcast Request (PF=0xEA, DA=0xFF) for PGN 64965
             → TP.CM BAM (PF=0xEC, ctrl=0x20) announcing PGN 64965

unicast      Unicast Request (PF=0xEA, DA=ECU-SA) for PGN 60928
             → Any extended CAN frame whose SA equals the probed DA

rts_probe    TP.CM_RTS (PF=0xEC, DA=ECU-SA)
             → TP.CM_CTS (ctrl=0x11) or TP_Conn_Abort (ctrl=0xFF)

uds          UDS Tester Present (PF=0xDA, DA=ECU-SA)
             → Positive response 0x7E from ECU-SA

Return value
------------
``j1939_scan()`` returns a **list** of ``(sa, detections)`` tuples,
sorted by SA in ascending order.  Each *detections* entry is itself a
list of dicts ``{'method': str, 'packet': packet}``, one per technique
that detected the SA.
"""

import logging
import struct
import time

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# J1939 protocol constants (matching firmware ramn_j1939.h)
# ---------------------------------------------------------------------------
PGN_REQUEST = 59904  # 0xEA00
PGN_TP_DT = 60160  # 0xEB00
PGN_TP_CM = 60416  # 0xEC00
PGN_ADDRESS_CLAIMED = 60928  # 0xEE00
PGN_ECU_ID = 64965  # 0xFDC5

PF_REQUEST = 0xEA
PF_TP_DT = 0xEB
PF_TP_CM = 0xEC
PF_ADDRESS_CLAIMED = 0xEE
PF_DIAG = 0xDA

TP_CM_RTS = 0x10
TP_CM_CTS = 0x11
TP_CM_BAM = 0x20
TP_CM_ABORT = 0xFF

DA_BROADCAST = 0xFF

SCANNER_SA = 0xFE  # SA used by the scanner tool

# UDS constants
UDS_TESTER_PRESENT_REQUEST = b'\x02\x3e\x00'   # Tester Present, no suppress
UDS_TESTER_PRESENT_RESPONSE = b'\x02\x7e\x00'  # Positive response

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def j1939_make_id(prio, pf, da, sa):
    """Build a 29-bit J1939 CAN ID (EDP=0, DP=0)."""
    return (
        ((prio & 0x7) << 26)
        | ((pf & 0xFF) << 16)
        | ((da & 0xFF) << 8)
        | (sa & 0xFF)
    )


def j1939_get_pf(can_id):
    return (can_id >> 16) & 0xFF


def j1939_get_ps(can_id):
    return (can_id >> 8) & 0xFF


def j1939_get_sa(can_id):
    return can_id & 0xFF


def _encode_pgn_le(pgn):
    """Encode a PGN as 3 little-endian bytes."""
    return bytes([pgn & 0xFF, (pgn >> 8) & 0xFF, (pgn >> 16) & 0xFF])


def _is_extended(pkt):
    """Return True if *pkt* carries an extended (29-bit) CAN identifier."""
    # Support both python-can Message objects and scapy CAN frames.
    if hasattr(pkt, "is_extended_id"):
        return pkt.is_extended_id
    if hasattr(pkt, "flags"):
        return bool(pkt.flags & 0x4)  # scapy CAN extended flag
    return False


def _pkt_id(pkt):
    """Extract the CAN identifier from *pkt*."""
    if hasattr(pkt, "arbitration_id"):
        return pkt.arbitration_id
    if hasattr(pkt, "identifier"):
        return pkt.identifier
    raise TypeError("Cannot extract CAN identifier from packet")


def _pkt_data(pkt):
    """Return the data payload as *bytes*."""
    if hasattr(pkt, "data"):
        d = pkt.data
        return bytes(d) if not isinstance(d, bytes) else d
    raise TypeError("Cannot extract CAN data from packet")


# ---------------------------------------------------------------------------
# Per-technique probe / collect helpers
# ---------------------------------------------------------------------------

def _record(found, sa, method, pkt):
    """Append a detection to the *found* accumulator (never overwrite)."""
    found.setdefault(sa, []).append({"method": method, "packet": pkt})


def _scan_addr_claim(sock, found, timeout, send_fn, recv_fn):
    """Technique 1 – broadcast Request for PGN 60928 (Address Claimed)."""
    probe_id = j1939_make_id(6, PF_REQUEST, DA_BROADCAST, SCANNER_SA)
    payload = _encode_pgn_le(PGN_ADDRESS_CLAIMED)
    send_fn(sock, probe_id, payload)
    log.debug("addr_claim: broadcast request sent (CAN-ID=0x%08X)", probe_id)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pkt = recv_fn(sock, max(0, deadline - time.monotonic()))
        if pkt is None:
            continue
        if not _is_extended(pkt):
            continue
        cid = _pkt_id(pkt)
        if j1939_get_pf(cid) == PF_ADDRESS_CLAIMED:
            sa = j1939_get_sa(cid)
            log.debug("addr_claim: response from SA=0x%02X", sa)
            _record(found, sa, "addr_claim", pkt)


def _scan_ecu_id(sock, found, timeout, send_fn, recv_fn):
    """Technique 2 – broadcast Request for PGN 64965 (ECU Identification)."""
    probe_id = j1939_make_id(6, PF_REQUEST, DA_BROADCAST, SCANNER_SA)
    payload = _encode_pgn_le(PGN_ECU_ID)
    send_fn(sock, probe_id, payload)
    log.debug("ecu_id: broadcast request sent (CAN-ID=0x%08X)", probe_id)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pkt = recv_fn(sock, max(0, deadline - time.monotonic()))
        if pkt is None:
            continue
        if not _is_extended(pkt):
            continue
        cid = _pkt_id(pkt)
        data = _pkt_data(pkt)
        if j1939_get_pf(cid) == PF_TP_CM and len(data) >= 8:
            if data[0] == TP_CM_BAM:
                sa = j1939_get_sa(cid)
                log.debug("ecu_id: BAM from SA=0x%02X", sa)
                _record(found, sa, "ecu_id", pkt)


def _scan_unicast(sock, found, timeout_per_da, send_fn, recv_fn,
                  da_range=range(0x00, 0xFE)):
    """Technique 3 – unicast Request for PGN 60928 to each DA."""
    for da in da_range:
        probe_id = j1939_make_id(6, PF_REQUEST, da, SCANNER_SA)
        payload = _encode_pgn_le(PGN_ADDRESS_CLAIMED)
        send_fn(sock, probe_id, payload)
        log.debug("unicast: probing DA=0x%02X", da)

        deadline = time.monotonic() + timeout_per_da
        while time.monotonic() < deadline:
            pkt = recv_fn(sock, max(0, deadline - time.monotonic()))
            if pkt is None:
                continue
            if not _is_extended(pkt):
                continue
            cid = _pkt_id(pkt)
            sa = j1939_get_sa(cid)
            if sa == da:
                log.debug("unicast: response from SA=0x%02X", sa)
                _record(found, sa, "unicast", pkt)


def _scan_rts_probe(sock, found, timeout_per_da, send_fn, recv_fn,
                    da_range=range(0x00, 0xFE)):
    """Technique 4 – TP.CM_RTS addressed to each DA."""
    for da in da_range:
        probe_id = j1939_make_id(7, PF_TP_CM, da, SCANNER_SA)
        # Build a minimal RTS payload
        rts_payload = bytearray(8)
        rts_payload[0] = TP_CM_RTS
        rts_payload[1] = 0x09  # total message size low byte
        rts_payload[2] = 0x00  # total message size high byte
        rts_payload[3] = 0x02  # number of packets
        rts_payload[4] = 0xFF  # max packets per CTS (0xFF = unlimited)
        pgn_bytes = _encode_pgn_le(PGN_ECU_ID)
        rts_payload[5] = pgn_bytes[0]
        rts_payload[6] = pgn_bytes[1]
        rts_payload[7] = pgn_bytes[2]

        send_fn(sock, probe_id, bytes(rts_payload))
        log.debug("rts_probe: probing DA=0x%02X", da)

        deadline = time.monotonic() + timeout_per_da
        while time.monotonic() < deadline:
            pkt = recv_fn(sock, max(0, deadline - time.monotonic()))
            if pkt is None:
                continue
            if not _is_extended(pkt):
                continue
            cid = _pkt_id(pkt)
            data = _pkt_data(pkt)
            if j1939_get_pf(cid) == PF_TP_CM and len(data) >= 1:
                ctrl = data[0]
                if ctrl in (TP_CM_CTS, TP_CM_ABORT):
                    sa = j1939_get_sa(cid)
                    log.debug(
                        "rts_probe: response (ctrl=0x%02X) from SA=0x%02X",
                        ctrl, sa,
                    )
                    _record(found, sa, "rts_probe", pkt)


def _scan_uds(sock, found, timeout_per_da, send_fn, recv_fn,
              da_range=range(0x00, 0xFE)):
    """Technique 5 – UDS Tester Present (PF=0xDA) to each DA."""
    for da in da_range:
        probe_id = j1939_make_id(6, PF_DIAG, da, SCANNER_SA)
        send_fn(sock, probe_id, UDS_TESTER_PRESENT_REQUEST)
        log.debug("uds: probing DA=0x%02X", da)

        deadline = time.monotonic() + timeout_per_da
        while time.monotonic() < deadline:
            pkt = recv_fn(sock, max(0, deadline - time.monotonic()))
            if pkt is None:
                continue
            if not _is_extended(pkt):
                continue
            cid = _pkt_id(pkt)
            sa = j1939_get_sa(cid)
            if sa == da:
                data = _pkt_data(pkt)
                if data == UDS_TESTER_PRESENT_RESPONSE:
                    log.debug("uds: response from SA=0x%02X", sa)
                    _record(found, sa, "uds", pkt)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def j1939_scan(sock, *, force=False, timeout=0.5, timeout_per_da=0.05,
               send_fn=None, recv_fn=None):
    """Scan the CAN bus for J1939 Controller Applications.

    Parameters
    ----------
    sock : CAN socket
        An open CAN socket (e.g. ``python-can`` Bus or scapy CANSocket).
    force : bool
        Reserved for future use (e.g. skip confirmation prompt).
    timeout : float
        Seconds to listen after broadcast probes (addr_claim, ecu_id).
    timeout_per_da : float
        Seconds to listen after each unicast/RTS probe.
    send_fn : callable, optional
        ``send_fn(sock, can_id, data)`` – send an extended CAN frame.
        Defaults to ``sock.send()`` with a ``python-can`` ``Message``.
    recv_fn : callable, optional
        ``recv_fn(sock, timeout)`` – receive one CAN frame or ``None``.
        Defaults to ``sock.recv(timeout)``.

    Returns
    -------
    list of (int, list[dict])
        A list of ``(sa, detections)`` tuples **sorted by SA** in
        ascending order.  Each *detections* entry is a list of dicts
        ``{'method': str, 'packet': pkt}``, one per technique that
        detected the SA.
    """
    if send_fn is None:
        send_fn = _default_send
    if recv_fn is None:
        recv_fn = _default_recv

    # Accumulator: {sa: [{'method': ..., 'packet': ...}, ...]}
    found = {}

    _scan_addr_claim(sock, found, timeout, send_fn, recv_fn)
    _scan_ecu_id(sock, found, timeout, send_fn, recv_fn)
    _scan_unicast(sock, found, timeout_per_da, send_fn, recv_fn)
    _scan_rts_probe(sock, found, timeout_per_da, send_fn, recv_fn)
    _scan_uds(sock, found, timeout_per_da, send_fn, recv_fn)

    # Return as a sorted list of (sa, detections) tuples
    return sorted(found.items(), key=lambda item: item[0])


# ---------------------------------------------------------------------------
# Default send/recv using python-can
# ---------------------------------------------------------------------------

def _default_send(sock, can_id, data):
    """Send an extended CAN frame via a python-can Bus."""
    try:
        from can import Message
    except ImportError:
        raise RuntimeError(
            "python-can is required for the default send/recv. "
            "Install it or provide custom send_fn/recv_fn."
        )
    msg = Message(arbitration_id=can_id, data=data,
                  is_extended_id=True)
    sock.send(msg)


def _default_recv(sock, timeout):
    """Receive a CAN frame via a python-can Bus, or None on timeout."""
    return sock.recv(timeout=timeout)
