#!/usr/bin/env python3
"""
Test suite for J1939 diagnostic routing changes.

Verifies:
1. UDS Physical (PF 0xDA) accepts variable priority and swaps SA/DA.
2. UDS Functional (PF 0xDB) with DA=0xFF works.
3. KWP2000 (PF 0xEF) works for TSA 0xF1-0xFA, physical only, swaps SA/DA.
4. XCP (PF 0xEF) works for TSA 0x3F or 0x5A, physical only, swaps SA/DA.
5. PropA (PF 0xEF) ignores other TSAs for diagnostics.
"""

import time
import unittest
import can

# ---------------------------------------------------------------------------
# CAN Configuration
# ---------------------------------------------------------------------------
INTERFACE = 'candle'
CHANNEL = '0'
BITRATE = 500000

# ---------------------------------------------------------------------------
# J1939 protocol constants
# ---------------------------------------------------------------------------
PF_UDS_PHYS     = 0xDA
PF_UDS_FUNC     = 0xDB
PF_PROPA        = 0xEF

ECU_SA_D = 33  # Body Control (ECUD)

def j1939_make_id(prio, pf, da, sa):
    return ((prio & 0x7) << 26) | (pf << 16) | (da << 8) | sa

def j1939_parse_id(can_id):
    prio = (can_id >> 26) & 0x7
    pf = (can_id >> 16) & 0xFF
    da = (can_id >> 8) & 0xFF
    sa = can_id & 0xFF
    return prio, pf, da, sa

class TestJ1939DiagRouting(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bus = can.interface.Bus(channel=CHANNEL, interface=INTERFACE, bitrate=BITRATE)

    @classmethod
    def tearDownClass(cls):
        cls.bus.shutdown()

    def send_and_wait(self, msg_id, data, timeout=0.5):
        msg = can.Message(arbitration_id=msg_id, data=data, is_extended_id=True)
        self.bus.send(msg)
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            rx_msg = self.bus.recv(timeout=0.1)
            if rx_msg:
                # We expect a response from ECU_SA_D (33) to the requester
                _, _, rx_da, rx_sa = j1939_parse_id(rx_msg.arbitration_id)
                if rx_sa == ECU_SA_D:
                    return rx_msg
        return None

    def test_uds_physical_prio7(self):
        """UDS Physical with Priority 7 should respond with Priority 7."""
        tsa = 0xF9
        req_id = j1939_make_id(7, PF_UDS_PHYS, ECU_SA_D, tsa)
        # Service 0x3E (Tester Present), Subfunction 0x00
        data = [0x02, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        rx = self.send_and_wait(req_id, data)
        self.assertIsNotNone(rx, "No response to UDS physical request")
        
        prio, pf, da, sa = j1939_parse_id(rx.arbitration_id)
        self.assertEqual(prio, 7)
        self.assertEqual(pf, PF_UDS_PHYS)
        self.assertEqual(da, tsa)
        self.assertEqual(sa, ECU_SA_D)
        self.assertEqual(rx.data[1], 0x7E) # Positive response (0x3E + 0x40)

    def test_uds_functional(self):
        """UDS Functional with DA=0xFF should respond physically."""
        tsa = 0xF1
        req_id = j1939_make_id(6, PF_UDS_FUNC, 0xFF, tsa)
        data = [0x02, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        rx = self.send_and_wait(req_id, data)
        self.assertIsNotNone(rx, "No response to UDS functional request")
        
        prio, pf, da, sa = j1939_parse_id(rx.arbitration_id)
        self.assertEqual(pf, PF_UDS_PHYS) # Response is physical
        self.assertEqual(da, tsa)
        self.assertEqual(sa, ECU_SA_D)

    def test_kwp_propa_f1(self):
        """KWP2000 on PropA (PF 0xEF) with TSA 0xF1."""
        tsa = 0xF1
        req_id = j1939_make_id(6, PF_PROPA, ECU_SA_D, tsa)
        # KWP Tester Present (0x3E 0x01)
        data = [0x02, 0x3E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        rx = self.send_and_wait(req_id, data)
        self.assertIsNotNone(rx, "No response to KWP request on PropA")
        
        _, pf, da, sa = j1939_parse_id(rx.arbitration_id)
        self.assertEqual(pf, PF_PROPA)
        self.assertEqual(da, tsa)
        self.assertEqual(sa, ECU_SA_D)
        self.assertEqual(rx.data[1], 0x7E) # Positive response

    def test_xcp_propa_3f(self):
        """XCP on PropA (PF 0xEF) with TSA 0x3F."""
        tsa = 0x3F
        req_id = j1939_make_id(3, PF_PROPA, ECU_SA_D, tsa)
        # XCP Connect (0xFF 0x00)
        data = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        rx = self.send_and_wait(req_id, data)
        self.assertIsNotNone(rx, "No response to XCP request on PropA")
        
        prio, pf, da, sa = j1939_parse_id(rx.arbitration_id)
        self.assertEqual(prio, 3)
        self.assertEqual(pf, PF_PROPA)
        self.assertEqual(da, tsa)
        self.assertEqual(sa, ECU_SA_D)
        self.assertEqual(rx.data[0], 0xFF) # Positive response (Connect)

    def test_propa_ignored_tsa(self):
        """PropA (PF 0xEF) with an invalid TSA (e.g., 0x20) should be ignored."""
        tsa = 0x20
        req_id = j1939_make_id(6, PF_PROPA, ECU_SA_D, tsa)
        data = [0x02, 0x3E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        rx = self.send_and_wait(req_id, data, timeout=0.3)
        self.assertIsNone(rx, "Should not have received a response for invalid TSA on PropA")

if __name__ == '__main__':
    unittest.main()
