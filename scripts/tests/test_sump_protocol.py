#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION. ALL RIGHTS RESERVED.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Unit tests for RAMN SUMP / OLS logic analyzer protocol.

Tests validate the SUMP protocol constants, command encoding, metadata format,
sample data format, and configuration logic without requiring hardware.
Based on the pytest pattern introduced in the J1939 branch.
"""

import struct
import unittest


# ---- SUMP Protocol Constants (mirroring ramn_sump.h) ----

# Short commands (1 byte, no parameters)
SUMP_RESET  = 0x00
SUMP_RUN    = 0x01
SUMP_ID     = 0x02
SUMP_DESC   = 0x04
SUMP_XON    = 0x11
SUMP_XOFF   = 0x13

# Long commands (1 byte command + 4 bytes parameters)
SUMP_DIV          = 0x80
SUMP_CNT          = 0x81
SUMP_FLAGS        = 0x82
SUMP_TRIG_1       = 0xC0
SUMP_TRIG_VALS_1  = 0xC1
SUMP_TRIG_2       = 0xC4
SUMP_TRIG_VALS_2  = 0xC5
SUMP_TRIG_3       = 0xC8
SUMP_TRIG_VALS_3  = 0xC9
SUMP_TRIG_4       = 0xCC
SUMP_TRIG_VALS_4  = 0xCD

# Metadata tags
SUMP_META_NAME         = 0x01
SUMP_META_SAMPLE_MEM   = 0x21
SUMP_META_SAMPLE_RATE  = 0x23
SUMP_META_NUM_PROBES   = 0x40
SUMP_META_PROTO_VERS   = 0x41
SUMP_META_END          = 0x00

# Configuration from ramn_sump.h
SUMP_SAMPLE_BUFFER_SIZE = 4096
SUMP_SAMPLE_BUFFER_MASK = SUMP_SAMPLE_BUFFER_SIZE - 1
SUMP_MAX_SAMPLE_RATE    = 1000000
SUMP_NUM_PROBES         = 3
SUMP_EXIT_CHAR          = 0x1B
SUMP_DEVICE_NAME        = "RAMN"

# CAN pin mapping
SUMP_PIN_TX_BIT = 0  # Channel 0: CAN TX (PB9)
SUMP_PIN_RX_BIT = 1  # Channel 1: CAN RX (PB8)
SUMP_PIN_TRIG_BIT = 2  # Channel 2: Trigger marker

# Sample bit constants
SUMP_BIT_TX   = 0x01
SUMP_BIT_RX   = 0x02
SUMP_BIT_TRIG = 0x04

# Special value indicating no trigger recorded
SUMP_NO_TRIGGER = 0xFFFFFFFF


# ---- Mock SUMP State Machine ----

class MockSUMPDevice:
    """
    A pure-Python mock of the RAMN SUMP state machine for testing.
    Mirrors the logic in ramn_sump.c without hardware dependencies.
    Uses a circular buffer with trigger position tracking.
    """

    STATE_IDLE    = 0
    STATE_ARMED   = 1
    STATE_TRIGGED = 2

    def __init__(self):
        self.state = self.STATE_IDLE
        self.divider = 0
        self.read_count = SUMP_SAMPLE_BUFFER_SIZE
        self.delay_count = SUMP_SAMPLE_BUFFER_SIZE
        self.trigger_mask = [0, 0, 0, 0]
        self.trigger_values = [0, 0, 0, 0]
        self.flags = 0
        self.cmd_pending = 0
        self.cmd_buf = bytearray(4)
        self.cmd_buf_idx = 0
        self.output = bytearray()
        self.exited = False
        # Circular sample buffer (simulates bitbang capture)
        self.samples = bytearray(SUMP_SAMPLE_BUFFER_SIZE)
        self.sample_count = 0       # Total samples written (linear counter)
        self.write_index = 0        # Circular buffer write position
        self.trigger_index = SUMP_NO_TRIGGER  # Linear index of trigger

    def record_sample(self, tx_high, rx_high):
        """Record a TX+RX sample into the circular sample buffer."""
        sample = 0
        if tx_high:
            sample |= SUMP_BIT_TX
        if rx_high:
            sample |= SUMP_BIT_RX
        self.samples[self.write_index] = sample
        self.write_index = (self.write_index + 1) & SUMP_SAMPLE_BUFFER_MASK
        self.sample_count += 1

    def mark_trigger(self):
        """Mark the current position as the trigger point.
        Sets the TRIG bit on the most recently written sample."""
        self.trigger_index = self.sample_count
        prev = (self.write_index - 1) & SUMP_SAMPLE_BUFFER_MASK
        self.samples[prev] |= SUMP_BIT_TRIG

    def reset_capture(self):
        """Reset the sample buffer before a new bitbang operation."""
        self.sample_count = 0
        self.write_index = 0
        self.trigger_index = SUMP_NO_TRIGGER

    def send_captured_samples(self):
        """Send pre-recorded samples windowed around the trigger point."""
        total_written = self.sample_count
        trig_idx = self.trigger_index

        available = min(total_written, SUMP_SAMPLE_BUFFER_SIZE)

        read_count = self.read_count
        if read_count > SUMP_SAMPLE_BUFFER_SIZE:
            read_count = SUMP_SAMPLE_BUFFER_SIZE
        if read_count == 0:
            read_count = SUMP_SAMPLE_BUFFER_SIZE

        if available == 0:
            # No samples: send all-recessive (TX+RX high, no trigger)
            for _ in range(read_count):
                self.output.extend(bytes([SUMP_BIT_TX | SUMP_BIT_RX, 0, 0, 0]))
            return

        delay_count = self.delay_count
        if delay_count > read_count:
            delay_count = read_count

        if trig_idx != SUMP_NO_TRIGGER and trig_idx <= total_written:
            end_linear = trig_idx + delay_count
            if end_linear > total_written:
                end_linear = total_written
            start_linear = (end_linear - read_count) if end_linear > read_count else 0
        else:
            end_linear = total_written
            start_linear = (total_written - read_count) if total_written > read_count else 0

        # Don't read overwritten samples
        if total_written > SUMP_SAMPLE_BUFFER_SIZE:
            oldest_available = total_written - SUMP_SAMPLE_BUFFER_SIZE
            if start_linear < oldest_available:
                start_linear = oldest_available

        send_count = end_linear - start_linear
        if send_count > read_count:
            send_count = read_count

        # Send newest-to-oldest
        idx = end_linear
        sent = 0
        while sent < send_count and idx > start_linear:
            idx -= 1
            circ_idx = idx & SUMP_SAMPLE_BUFFER_MASK
            s = self.samples[circ_idx]
            self.output.extend(bytes([s, 0, 0, 0]))
            sent += 1

        # Pad with idle samples if needed
        while sent < read_count:
            self.output.extend(bytes([SUMP_BIT_TX | SUMP_BIT_RX, 0, 0, 0]))
            sent += 1

    @staticmethod
    def is_sump_probe(buffer, length):
        """Check if a received buffer indicates PulseView is connecting.
        Mirrors RAMN_SUMP_IsSUMPProbe logic."""
        if length == 1 and buffer[0] == SUMP_ID:
            return True
        return False

    def send_byte(self, b):
        """Send a single byte to the device output."""
        self.output.append(b)

    def send_bytes(self, data):
        """Send bytes to the device output."""
        self.output.extend(data)

    def send_id(self):
        self.send_bytes(b"1ALS")

    def send_metadata(self):
        # Device name
        self.send_byte(SUMP_META_NAME)
        self.send_bytes(SUMP_DEVICE_NAME.encode("ascii") + b"\x00")
        # Sample memory (big-endian)
        self.send_byte(SUMP_META_SAMPLE_MEM)
        self.send_bytes(struct.pack(">I", SUMP_SAMPLE_BUFFER_SIZE))
        # Max sample rate (big-endian)
        self.send_byte(SUMP_META_SAMPLE_RATE)
        self.send_bytes(struct.pack(">I", SUMP_MAX_SAMPLE_RATE))
        # Number of probes
        self.send_byte(SUMP_META_NUM_PROBES)
        self.send_byte(SUMP_NUM_PROBES)
        # Protocol version
        self.send_byte(SUMP_META_PROTO_VERS)
        self.send_byte(2)
        # End
        self.send_byte(SUMP_META_END)

    def reset(self):
        self.state = self.STATE_IDLE
        self.divider = 0
        self.read_count = SUMP_SAMPLE_BUFFER_SIZE
        self.delay_count = SUMP_SAMPLE_BUFFER_SIZE
        self.flags = 0
        for i in range(4):
            self.trigger_mask[i] = 0
            self.trigger_values[i] = 0

    def is_long_command(self, cmd):
        return cmd >= 0x80

    def process_command(self, cmd, params=None):
        if cmd == SUMP_RESET:
            self.reset()
        elif cmd == SUMP_ID:
            self.send_id()
        elif cmd == SUMP_DESC:
            self.send_metadata()
        elif cmd == SUMP_XON or cmd == SUMP_XOFF:
            pass  # Flow control: not implemented
        elif cmd == SUMP_RUN:
            self.send_captured_samples()  # Return pre-recorded bitbang samples
        elif cmd == SUMP_DIV:
            self.divider = params[0] | (params[1] << 8) | (params[2] << 16)
        elif cmd == SUMP_CNT:
            self.read_count = ((params[1] << 8) | params[0]) + 1
            self.read_count <<= 2
            self.delay_count = (params[3] << 8) | params[2]
            self.delay_count <<= 2
        elif cmd == SUMP_FLAGS:
            self.flags = params[0]
        elif (cmd & 0xF0) == 0xC0:
            stage = (cmd & 0x0C) >> 2
            val = params[0] | (params[1] << 8) | (params[2] << 16) | (params[3] << 24)
            if (cmd & 0x01) == 0:
                if stage < 4:
                    self.trigger_mask[stage] = val
            else:
                if stage < 4:
                    self.trigger_values[stage] = val

    def process_byte(self, byte):
        """Process a single input byte. Returns True if exiting SUMP mode."""
        if byte == SUMP_EXIT_CHAR:
            self.exited = True
            return True

        if self.cmd_pending != 0:
            self.cmd_buf[self.cmd_buf_idx] = byte
            self.cmd_buf_idx += 1
            if self.cmd_buf_idx >= 4:
                self.process_command(self.cmd_pending, self.cmd_buf)
                self.cmd_pending = 0
                self.cmd_buf_idx = 0
            return False

        if self.is_long_command(byte):
            self.cmd_pending = byte
            self.cmd_buf_idx = 0
            return False
        else:
            self.process_command(byte)
            return False


# ---- Helper Functions ----

def encode_sump_div(divider):
    """Encode a SUMP_DIV command with a 24-bit divider value."""
    return bytes([SUMP_DIV, divider & 0xFF, (divider >> 8) & 0xFF, (divider >> 16) & 0xFF, 0])


def encode_sump_cnt(read_count, delay_count):
    """Encode a SUMP_CNT command with read and delay counts.

    read_count and delay_count are the final desired values.
    The encoding reverses the firmware's <<2 and +1 operations.
    """
    # Reverse: read_count = ((hi << 8 | lo) + 1) << 2
    # So: (hi << 8 | lo) = (read_count >> 2) - 1
    rc_raw = (read_count >> 2) - 1
    dc_raw = delay_count >> 2
    return bytes([
        SUMP_CNT,
        rc_raw & 0xFF, (rc_raw >> 8) & 0xFF,
        dc_raw & 0xFF, (dc_raw >> 8) & 0xFF,
    ])


def encode_sump_flags(flags_byte):
    """Encode a SUMP_FLAGS command."""
    return bytes([SUMP_FLAGS, flags_byte, 0, 0, 0])


def encode_sump_trigger(stage, mask, value):
    """Encode trigger mask and value commands for a given stage."""
    mask_cmd = SUMP_TRIG_1 + (stage * 4)
    val_cmd = SUMP_TRIG_VALS_1 + (stage * 4)
    mask_bytes = bytes([mask_cmd]) + struct.pack("<I", mask)
    val_bytes = bytes([val_cmd]) + struct.pack("<I", value)
    return mask_bytes + val_bytes


def make_sample(tx_high, rx_high, trig=False):
    """Create a sample byte matching the RAMN pin mapping."""
    sample = 0
    if tx_high:
        sample |= (1 << SUMP_PIN_TX_BIT)
    if rx_high:
        sample |= (1 << SUMP_PIN_RX_BIT)
    if trig:
        sample |= (1 << SUMP_PIN_TRIG_BIT)
    return sample


# ===========================================================================
# Test Classes
# ===========================================================================

class TestSUMPConstants(unittest.TestCase):
    """Verify SUMP protocol constant values match the specification."""

    def test_short_command_values(self):
        self.assertEqual(SUMP_RESET, 0x00)
        self.assertEqual(SUMP_RUN, 0x01)
        self.assertEqual(SUMP_ID, 0x02)
        self.assertEqual(SUMP_DESC, 0x04)
        self.assertEqual(SUMP_XON, 0x11)
        self.assertEqual(SUMP_XOFF, 0x13)

    def test_long_command_values(self):
        self.assertEqual(SUMP_DIV, 0x80)
        self.assertEqual(SUMP_CNT, 0x81)
        self.assertEqual(SUMP_FLAGS, 0x82)

    def test_trigger_command_values(self):
        self.assertEqual(SUMP_TRIG_1, 0xC0)
        self.assertEqual(SUMP_TRIG_VALS_1, 0xC1)
        self.assertEqual(SUMP_TRIG_2, 0xC4)
        self.assertEqual(SUMP_TRIG_VALS_2, 0xC5)
        self.assertEqual(SUMP_TRIG_3, 0xC8)
        self.assertEqual(SUMP_TRIG_VALS_3, 0xC9)
        self.assertEqual(SUMP_TRIG_4, 0xCC)
        self.assertEqual(SUMP_TRIG_VALS_4, 0xCD)

    def test_trigger_stage_encoding(self):
        """Trigger commands encode the stage in bits 2-3."""
        for stage in range(4):
            mask_cmd = SUMP_TRIG_1 + (stage * 4)
            val_cmd = SUMP_TRIG_VALS_1 + (stage * 4)
            self.assertEqual((mask_cmd & 0x0C) >> 2, stage)
            self.assertEqual((val_cmd & 0x0C) >> 2, stage)
            # Mask commands are even, value commands are odd
            self.assertEqual(mask_cmd & 0x01, 0)
            self.assertEqual(val_cmd & 0x01, 1)

    def test_long_command_detection(self):
        """Commands >= 0x80 are long commands (need 4 parameter bytes)."""
        dev = MockSUMPDevice()
        for cmd in [SUMP_DIV, SUMP_CNT, SUMP_FLAGS, SUMP_TRIG_1, SUMP_TRIG_VALS_4]:
            self.assertTrue(dev.is_long_command(cmd), f"0x{cmd:02x} should be long")
        for cmd in [SUMP_RESET, SUMP_RUN, SUMP_ID, SUMP_DESC, SUMP_XON, SUMP_XOFF]:
            self.assertFalse(dev.is_long_command(cmd), f"0x{cmd:02x} should be short")

    def test_metadata_tags(self):
        self.assertEqual(SUMP_META_NAME, 0x01)
        self.assertEqual(SUMP_META_SAMPLE_MEM, 0x21)
        self.assertEqual(SUMP_META_SAMPLE_RATE, 0x23)
        self.assertEqual(SUMP_META_NUM_PROBES, 0x40)
        self.assertEqual(SUMP_META_PROTO_VERS, 0x41)
        self.assertEqual(SUMP_META_END, 0x00)

    def test_device_configuration(self):
        self.assertEqual(SUMP_SAMPLE_BUFFER_SIZE, 4096)
        self.assertEqual(SUMP_SAMPLE_BUFFER_MASK, 4095)
        self.assertEqual(SUMP_MAX_SAMPLE_RATE, 1000000)
        self.assertEqual(SUMP_NUM_PROBES, 3)
        self.assertEqual(SUMP_EXIT_CHAR, 0x1B)
        self.assertEqual(SUMP_DEVICE_NAME, "RAMN")
        self.assertEqual(SUMP_BIT_TX, 0x01)
        self.assertEqual(SUMP_BIT_RX, 0x02)
        self.assertEqual(SUMP_BIT_TRIG, 0x04)
        self.assertEqual(SUMP_NO_TRIGGER, 0xFFFFFFFF)


class TestSUMPIDResponse(unittest.TestCase):
    """Test the SUMP ID response."""

    def test_id_query_response(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_ID)
        self.assertEqual(bytes(dev.output), b"1ALS")

    def test_id_is_exactly_4_bytes(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_ID)
        self.assertEqual(len(dev.output), 4)


class TestSUMPMetadata(unittest.TestCase):
    """Test the SUMP metadata (descriptor) response."""

    def test_metadata_starts_with_device_name(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_DESC)
        output = bytes(dev.output)
        # First byte should be the name tag
        self.assertEqual(output[0], SUMP_META_NAME)
        # Followed by null-terminated device name
        name_end = output.index(0, 1)
        name = output[1:name_end].decode("ascii")
        self.assertEqual(name, SUMP_DEVICE_NAME)

    def test_metadata_contains_sample_memory(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_DESC)
        output = bytes(dev.output)
        idx = output.index(SUMP_META_SAMPLE_MEM)
        mem = struct.unpack(">I", output[idx+1:idx+5])[0]
        self.assertEqual(mem, SUMP_SAMPLE_BUFFER_SIZE)

    def test_metadata_contains_sample_rate(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_DESC)
        output = bytes(dev.output)
        idx = output.index(SUMP_META_SAMPLE_RATE)
        rate = struct.unpack(">I", output[idx+1:idx+5])[0]
        self.assertEqual(rate, SUMP_MAX_SAMPLE_RATE)

    def test_metadata_contains_num_probes(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_DESC)
        output = bytes(dev.output)
        # Parse metadata sequentially to find NUM_PROBES tag
        found = False
        i = 0
        while i < len(output):
            tag = output[i]
            if tag == SUMP_META_END:
                break
            elif tag == SUMP_META_NAME:
                # Skip null-terminated string
                i += 1
                while i < len(output) and output[i] != 0:
                    i += 1
                i += 1  # skip null
            elif tag in (SUMP_META_SAMPLE_MEM, SUMP_META_SAMPLE_RATE):
                i += 5  # tag + 4 bytes
            elif tag == SUMP_META_NUM_PROBES:
                self.assertEqual(output[i+1], SUMP_NUM_PROBES)
                found = True
                i += 2
            elif tag == SUMP_META_PROTO_VERS:
                i += 2
            else:
                i += 1
        self.assertTrue(found, "NUM_PROBES tag not found in metadata")

    def test_metadata_contains_protocol_version(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_DESC)
        output = bytes(dev.output)
        # Parse metadata sequentially to find PROTO_VERS tag
        found = False
        i = 0
        while i < len(output):
            tag = output[i]
            if tag == SUMP_META_END:
                break
            elif tag == SUMP_META_NAME:
                i += 1
                while i < len(output) and output[i] != 0:
                    i += 1
                i += 1
            elif tag in (SUMP_META_SAMPLE_MEM, SUMP_META_SAMPLE_RATE):
                i += 5
            elif tag == SUMP_META_NUM_PROBES:
                i += 2
            elif tag == SUMP_META_PROTO_VERS:
                self.assertEqual(output[i+1], 2)
                found = True
                i += 2
            else:
                i += 1
        self.assertTrue(found, "PROTO_VERS tag not found in metadata")

    def test_metadata_ends_with_terminator(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_DESC)
        output = bytes(dev.output)
        self.assertEqual(output[-1], SUMP_META_END)


class TestSUMPReset(unittest.TestCase):
    """Test the SUMP reset command."""

    def test_reset_clears_divider(self):
        dev = MockSUMPDevice()
        # Set a divider first
        for b in encode_sump_div(1000):
            dev.process_byte(b)
        self.assertEqual(dev.divider, 1000)
        # Reset should clear it
        dev.process_byte(SUMP_RESET)
        self.assertEqual(dev.divider, 0)

    def test_reset_clears_trigger_masks(self):
        dev = MockSUMPDevice()
        for b in encode_sump_trigger(0, 0xFF, 0x01):
            dev.process_byte(b)
        self.assertEqual(dev.trigger_mask[0], 0xFF)
        dev.process_byte(SUMP_RESET)
        self.assertEqual(dev.trigger_mask[0], 0)
        self.assertEqual(dev.trigger_values[0], 0)

    def test_reset_restores_default_counts(self):
        dev = MockSUMPDevice()
        for b in encode_sump_cnt(256, 128):
            dev.process_byte(b)
        dev.process_byte(SUMP_RESET)
        self.assertEqual(dev.read_count, SUMP_SAMPLE_BUFFER_SIZE)
        self.assertEqual(dev.delay_count, SUMP_SAMPLE_BUFFER_SIZE)

    def test_reset_does_not_produce_output(self):
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_RESET)
        self.assertEqual(len(dev.output), 0)


class TestSUMPDivider(unittest.TestCase):
    """Test the SUMP_DIV sample rate divider command."""

    def test_divider_zero(self):
        dev = MockSUMPDevice()
        for b in encode_sump_div(0):
            dev.process_byte(b)
        self.assertEqual(dev.divider, 0)

    def test_divider_small(self):
        dev = MockSUMPDevice()
        for b in encode_sump_div(79):
            dev.process_byte(b)
        self.assertEqual(dev.divider, 79)

    def test_divider_large(self):
        dev = MockSUMPDevice()
        for b in encode_sump_div(0xFFFFFF):
            dev.process_byte(b)
        self.assertEqual(dev.divider, 0xFFFFFF)

    def test_divider_only_uses_24_bits(self):
        """The 4th parameter byte is unused for DIV."""
        dev = MockSUMPDevice()
        for b in [SUMP_DIV, 0x56, 0x34, 0x12, 0xFF]:
            dev.process_byte(b)
        self.assertEqual(dev.divider, 0x123456)


class TestSUMPCounts(unittest.TestCase):
    """Test the SUMP_CNT read/delay count command."""

    def test_basic_counts(self):
        dev = MockSUMPDevice()
        # read_count_raw = 63 => (63 + 1) << 2 = 256
        # delay_count_raw = 32 => 32 << 2 = 128
        for b in [SUMP_CNT, 63, 0, 32, 0]:
            dev.process_byte(b)
        self.assertEqual(dev.read_count, 256)
        self.assertEqual(dev.delay_count, 128)

    def test_minimum_counts(self):
        dev = MockSUMPDevice()
        # read_count_raw = 0 => (0 + 1) << 2 = 4
        # delay_count_raw = 0 => 0 << 2 = 0
        for b in [SUMP_CNT, 0, 0, 0, 0]:
            dev.process_byte(b)
        self.assertEqual(dev.read_count, 4)
        self.assertEqual(dev.delay_count, 0)

    def test_maximum_counts(self):
        dev = MockSUMPDevice()
        # read_count_raw = 0xFFFF => (0xFFFF + 1) << 2 = 0x40000
        for b in [SUMP_CNT, 0xFF, 0xFF, 0xFF, 0xFF]:
            dev.process_byte(b)
        self.assertEqual(dev.read_count, 0x40000)
        self.assertEqual(dev.delay_count, 0x3FFFC)


class TestSUMPFlags(unittest.TestCase):
    """Test the SUMP_FLAGS channel configuration command."""

    def test_flags_set(self):
        dev = MockSUMPDevice()
        for b in encode_sump_flags(0x0C):
            dev.process_byte(b)
        self.assertEqual(dev.flags, 0x0C)

    def test_flags_zero(self):
        dev = MockSUMPDevice()
        for b in encode_sump_flags(0):
            dev.process_byte(b)
        self.assertEqual(dev.flags, 0)

    def test_flags_all_bits(self):
        dev = MockSUMPDevice()
        for b in encode_sump_flags(0xFF):
            dev.process_byte(b)
        self.assertEqual(dev.flags, 0xFF)


class TestSUMPTriggers(unittest.TestCase):
    """Test SUMP trigger mask and value configuration."""

    def test_stage0_trigger(self):
        dev = MockSUMPDevice()
        for b in encode_sump_trigger(0, 0x03, 0x02):
            dev.process_byte(b)
        self.assertEqual(dev.trigger_mask[0], 0x03)
        self.assertEqual(dev.trigger_values[0], 0x02)

    def test_stage1_trigger(self):
        dev = MockSUMPDevice()
        for b in encode_sump_trigger(1, 0xFF00, 0xAB00):
            dev.process_byte(b)
        self.assertEqual(dev.trigger_mask[1], 0xFF00)
        self.assertEqual(dev.trigger_values[1], 0xAB00)

    def test_all_four_stages(self):
        dev = MockSUMPDevice()
        for stage in range(4):
            mask = (stage + 1) * 0x11
            val = (stage + 1) * 0x22
            for b in encode_sump_trigger(stage, mask, val):
                dev.process_byte(b)
        for stage in range(4):
            self.assertEqual(dev.trigger_mask[stage], (stage + 1) * 0x11)
            self.assertEqual(dev.trigger_values[stage], (stage + 1) * 0x22)

    def test_trigger_32bit_values(self):
        dev = MockSUMPDevice()
        for b in encode_sump_trigger(0, 0xDEADBEEF, 0xCAFEBABE):
            dev.process_byte(b)
        self.assertEqual(dev.trigger_mask[0], 0xDEADBEEF)
        self.assertEqual(dev.trigger_values[0], 0xCAFEBABE)

    def test_trigger_match_logic(self):
        """Verify the trigger matching formula: (sample ^ value) & mask == 0."""
        mask = 0x03  # Check bits 0 and 1
        value = 0x02  # Expect: TX=0, RX=1

        # Sample matches: TX=0, RX=1 => sample=0x02
        sample = make_sample(tx_high=False, rx_high=True)
        self.assertEqual((sample ^ value) & mask, 0, "Should trigger")

        # Sample does not match: TX=1, RX=1 => sample=0x03
        sample = make_sample(tx_high=True, rx_high=True)
        self.assertNotEqual((sample ^ value) & mask, 0, "Should not trigger")

        # Sample does not match: TX=0, RX=0 => sample=0x00
        sample = make_sample(tx_high=False, rx_high=False)
        self.assertNotEqual((sample ^ value) & mask, 0, "Should not trigger")


class TestSUMPExitMechanism(unittest.TestCase):
    """Test SUMP mode exit via ESC character."""

    def test_esc_exits_sump(self):
        dev = MockSUMPDevice()
        result = dev.process_byte(SUMP_EXIT_CHAR)
        self.assertTrue(result)
        self.assertTrue(dev.exited)

    def test_non_esc_does_not_exit(self):
        dev = MockSUMPDevice()
        for cmd in [SUMP_RESET, SUMP_ID, SUMP_RUN, SUMP_XON, 0x42]:
            result = dev.process_byte(cmd)
            self.assertFalse(result, f"Byte 0x{cmd:02x} should not exit")

    def test_esc_during_long_command_exits(self):
        """ESC should exit even in the middle of collecting long command params."""
        dev = MockSUMPDevice()
        dev.process_byte(SUMP_DIV)  # Start long command
        dev.process_byte(0x10)      # First param byte
        result = dev.process_byte(SUMP_EXIT_CHAR)  # ESC interrupts
        self.assertTrue(result)

    def test_esc_value_is_0x1b(self):
        self.assertEqual(SUMP_EXIT_CHAR, 0x1B)
        self.assertEqual(SUMP_EXIT_CHAR, 27)


class TestSUMPSampleFormat(unittest.TestCase):
    """Test the sample data format for CAN TX/RX pin mapping."""

    def test_both_pins_high(self):
        sample = make_sample(tx_high=True, rx_high=True)
        self.assertEqual(sample, 0x03)

    def test_both_pins_low(self):
        sample = make_sample(tx_high=False, rx_high=False)
        self.assertEqual(sample, 0x00)

    def test_tx_high_rx_low(self):
        sample = make_sample(tx_high=True, rx_high=False)
        self.assertEqual(sample, 0x01)

    def test_tx_low_rx_high(self):
        sample = make_sample(tx_high=False, rx_high=True)
        self.assertEqual(sample, 0x02)

    def test_sample_fits_in_byte(self):
        """All valid samples must fit in one byte."""
        for tx in [False, True]:
            for rx in [False, True]:
                sample = make_sample(tx, rx)
                self.assertLessEqual(sample, 0xFF)
                self.assertGreaterEqual(sample, 0x00)

    def test_sample_uses_only_2_bits(self):
        """Only bits 0 and 1 should ever be set for normal samples (no trigger)."""
        for tx in [False, True]:
            for rx in [False, True]:
                sample = make_sample(tx, rx)
                self.assertEqual(sample & 0xF8, 0, "Only bits 0-2 should be used")

    def test_trigger_bit_is_bit2(self):
        """The trigger marker uses bit 2 (0x04)."""
        sample = make_sample(True, True, trig=True)
        self.assertEqual(sample & SUMP_BIT_TRIG, SUMP_BIT_TRIG)
        self.assertEqual(sample, 0x07)  # TX + RX + TRIG

    def test_sump_4byte_sample_format(self):
        """SUMP sends each sample as 4 bytes: [sample, 0, 0, 0]."""
        sample = make_sample(tx_high=True, rx_high=True)
        sump_bytes = bytes([sample, 0, 0, 0])
        self.assertEqual(len(sump_bytes), 4)
        self.assertEqual(sump_bytes[0], 0x03)
        self.assertEqual(sump_bytes[1], 0)
        self.assertEqual(sump_bytes[2], 0)
        self.assertEqual(sump_bytes[3], 0)

    def test_3_channel_sample(self):
        """3-channel samples: TX(bit0) + RX(bit1) + TRIG(bit2)."""
        # All channels on
        s = make_sample(True, True, True)
        self.assertEqual(s, 0x07)
        # Only trigger
        s = make_sample(False, False, True)
        self.assertEqual(s, 0x04)


class TestSUMPCommandSequence(unittest.TestCase):
    """Test complete SUMP command sequences as PulseView would send them."""

    def test_typical_pulseview_handshake(self):
        """Simulate a typical PulseView handshake sequence."""
        dev = MockSUMPDevice()

        # 1. Reset (sent multiple times by PulseView)
        for _ in range(5):
            dev.process_byte(SUMP_RESET)

        # 2. Query ID
        dev.process_byte(SUMP_ID)
        self.assertEqual(bytes(dev.output), b"1ALS")

        dev.output.clear()

        # 3. Query metadata
        dev.process_byte(SUMP_DESC)
        self.assertGreater(len(dev.output), 0)
        # Should end with 0x00 terminator
        self.assertEqual(dev.output[-1], 0x00)

    def test_configure_and_reset_sequence(self):
        """Set configuration then reset should clear everything."""
        dev = MockSUMPDevice()

        # Configure
        for b in encode_sump_div(79):
            dev.process_byte(b)
        for b in encode_sump_cnt(256, 128):
            dev.process_byte(b)
        for b in encode_sump_trigger(0, 0x02, 0x02):
            dev.process_byte(b)

        self.assertEqual(dev.divider, 79)
        self.assertEqual(dev.trigger_mask[0], 0x02)

        # Reset
        dev.process_byte(SUMP_RESET)
        self.assertEqual(dev.divider, 0)
        self.assertEqual(dev.trigger_mask[0], 0)
        self.assertEqual(dev.read_count, SUMP_SAMPLE_BUFFER_SIZE)

    def test_multiple_resets_are_safe(self):
        """Sending many resets (as PulseView does) should be safe."""
        dev = MockSUMPDevice()
        for _ in range(20):
            dev.process_byte(SUMP_RESET)
        # Should still respond to ID query
        dev.process_byte(SUMP_ID)
        self.assertEqual(bytes(dev.output), b"1ALS")


class TestSUMPModeGuard(unittest.TestCase):
    """Test that ENABLE_SUMP_OLS requires ENABLE_BITBANG and ENABLE_CDC."""

    def test_sump_requires_bitbang(self):
        """Verify the config.h guard: ENABLE_SUMP_OLS requires ENABLE_BITBANG."""
        # This is a compile-time check. We verify the guard text exists in the header.
        import os
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "..",
            "firmware", "RAMNV1", "Core", "Inc", "ramn_config.h"
        )
        if os.path.exists(config_path):
            with open(config_path) as f:
                content = f.read()
            self.assertIn("ENABLE_SUMP_OLS", content)
            self.assertIn("#if defined(ENABLE_SUMP_OLS) && !defined(ENABLE_BITBANG)", content)
            self.assertIn("#if defined(ENABLE_SUMP_OLS) && !defined(ENABLE_CDC)", content)

    def test_sump_header_guard(self):
        """Verify ramn_sump.h has the correct guard and new API functions."""
        import os
        header_path = os.path.join(
            os.path.dirname(__file__), "..", "..",
            "firmware", "RAMNV1", "Core", "Inc", "ramn_sump.h"
        )
        if os.path.exists(header_path):
            with open(header_path) as f:
                content = f.read()
            self.assertIn("#if defined(ENABLE_SUMP_OLS) && defined(ENABLE_BITBANG)", content)
            self.assertIn("SUMP_RESET", content)
            self.assertIn("SUMP_RUN", content)
            self.assertIn("RAMN_SUMP_Enter", content)
            self.assertIn("RAMN_SUMP_ProcessByte", content)
            self.assertIn("RAMN_SUMP_IsSUMPProbe", content)
            self.assertIn("RAMN_SUMP_RecordSample", content)
            self.assertIn("RAMN_SUMP_ResetCapture", content)
            self.assertIn("RAMN_SUMP_Samples", content)
            self.assertIn("RAMN_SUMP_SampleCount", content)
            self.assertIn("RAMN_SUMP_WriteIndex", content)
            self.assertIn("RAMN_SUMP_TriggerIndex", content)
            self.assertIn("RAMN_SUMP_MarkTrigger", content)
            self.assertIn("SUMP_BIT_TX", content)
            self.assertIn("SUMP_BIT_RX", content)
            self.assertIn("SUMP_BIT_TRIG", content)
            self.assertIn("SUMP_NO_TRIGGER", content)
            self.assertIn("SUMP_SAMPLE_BUFFER_MASK", content)


class TestSUMPFlowControl(unittest.TestCase):
    """Test flow control commands (XON/XOFF) are accepted but ignored."""

    def test_xon_accepted(self):
        dev = MockSUMPDevice()
        result = dev.process_byte(SUMP_XON)
        self.assertFalse(result)
        self.assertEqual(len(dev.output), 0)

    def test_xoff_accepted(self):
        dev = MockSUMPDevice()
        result = dev.process_byte(SUMP_XOFF)
        self.assertFalse(result)
        self.assertEqual(len(dev.output), 0)


class TestSUMPEncodingHelpers(unittest.TestCase):
    """Test the test helper encoding functions themselves."""

    def test_encode_div_length(self):
        data = encode_sump_div(100)
        self.assertEqual(len(data), 5)
        self.assertEqual(data[0], SUMP_DIV)

    def test_encode_cnt_length(self):
        data = encode_sump_cnt(256, 128)
        self.assertEqual(len(data), 5)
        self.assertEqual(data[0], SUMP_CNT)

    def test_encode_flags_length(self):
        data = encode_sump_flags(0)
        self.assertEqual(len(data), 5)
        self.assertEqual(data[0], SUMP_FLAGS)

    def test_encode_trigger_length(self):
        data = encode_sump_trigger(0, 0xFF, 0x01)
        # Two commands: mask (5 bytes) + value (5 bytes)
        self.assertEqual(len(data), 10)

    def test_make_sample_symmetry(self):
        """Each pin independently maps to its own bit."""
        self.assertNotEqual(
            make_sample(True, False),
            make_sample(False, True)
        )


class TestSUMPAutoDetect(unittest.TestCase):
    """Test SUMP auto-detection from PulseView connection."""

    def test_single_byte_0x02_is_sump_probe(self):
        """A single-byte buffer containing 0x02 (SUMP_ID) is a SUMP probe."""
        self.assertTrue(MockSUMPDevice.is_sump_probe(bytes([SUMP_ID]), 1))

    def test_multi_byte_buffer_is_not_sump_probe(self):
        """A multi-byte buffer is not a SUMP probe."""
        self.assertFalse(MockSUMPDevice.is_sump_probe(bytes([SUMP_ID, 0x00]), 2))

    def test_other_single_bytes_are_not_sump_probe(self):
        """Other single-byte commands are not SUMP probes."""
        for b in [0x00, 0x01, 0x03, 0x04, 0x11, 0x13, 0x80, 0xFF]:
            self.assertFalse(MockSUMPDevice.is_sump_probe(bytes([b]), 1),
                             f"Byte 0x{b:02x} should not be a SUMP probe")

    def test_sump_id_value_matches(self):
        """SUMP_ID constant is 0x02."""
        self.assertEqual(SUMP_ID, 0x02)

    def test_autodetect_responds_with_1als(self):
        """When auto-detect fires, device responds with '1ALS' and enters SUMP mode."""
        dev = MockSUMPDevice()
        if MockSUMPDevice.is_sump_probe(bytes([SUMP_ID]), 1):
            dev.send_id()  # The auto-detect handler responds with the ID
        self.assertEqual(bytes(dev.output), b"1ALS")

    def test_pulseview_reset_bytes_ignored(self):
        """PulseView sends 5x 0x00 (SUMP_RESET) before 0x02. Resets don't generate output."""
        dev = MockSUMPDevice()
        for _ in range(5):
            dev.process_byte(SUMP_RESET)
        self.assertEqual(len(dev.output), 0)
        # Then the ID query should work
        dev.process_byte(SUMP_ID)
        self.assertEqual(bytes(dev.output), b"1ALS")


class TestSUMPBitbangCapture(unittest.TestCase):
    """Test that bitbang operations populate the SUMP circular sample buffer."""

    def test_record_sample_basic(self):
        """Recording a sample stores correct TX/RX bits."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(tx_high=True, rx_high=False)
        self.assertEqual(dev.sample_count, 1)
        self.assertEqual(dev.samples[0], SUMP_BIT_TX)  # TX high, RX low

    def test_record_sample_both_high(self):
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(tx_high=True, rx_high=True)
        self.assertEqual(dev.samples[0], SUMP_BIT_TX | SUMP_BIT_RX)

    def test_record_sample_both_low(self):
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(tx_high=False, rx_high=False)
        self.assertEqual(dev.samples[0], 0x00)

    def test_record_sample_rx_only(self):
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(tx_high=False, rx_high=True)
        self.assertEqual(dev.samples[0], SUMP_BIT_RX)

    def test_record_multiple_samples(self):
        """Multiple samples accumulate sequentially."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(True, True)   # 0x03
        dev.record_sample(True, False)  # 0x01
        dev.record_sample(False, True)  # 0x02
        dev.record_sample(False, False) # 0x00
        self.assertEqual(dev.sample_count, 4)
        self.assertEqual(dev.samples[0], 0x03)
        self.assertEqual(dev.samples[1], 0x01)
        self.assertEqual(dev.samples[2], 0x02)
        self.assertEqual(dev.samples[3], 0x00)

    def test_reset_capture_clears_count(self):
        """reset_capture zeros the sample count and write index."""
        dev = MockSUMPDevice()
        dev.record_sample(True, True)
        dev.record_sample(True, True)
        self.assertEqual(dev.sample_count, 2)
        dev.reset_capture()
        self.assertEqual(dev.sample_count, 0)
        self.assertEqual(dev.write_index, 0)
        self.assertEqual(dev.trigger_index, SUMP_NO_TRIGGER)

    def test_circular_buffer_wraps(self):
        """When more than SUMP_SAMPLE_BUFFER_SIZE samples are written, buffer wraps."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        for i in range(SUMP_SAMPLE_BUFFER_SIZE + 10):
            tx = (i % 2 == 0)
            dev.record_sample(tx, not tx)
        self.assertEqual(dev.sample_count, SUMP_SAMPLE_BUFFER_SIZE + 10)
        self.assertEqual(dev.write_index, 10)  # wrapped around

    def test_mark_trigger_sets_index(self):
        """mark_trigger records the linear sample index."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(True, True)
        dev.record_sample(False, False)
        dev.mark_trigger()
        self.assertEqual(dev.trigger_index, 2)

    def test_mark_trigger_sets_trig_bit(self):
        """mark_trigger sets bit 2 on the most recently written sample."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(True, True)   # index 0: 0x03
        dev.record_sample(False, False) # index 1: 0x00
        dev.mark_trigger()
        # The last sample (index 1) should now have TRIG bit set
        self.assertEqual(dev.samples[1] & SUMP_BIT_TRIG, SUMP_BIT_TRIG)
        # The first sample should NOT have TRIG bit
        self.assertEqual(dev.samples[0] & SUMP_BIT_TRIG, 0)

    def test_trigger_index_default_is_no_trigger(self):
        """Before any trigger, trigger_index is SUMP_NO_TRIGGER."""
        dev = MockSUMPDevice()
        self.assertEqual(dev.trigger_index, SUMP_NO_TRIGGER)

    def test_reset_clears_trigger_index(self):
        """reset_capture resets trigger_index to SUMP_NO_TRIGGER."""
        dev = MockSUMPDevice()
        dev.record_sample(True, True)
        dev.mark_trigger()
        self.assertNotEqual(dev.trigger_index, SUMP_NO_TRIGGER)
        dev.reset_capture()
        self.assertEqual(dev.trigger_index, SUMP_NO_TRIGGER)


class TestSUMPRunServesPreRecordedSamples(unittest.TestCase):
    """Test that SUMP_RUN returns the pre-recorded bitbang samples with windowing."""

    def test_run_with_no_samples_returns_idle(self):
        """When no bitbang has been executed, RUN returns all-recessive samples."""
        dev = MockSUMPDevice()
        dev.read_count = 4
        dev.process_byte(SUMP_RUN)
        # Should have 4 * 4 = 16 bytes (all SUMP_BIT_TX|SUMP_BIT_RX = 0x03, 0, 0, 0)
        self.assertEqual(len(dev.output), 16)
        for i in range(4):
            self.assertEqual(dev.output[i * 4], SUMP_BIT_TX | SUMP_BIT_RX)
            self.assertEqual(dev.output[i * 4 + 1], 0)
            self.assertEqual(dev.output[i * 4 + 2], 0)
            self.assertEqual(dev.output[i * 4 + 3], 0)

    def test_run_returns_samples_newest_to_oldest(self):
        """Samples should be sent in reverse order (newest first)."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(False, False)  # sample 0: 0x00
        dev.record_sample(True, False)   # sample 1: 0x01
        dev.record_sample(False, True)   # sample 2: 0x02
        dev.record_sample(True, True)    # sample 3: 0x03
        dev.read_count = 4
        dev.delay_count = 4  # all post-trigger (no trigger set)
        dev.process_byte(SUMP_RUN)
        # Newest first: 0x03, 0x02, 0x01, 0x00
        self.assertEqual(dev.output[0], 0x03)
        self.assertEqual(dev.output[4], 0x02)
        self.assertEqual(dev.output[8], 0x01)
        self.assertEqual(dev.output[12], 0x00)

    def test_run_clamps_to_available_samples(self):
        """If read_count > available samples, pads with idle."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(True, True)
        dev.record_sample(False, False)
        dev.read_count = 4
        dev.delay_count = 4
        dev.process_byte(SUMP_RUN)
        # 2 real + 2 padding = 4 samples * 4 bytes
        self.assertEqual(len(dev.output), 16)

    def test_run_preserves_samples_across_multiple_reads(self):
        """Samples persist after SUMP_RUN (user can read again)."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(True, True)
        dev.record_sample(False, True)
        dev.read_count = 2
        dev.delay_count = 2
        dev.process_byte(SUMP_RUN)
        output1 = bytes(dev.output)
        dev.output.clear()
        dev.process_byte(SUMP_RUN)
        output2 = bytes(dev.output)
        self.assertEqual(output1, output2)

    def test_run_each_sample_is_4_bytes(self):
        """SUMP protocol sends each sample as 4 bytes: [sample, 0, 0, 0]."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(True, False)  # 0x01
        dev.read_count = 1
        dev.delay_count = 1
        dev.process_byte(SUMP_RUN)
        self.assertEqual(len(dev.output), 4)
        self.assertEqual(dev.output[0], 0x01)
        self.assertEqual(dev.output[1], 0)
        self.assertEqual(dev.output[2], 0)
        self.assertEqual(dev.output[3], 0)


class TestSUMPPrePostTriggerWindowing(unittest.TestCase):
    """Test the pre-trigger / post-trigger windowing behavior."""

    def _record_sequence(self, dev, count, pattern=None):
        """Record count samples. If pattern is None, use incrementing values."""
        for i in range(count):
            if pattern is not None:
                tx = bool(pattern[i % len(pattern)] & SUMP_BIT_TX)
                rx = bool(pattern[i % len(pattern)] & SUMP_BIT_RX)
            else:
                tx = bool(i & 1)
                rx = bool(i & 2)
            dev.record_sample(tx, rx)

    def test_trigger_at_middle_windows_correctly(self):
        """With trigger at middle, pre- and post-trigger samples are balanced."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        # Record 10 samples, trigger after sample 5
        for i in range(5):
            dev.record_sample(True, True)   # pre-trigger: 0x03
        dev.mark_trigger()
        for i in range(5):
            dev.record_sample(False, False) # post-trigger: 0x00

        dev.read_count = 8
        dev.delay_count = 4  # 4 post-trigger, 4 pre-trigger

        dev.output.clear()
        dev.process_byte(SUMP_RUN)

        # Expected: 8 samples * 4 bytes = 32 bytes
        self.assertEqual(len(dev.output), 32)

        # The window should be: 4 pre-trigger samples + 4 post-trigger samples
        # Sent newest-to-oldest:
        # Post-trigger samples are at indices 5,6,7,8 (values: 0x00)
        # Pre-trigger samples are at indices 1,2,3,4 (values: 0x03, last one has TRIG bit)
        # The trigger sample (index 4) has TRIG bit set: 0x03 | 0x04 = 0x07

        # Newest first: index 8,7,6,5 (post-trigger, value 0x00), then 4,3,2,1 (pre)
        samples = [dev.output[i * 4] for i in range(8)]
        # Post-trigger (4 samples, value 0x00)
        self.assertEqual(samples[0], 0x00)  # index 8
        self.assertEqual(samples[1], 0x00)  # index 7
        self.assertEqual(samples[2], 0x00)  # index 6
        self.assertEqual(samples[3], 0x00)  # index 5
        # Pre-trigger (4 samples): index 4 has TRIG bit, rest are 0x03
        self.assertEqual(samples[4] & SUMP_BIT_TRIG, SUMP_BIT_TRIG)  # trigger sample
        self.assertEqual(samples[5], 0x03)  # index 3
        self.assertEqual(samples[6], 0x03)  # index 2
        self.assertEqual(samples[7], 0x03)  # index 1

    def test_no_trigger_sends_most_recent(self):
        """Without trigger, sends the most recent read_count samples."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        for i in range(100):
            dev.record_sample(i % 2, (i + 1) % 2)
        dev.read_count = 10
        dev.delay_count = 10
        dev.output.clear()
        dev.process_byte(SUMP_RUN)
        # Should get 10 samples, newest to oldest (indices 99..90)
        self.assertEqual(len(dev.output), 40)

    def test_trigger_with_max_pre_window(self):
        """All pre-trigger, no post-trigger (delay_count=0)."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        for i in range(20):
            dev.record_sample(True, True)
        dev.mark_trigger()
        for i in range(20):
            dev.record_sample(False, False)

        dev.read_count = 10
        dev.delay_count = 0  # All pre-trigger

        dev.output.clear()
        dev.process_byte(SUMP_RUN)
        self.assertEqual(len(dev.output), 40)

        # All samples should be from before the trigger (0x03)
        # The last sample sent is the trigger point with TRIG bit
        samples = [dev.output[i * 4] for i in range(10)]
        # Newest sample in the window is the trigger point itself
        self.assertEqual(samples[0] & SUMP_BIT_TRIG, SUMP_BIT_TRIG)
        # Earlier samples are just 0x03
        for s in samples[1:]:
            self.assertEqual(s, 0x03)

    def test_trigger_with_max_post_window(self):
        """All post-trigger, no pre-trigger (delay_count=read_count)."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        for i in range(20):
            dev.record_sample(True, True)
        dev.mark_trigger()
        for i in range(20):
            dev.record_sample(False, False)

        dev.read_count = 10
        dev.delay_count = 10  # All post-trigger

        dev.output.clear()
        dev.process_byte(SUMP_RUN)
        self.assertEqual(len(dev.output), 40)

        # All 10 samples should be from after the trigger (0x00)
        samples = [dev.output[i * 4] for i in range(10)]
        for s in samples:
            self.assertEqual(s, 0x00)

    def test_trigger_near_start_pads_with_idle(self):
        """If trigger is near the start, pre-window is padded."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        dev.record_sample(True, True)  # 1 sample before trigger
        dev.mark_trigger()
        for i in range(10):
            dev.record_sample(False, False)

        dev.read_count = 8
        dev.delay_count = 4  # 4 pre + 4 post

        dev.output.clear()
        dev.process_byte(SUMP_RUN)
        self.assertEqual(len(dev.output), 32)

    def test_trigger_bit_visible_in_output(self):
        """The TRIG bit (bit 2) appears in exactly one sample in the output."""
        dev = MockSUMPDevice()
        dev.reset_capture()
        for i in range(20):
            dev.record_sample(True, True)
        dev.mark_trigger()
        for i in range(20):
            dev.record_sample(True, True)

        dev.read_count = 30
        dev.delay_count = 15

        dev.output.clear()
        dev.process_byte(SUMP_RUN)

        samples = [dev.output[i * 4] for i in range(30)]
        trig_samples = [s for s in samples if s & SUMP_BIT_TRIG]
        self.assertEqual(len(trig_samples), 1, "Exactly one sample should have TRIG bit")

    def test_circular_buffer_with_trigger(self):
        """Circular buffer correctly wraps and trigger still works."""
        dev = MockSUMPDevice()
        dev.reset_capture()

        # Fill buffer past capacity (5000 > 4096)
        for i in range(4000):
            dev.record_sample(True, True)
        dev.mark_trigger()
        for i in range(1000):
            dev.record_sample(False, False)

        dev.read_count = 2000
        dev.delay_count = 1000  # 1000 pre + 1000 post

        dev.output.clear()
        dev.process_byte(SUMP_RUN)
        self.assertEqual(len(dev.output), 2000 * 4)

        # First 1000 samples (newest) should be post-trigger (0x00)
        samples = [dev.output[i * 4] for i in range(1000)]
        for s in samples:
            self.assertEqual(s, 0x00)

    def test_delay_count_comes_from_sump_cnt(self):
        """SUMP_CNT command correctly sets delay_count for windowing."""
        dev = MockSUMPDevice()
        # SUMP_CNT: read=256, delay=128
        for b in encode_sump_cnt(256, 128):
            dev.process_byte(b)
        self.assertEqual(dev.read_count, 256)
        self.assertEqual(dev.delay_count, 128)

    def test_default_delay_equals_buffer_size(self):
        """Default delay_count = SUMP_SAMPLE_BUFFER_SIZE (all post-trigger)."""
        dev = MockSUMPDevice()
        self.assertEqual(dev.delay_count, SUMP_SAMPLE_BUFFER_SIZE)


class TestSUMPCDCBinaryForwarding(unittest.TestCase):
    """Test the CDC ISR binary byte forwarding logic.

    Simulates the behavior of usbd_cdc_if.c CDC_Receive_FS:
    - Null (0x00) and newline bytes are ignored
    - Non-printable control bytes (< 0x20) arriving as the first byte
      with an empty buffer are forwarded immediately as 1-byte commands
    - Printable bytes are buffered until \\r
    """

    def _simulate_cdc_receive(self, data):
        """Simulate CDC_Receive_FS logic. Returns list of forwarded commands."""
        commands = []
        current_buf = bytearray()

        for byte in data:
            if byte == ord('\n') or byte == 0:
                # Ignored
                continue
            elif byte < 0x20 and byte != ord('\r') and len(current_buf) == 0:
                # SUMP auto-detect: forward immediately as 1-byte command
                commands.append(bytes([byte]))
            elif byte != ord('\r'):
                current_buf.append(byte)
            else:
                # CR: forward buffered command
                if len(current_buf) > 0:
                    commands.append(bytes(current_buf))
                current_buf = bytearray()

        return commands

    def test_sump_id_forwarded_immediately(self):
        """SUMP_ID (0x02) arriving alone is forwarded as a 1-byte command."""
        cmds = self._simulate_cdc_receive([0x02])
        self.assertEqual(len(cmds), 1)
        self.assertEqual(cmds[0], bytes([0x02]))

    def test_null_bytes_ignored(self):
        """Null bytes (0x00) are silently ignored."""
        cmds = self._simulate_cdc_receive([0x00, 0x00, 0x00])
        self.assertEqual(len(cmds), 0)

    def test_pulseview_handshake_sequence(self):
        """PulseView sends 5x 0x00 then 0x02. Only 0x02 gets forwarded."""
        cmds = self._simulate_cdc_receive([0x00, 0x00, 0x00, 0x00, 0x00, 0x02])
        self.assertEqual(len(cmds), 1)
        self.assertEqual(cmds[0], bytes([0x02]))

    def test_normal_slcan_command_unaffected(self):
        """Normal slcan commands (printable + CR) work as before."""
        cmds = self._simulate_cdc_receive(b"t12340011223344\r")
        self.assertEqual(len(cmds), 1)
        self.assertEqual(cmds[0], b"t12340011223344")

    def test_cli_hash_command_unaffected(self):
        """The # command to enter CLI mode still works."""
        cmds = self._simulate_cdc_receive(b"#\r")
        self.assertEqual(len(cmds), 1)
        self.assertEqual(cmds[0], b"#")

    def test_binary_byte_not_forwarded_if_buffer_nonempty(self):
        """Control bytes after printable chars are buffered normally, not forwarded."""
        cmds = self._simulate_cdc_receive([ord('t'), 0x02, ord('\r')])
        self.assertEqual(len(cmds), 1)
        # The 0x02 is buffered as part of the command, not forwarded separately
        self.assertEqual(cmds[0], b"t\x02")

    def test_esc_forwarded_for_sump_exit(self):
        """ESC (0x1B) arriving alone is forwarded immediately."""
        cmds = self._simulate_cdc_receive([0x1B])
        self.assertEqual(len(cmds), 1)
        self.assertEqual(cmds[0], bytes([0x1B]))


if __name__ == "__main__":
    unittest.main()
