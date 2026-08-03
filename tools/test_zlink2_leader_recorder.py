import json
import socket
import unittest
from pathlib import Path
import sys

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from zlink2_leader_recorder import (
    SpaceToggleLatch,
    UdpTeleopSender,
    control_event_text,
    parse_ipv4_address,
)


class SpaceToggleLatchTests(unittest.TestCase):
    def test_space_rising_edges_toggle_and_release_does_not_stop(self) -> None:
        latch = SpaceToggleLatch()
        latch.initialize(False)

        self.assertIsNone(latch.update(False))
        self.assertEqual(latch.update(True), "activated")
        self.assertTrue(latch.active())
        self.assertIsNone(latch.update(True))
        self.assertIsNone(latch.update(False))
        self.assertTrue(latch.active())
        self.assertEqual(latch.update(True), "deactivated")
        self.assertFalse(latch.active())

    def test_deactivate_clears_latch(self) -> None:
        latch = SpaceToggleLatch()
        latch.initialize(False)
        latch.update(True)
        self.assertTrue(latch.active())
        latch.deactivate()
        self.assertFalse(latch.active())


class ControlEventTextTests(unittest.TestCase):
    def test_control_event_is_visually_delimited_and_keeps_all_lines(self) -> None:
        text = control_event_text("TELEOP ACTIVE", ("line one", "line two"))

        self.assertIn("TELEOP ACTIVE", text)
        self.assertIn("line one", text)
        self.assertIn("line two", text)
        self.assertTrue(text.startswith("="))
        self.assertTrue(text.endswith("="))


class UdpBindingTests(unittest.TestCase):
    def test_sender_binds_to_requested_source_address(self) -> None:
        sender = UdpTeleopSender(("127.0.0.1", 5005), "127.0.0.1")
        try:
            self.assertEqual(sender.socket.getsockname()[0], "127.0.0.1")
        finally:
            sender.close()

    def test_ipv6_bind_address_is_rejected(self) -> None:
        with self.assertRaisesRegex(Exception, "only an IPv4"):
            parse_ipv4_address("::1")

    def test_leader_packet_carries_monotonic_freshness_clock(self) -> None:
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1.0)
        sender = UdpTeleopSender(receiver.getsockname(), "127.0.0.1")
        labels = tuple(f"joint_{index}" for index in range(1, 8)) + (
            "gripper",
        )
        row = {
            "sequence": 9,
            "timestamp_unix_ns": 1_900_000_000_000_000_000,
            "monotonic_ns": 123_456_789,
            "gripper_pulse": 2320,
            "gripper_unwrapped_pulse": 2320,
            "gripper_normalized": 0.0,
            "gripper_state": "CLOSED",
            "gripper_state_changed": 0,
        }
        for index in range(1, 8):
            row[f"joint_{index}_pulse"] = 1000 + index
        try:
            sender.send(
                row,
                labels,
                {label: index for index, label in enumerate(labels)},
                "test-session",
                True,
            )
            payload, _ = receiver.recvfrom(4096)
            packet = json.loads(payload.decode("ascii"))
            self.assertEqual(packet["version"], 2)
            self.assertEqual(packet["sender_monotonic_ns"], 123_456_789)
            self.assertEqual(
                packet["timestamp_unix_ns"],
                1_900_000_000_000_000_000,
            )
        finally:
            sender.close()
            receiver.close()


if __name__ == "__main__":
    unittest.main()
