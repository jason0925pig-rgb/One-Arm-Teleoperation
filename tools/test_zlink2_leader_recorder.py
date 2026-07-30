import unittest
from pathlib import Path
import sys

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from zlink2_leader_recorder import SpaceToggleLatch, control_event_text


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


if __name__ == "__main__":
    unittest.main()
