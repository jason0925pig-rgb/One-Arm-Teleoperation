import unittest
from pathlib import Path
import sys

import numpy as np


TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from export_rosbag_to_lerobot import (
    _feature_schema,
    _schema_matches,
    nearest_sample,
    previous_sample,
    uniform_grid_ns,
)


class TimestampSelectionTests(unittest.TestCase):
    def test_nearest_sample_selects_smallest_absolute_skew(self) -> None:
        selected = nearest_sample([0, 10, 20], 14, 5)
        self.assertIsNotNone(selected)
        assert selected is not None
        self.assertEqual(selected.index, 1)
        self.assertEqual(selected.skew_ns, 4)

    def test_nearest_sample_rejects_stale_data(self) -> None:
        self.assertIsNone(nearest_sample([0, 10], 20, 9))

    def test_previous_sample_never_uses_future_command(self) -> None:
        selected = previous_sample([0, 10, 20], 19, 20)
        self.assertIsNotNone(selected)
        assert selected is not None
        self.assertEqual(selected.index, 1)
        self.assertEqual(selected.skew_ns, 9)

    def test_previous_sample_rejects_old_executed_action(self) -> None:
        self.assertIsNone(previous_sample([0, 10], 20, 9))

    def test_uniform_grid_is_exactly_frame_index_based(self) -> None:
        grid = uniform_grid_ns(1_000_000_000, 1_100_000_000, 30)
        self.assertEqual(
            grid,
            [
                1_000_000_000,
                1_033_333_333,
                1_066_666_667,
                1_100_000_000,
            ],
        )
        periods = np.diff(grid)
        self.assertTrue(np.all(np.isin(periods, [33_333_333, 33_333_334])))


class FeatureSchemaTests(unittest.TestCase):
    def test_schema_uses_standard_eight_axis_state_and_action(self) -> None:
        features = _feature_schema((720, 1280, 3), (720, 1280, 3))
        self.assertEqual(features["observation.state"]["shape"], (8,))
        self.assertEqual(features["action"]["shape"], (8,))
        self.assertEqual(
            features["observation.images.head"]["dtype"], "video"
        )
        self.assertEqual(
            features["observation.images.wrist_right"]["names"],
            ["height", "width", "channels"],
        )

    def test_schema_allows_lerobot_generated_bookkeeping_features(self) -> None:
        expected = _feature_schema((720, 1280, 3), (720, 1280, 3))
        current = dict(expected)
        current["timestamp"] = {
            "dtype": "float32",
            "shape": (1,),
            "names": None,
        }
        current["episode_index"] = {
            "dtype": "int64",
            "shape": (1,),
            "names": None,
        }
        self.assertTrue(_schema_matches(current, expected))

    def test_schema_still_rejects_changed_user_feature(self) -> None:
        expected = _feature_schema((720, 1280, 3), (720, 1280, 3))
        current = dict(expected)
        current["action"] = dict(current["action"])
        current["action"]["shape"] = (7,)
        self.assertFalse(_schema_matches(current, expected))


if __name__ == "__main__":
    unittest.main()
