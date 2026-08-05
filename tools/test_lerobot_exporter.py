import json
import sqlite3
import tempfile
import unittest
from pathlib import Path
import sys

import numpy as np


TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from export_rosbag_to_lerobot import (
    DEFAULT_ACTION_TOPIC,
    SqliteMessageRef,
    SqliteMessageStore,
    TimedStream,
    _completed_export_index,
    _feature_schema,
    _schema_matches,
    _validate_stream_type,
    latched_previous_sample,
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

    def test_latched_previous_sample_keeps_old_gripper_state(self) -> None:
        selected = latched_previous_sample([0, 10], 10_000, 5)
        self.assertIsNotNone(selected)
        assert selected is not None
        self.assertEqual(selected.index, 1)
        self.assertEqual(selected.skew_ns, 9_990)

    def test_latched_previous_sample_never_borrows_future_state(self) -> None:
        self.assertIsNone(latched_previous_sample([10, 20], 9, 5))

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

    def test_schema_can_name_the_primary_camera_chest(self) -> None:
        features = _feature_schema(
            (720, 1280, 3),
            (720, 1280, 3),
            "observation.images.chest",
        )
        self.assertIn("observation.images.chest", features)
        self.assertNotIn("observation.images.head", features)
        self.assertIn("observation.images.wrist_right", features)

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


class ExportIdempotencyTests(unittest.TestCase):
    def test_completed_report_prevents_duplicate_export(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            (dataset_root / "meta").mkdir(parents=True)
            (dataset_root / "meta" / "info.json").write_text(
                json.dumps({"total_episodes": 2}),
                encoding="utf-8",
            )
            report_path = root / "report.json"
            report_path.write_text(
                json.dumps(
                    {
                        "status": "complete",
                        "dataset_episode_index": 1,
                        "dataset_root": str(dataset_root),
                        "repo_id": "local/onearm_tele",
                    }
                ),
                encoding="utf-8",
            )
            self.assertEqual(
                _completed_export_index(
                    report_path,
                    dataset_root.resolve(),
                    "local/onearm_tele",
                ),
                1,
            )

    def test_stale_report_does_not_suppress_export(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            (dataset_root / "meta").mkdir(parents=True)
            (dataset_root / "meta" / "info.json").write_text(
                json.dumps({"total_episodes": 1}),
                encoding="utf-8",
            )
            report_path = root / "report.json"
            report_path.write_text(
                json.dumps(
                    {
                        "status": "complete",
                        "dataset_episode_index": 1,
                        "dataset_root": str(dataset_root),
                        "repo_id": "local/onearm_tele",
                    }
                ),
                encoding="utf-8",
            )
            self.assertIsNone(
                _completed_export_index(
                    report_path,
                    dataset_root.resolve(),
                    "local/onearm_tele",
                )
            )


class RequiredStreamTests(unittest.TestCase):
    def test_empty_executed_action_is_rejected_as_invalid_training_data(
        self,
    ) -> None:
        stream = TimedStream(
            topic=DEFAULT_ACTION_TOPIC,
            type_name="sensor_msgs/msg/JointState",
        )
        with self.assertRaisesRegex(
            SystemExit,
            "contains no executed robot actions",
        ):
            _validate_stream_type(
                stream,
                {"sensor_msgs/msg/JointState"},
            )


class SqliteMessageStoreTests(unittest.TestCase):
    def test_index_keeps_references_instead_of_serialized_payloads(self) -> None:
        class FakeMessage:
            header = None

        with tempfile.TemporaryDirectory() as temporary:
            bag_dir = Path(temporary)
            connection = sqlite3.connect(bag_dir / "rosbag_0.db3")
            connection.executescript(
                """
                CREATE TABLE topics(
                    id INTEGER PRIMARY KEY,
                    name TEXT NOT NULL,
                    type TEXT NOT NULL
                );
                CREATE TABLE messages(
                    id INTEGER PRIMARY KEY,
                    topic_id INTEGER NOT NULL,
                    timestamp INTEGER NOT NULL,
                    data BLOB NOT NULL
                );
                INSERT INTO topics VALUES (
                    1, '/test', 'example_interfaces/msg/UInt8'
                );
                INSERT INTO messages VALUES (10, 1, 1234, X'01020304');
                """
            )
            connection.commit()
            connection.close()

            store = SqliteMessageStore(bag_dir)
            store._deserialize = lambda *_: FakeMessage()  # type: ignore[method-assign]
            try:
                streams = store.index({"/test"})
                reference = streams["/test"].values[0]
                self.assertIsInstance(reference, SqliteMessageRef)
                self.assertEqual(streams["/test"].times_ns, [1234])
                self.assertIsInstance(store.load(reference), FakeMessage)
            finally:
                store.close()


if __name__ == "__main__":
    unittest.main()
