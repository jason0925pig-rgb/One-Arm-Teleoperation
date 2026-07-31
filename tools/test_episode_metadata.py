import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

from set_episode_outcome import recover_closed_sqlite_recording


class ClosedRosbagRecoveryTests(unittest.TestCase):
    def test_recovers_stuck_recording_from_valid_closed_sqlite_bag(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            episode_dir = Path(temporary)
            bag_dir = episode_dir / "rosbag"
            bag_dir.mkdir()
            (bag_dir / "metadata.yaml").write_text(
                "rosbag2_bagfile_information:\n",
                encoding="utf-8",
            )
            database = bag_dir / "rosbag_0.db3"
            connection = sqlite3.connect(database)
            connection.execute(
                "CREATE TABLE messages("
                "id INTEGER PRIMARY KEY, "
                "topic_id INTEGER NOT NULL, "
                "timestamp INTEGER NOT NULL, "
                "data BLOB NOT NULL)"
            )
            connection.executemany(
                "INSERT INTO messages(topic_id, timestamp, data) "
                "VALUES(1, ?, ?)",
                (
                    (1_000_000_000, b"a"),
                    (2_500_000_000, b"b"),
                ),
            )
            connection.commit()
            connection.close()

            payload = {
                "status": "recording",
                "storage": "sqlite3",
                "outcome": "unknown",
            }
            recover_closed_sqlite_recording(episode_dir, payload)

            self.assertEqual(payload["status"], "complete")
            self.assertEqual(payload["duration_seconds"], 1.5)
            self.assertEqual(payload["recovered_message_count"], 2)
            self.assertEqual(
                payload["stop_reason"], "recovered_closed_rosbag"
            )

    def test_does_not_change_already_complete_metadata(self) -> None:
        payload = {"status": "complete", "outcome": "unknown"}
        recover_closed_sqlite_recording(Path("."), payload)
        self.assertEqual(
            payload,
            {"status": "complete", "outcome": "unknown"},
        )

    def test_refuses_recording_without_closed_bag_marker(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            episode_dir = Path(temporary)
            (episode_dir / "rosbag").mkdir()
            payload = {"status": "recording", "storage": "sqlite3"}
            with self.assertRaises(SystemExit):
                recover_closed_sqlite_recording(episode_dir, payload)


if __name__ == "__main__":
    unittest.main()
