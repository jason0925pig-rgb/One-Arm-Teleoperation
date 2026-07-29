import json
import time
import unittest

from one_arm_teleop_bridge.core import (
    LeaderSignalFilter,
    MappingConfig,
    MultiJointUnwrapper,
    OffsetAbsoluteMapper,
    PacketError,
    SafetyError,
    StopFrame,
    parse_leader_packet,
    parse_teleop_packet,
    validate_packet_timestamp,
)


def make_packet():
    return {
        "protocol": "one_arm_teleop",
        "version": 1,
        "message_type": "leader_frame",
        "session_id": "test",
        "sequence": 1,
        "timestamp_unix_ns": time.time_ns(),
        "complete": True,
        "deadman_held": True,
        "joint_names": [f"joint_{index}" for index in range(1, 8)],
        "servo_ids": list(range(7)),
        "joint_pulses": [2400, 100, 500, 600, 700, 800, 900],
        "gripper": {
            "raw_pulse": 2320,
            "unwrapped_pulse": 2320,
            "normalized": 0.0,
            "state": "CLOSED",
            "state_changed": False,
        },
    }


class ProtocolTests(unittest.TestCase):
    def test_packet_and_wrap(self):
        frame = parse_leader_packet(json.dumps(make_packet()).encode("ascii"))
        self.assertTrue(frame.deadman_held)
        unwrapper = MultiJointUnwrapper(2500, 800)
        first = unwrapper.update(frame.joint_pulses)
        second = unwrapper.update((100, 200, 500, 600, 700, 800, 900))
        self.assertEqual(first[0], 2400)
        self.assertEqual(second[0], 2600)

    def test_missing_deadman_defaults_to_released(self):
        packet = make_packet()
        del packet["deadman_held"]
        frame = parse_leader_packet(json.dumps(packet).encode("ascii"))
        self.assertFalse(frame.deadman_held)

    def test_bad_joint_names_are_rejected(self):
        packet = make_packet()
        packet["joint_names"][0] = "wrong"
        with self.assertRaises(PacketError):
            parse_leader_packet(json.dumps(packet).encode("ascii"))

    def test_stop_packet_is_independent_and_idempotent(self):
        packet = {
            "protocol": "one_arm_teleop",
            "version": 1,
            "message_type": "stop",
            "session_id": "test",
            "sequence": 8,
            "timestamp_unix_ns": time.time_ns(),
            "reason": "deadman_released",
        }
        payload = json.dumps(packet).encode("ascii")
        first = parse_teleop_packet(payload)
        second = parse_teleop_packet(payload)
        self.assertIsInstance(first, StopFrame)
        self.assertEqual(first, second)
        with self.assertRaises(PacketError):
            parse_leader_packet(payload)

    def test_stale_and_future_timestamps_are_rejected(self):
        now = 10_000_000_000
        self.assertAlmostEqual(
            validate_packet_timestamp(now - 100_000_000, now, 0.5, 0.25),
            0.1,
        )
        with self.assertRaises(PacketError):
            validate_packet_timestamp(now - 600_000_000, now, 0.5, 0.25)
        with self.assertRaises(PacketError):
            validate_packet_timestamp(now + 300_000_000, now, 0.5, 0.25)


class FilterAndMappingTests(unittest.TestCase):
    def test_filter_rejects_short_spike(self):
        signal_filter = LeaderSignalFilter(median_window=3)
        self.assertEqual(signal_filter.update((100.0,) * 7)[0], 100.0)
        self.assertEqual(signal_filter.update((102.0,) * 7)[0], 101.0)
        self.assertEqual(signal_filter.update((1000.0,) * 7)[0], 102.0)

    def test_filter_deadband_and_low_pass(self):
        deadband = LeaderSignalFilter(deadband_pulses=2.0)
        self.assertEqual(deadband.update((10.0,) * 7)[0], 10.0)
        self.assertEqual(deadband.update((11.0,) * 7)[0], 10.0)
        self.assertEqual(deadband.update((13.0,) * 7)[0], 13.0)

        low_pass = LeaderSignalFilter(low_pass_alpha=0.5)
        low_pass.update((0.0,) * 7)
        self.assertEqual(low_pass.update((10.0,) * 7)[0], 5.0)

    def test_offset_absolute_mapping_and_limit_stop(self):
        config = MappingConfig(
            signs=(1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
            scale_rad_per_pulse=(0.001,) * 7,
            lower_limits=(-1.0,) * 7,
            upper_limits=(1.0,) * 7,
        )
        mapper = OffsetAbsoluteMapper(config)
        mapper.set_reference((1000.0,) * 7, (0.0,) * 7)
        target = mapper.map((1100.0,) * 7)
        self.assertAlmostEqual(target[0], 0.1)
        self.assertAlmostEqual(target[1], -0.1)
        with self.assertRaises(SafetyError):
            mapper.map((3000.0,) * 7)


if __name__ == "__main__":
    unittest.main()
