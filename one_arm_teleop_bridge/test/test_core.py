import json

import pytest

from one_arm_teleop_bridge.core import (
    MappingConfig,
    MultiJointUnwrapper,
    OffsetAbsoluteMapper,
    PacketError,
    SafetyError,
    parse_leader_packet,
)


def make_packet():
    return {
        "protocol": "one_arm_teleop",
        "version": 1,
        "session_id": "test",
        "sequence": 1,
        "timestamp_unix_ns": 100,
        "complete": True,
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


def test_packet_and_wrap():
    frame = parse_leader_packet(json.dumps(make_packet()).encode("ascii"))
    unwrapper = MultiJointUnwrapper(2500, 800)
    first = unwrapper.update(frame.joint_pulses)
    second = unwrapper.update((100, 200, 500, 600, 700, 800, 900))
    assert first[0] == 2400
    assert second[0] == 2600


def test_bad_joint_names_are_rejected():
    packet = make_packet()
    packet["joint_names"][0] = "wrong"
    with pytest.raises(PacketError):
        parse_leader_packet(json.dumps(packet).encode("ascii"))


def test_offset_absolute_mapping_and_limit_stop():
    config = MappingConfig(
        signs=(1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        scale_rad_per_pulse=(0.001,) * 7,
        lower_limits=(-1.0,) * 7,
        upper_limits=(1.0,) * 7,
    )
    mapper = OffsetAbsoluteMapper(config)
    mapper.set_reference((1000.0,) * 7, (0.0,) * 7)
    target = mapper.map((1100.0,) * 7)
    assert target[0] == pytest.approx(0.1)
    assert target[1] == pytest.approx(-0.1)
    with pytest.raises(SafetyError):
        mapper.map((3000.0,) * 7)
