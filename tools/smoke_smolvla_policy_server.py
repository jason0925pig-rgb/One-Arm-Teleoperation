#!/usr/bin/env python3
"""Exercise the LeRobot gRPC policy path without importing ROS or moving a robot."""

from __future__ import annotations

import argparse
import math
import pickle  # nosec: test talks only to an explicitly trusted local server
import time
from pathlib import Path

import grpc
import numpy as np

from lerobot.async_inference.helpers import RemotePolicyConfig, TimedObservation
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.transport import services_pb2, services_pb2_grpc
from lerobot.transport.utils import grpc_channel_options, send_bytes_in_chunks
from lerobot.utils.constants import OBS_STR


JOINT_NAMES = tuple(f"right_joint{index}" for index in range(1, 8))
GRIPPER_NAME = "right_gripper_closed"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", default="127.0.0.1:18080")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--repo-id", default="local/onearm_tele")
    parser.add_argument("--sample-index", type=int, default=0)
    parser.add_argument("--actions-per-chunk", type=int, default=10)
    parser.add_argument("--task", default="把那瓶水放进箱子里。")
    parser.add_argument("--timeout", type=float, default=120.0)
    return parser.parse_args()


def image_to_uint8_hwc(image) -> np.ndarray:
    array = image.detach().float().cpu().numpy()
    if array.ndim != 3:
        raise ValueError(f"expected CHW image, got {array.shape}")
    array = np.moveaxis(array, 0, -1)
    if array.max(initial=0.0) <= 1.0:
        array = array * 255.0
    return np.clip(array, 0, 255).astype(np.uint8)


def main() -> int:
    args = parse_args()
    checkpoint = args.checkpoint.resolve()
    dataset_root = args.dataset_root.resolve()
    if not (checkpoint / "config.json").is_file():
        raise FileNotFoundError(checkpoint / "config.json")

    dataset = LeRobotDataset(
        repo_id=args.repo_id,
        root=dataset_root,
        video_backend="torchcodec",
    )
    sample = dataset[args.sample_index]
    state = sample["observation.state"].detach().float().cpu().tolist()
    if len(state) != 8 or not all(math.isfinite(value) for value in state):
        raise RuntimeError(f"invalid test state: {state}")

    hardware_features = {
        **{name: float for name in JOINT_NAMES},
        GRIPPER_NAME: float,
        "chest": (720, 1280, 3),
        "wrist_right": (720, 1280, 3),
    }
    features = hw_to_dataset_features(hardware_features, OBS_STR, use_video=False)
    policy = RemotePolicyConfig(
        policy_type="smolvla",
        pretrained_name_or_path=str(checkpoint),
        lerobot_features=features,
        actions_per_chunk=args.actions_per_chunk,
        device="cuda",
    )
    raw_observation = {
        **dict(zip((*JOINT_NAMES, GRIPPER_NAME), state, strict=True)),
        "chest": image_to_uint8_hwc(sample["observation.images.chest"]),
        "wrist_right": image_to_uint8_hwc(sample["observation.images.wrist_right"]),
        "task": args.task,
    }
    observation = TimedObservation(
        timestamp=time.time(),
        observation=raw_observation,
        timestep=0,
        must_go=True,
    )

    channel = grpc.insecure_channel(
        args.server,
        grpc_channel_options(initial_backoff="0.0333s"),
    )
    stub = services_pb2_grpc.AsyncInferenceStub(channel)
    try:
        grpc.channel_ready_future(channel).result(timeout=args.timeout)
        stub.Ready(services_pb2.Empty(), timeout=args.timeout)
        stub.SendPolicyInstructions(
            services_pb2.PolicySetup(data=pickle.dumps(policy)),
            timeout=args.timeout,
        )
        chunks = send_bytes_in_chunks(
            pickle.dumps(observation),
            services_pb2.Observation,
            log_prefix="[SMOKE] Observation",
            silent=True,
        )
        stub.SendObservations(chunks, timeout=args.timeout)
        response = stub.GetActions(services_pb2.Empty(), timeout=args.timeout)
        actions = pickle.loads(response.data)  # nosec: trusted local policy server
    finally:
        channel.close()

    if len(actions) != args.actions_per_chunk:
        raise RuntimeError(f"expected {args.actions_per_chunk} actions, got {len(actions)}")
    values = np.stack([action.get_action().detach().float().cpu().numpy() for action in actions])
    if values.shape != (args.actions_per_chunk, 8) or not np.isfinite(values).all():
        raise RuntimeError(f"invalid action chunk: shape={values.shape}")
    print(
        "SMOLVLA_POLICY_SERVER_SMOKE_OK "
        f"actions={values.shape} min={values.min(axis=0).tolist()} max={values.max(axis=0).tolist()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
