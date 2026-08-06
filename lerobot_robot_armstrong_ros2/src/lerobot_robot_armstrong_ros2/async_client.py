"""Armstrong-specific LeRobot async client with overlapping action chunks."""

import logging
import threading
import time
from dataclasses import asdict
from pprint import pformat

import draccus

from lerobot.async_inference.configs import RobotClientConfig
from lerobot.async_inference.helpers import TimedObservation, visualize_action_queue_size
from lerobot.async_inference.robot_client import RobotClient
from lerobot.utils.import_utils import register_third_party_plugins


class ArmstrongRobotClient(RobotClient):
    """Request the next chunk before the current 30 Hz queue becomes empty.

    Upstream marks an observation ``must_go`` only after the queue is empty.
    That is safe for high-latency action units but causes gaps for our radian
    state, because the policy server otherwise regards nearby states as
    similar.  This override marks the threshold observation as mandatory, so
    the normal upstream timestamp aggregation overlaps old and new chunks.
    """

    def control_loop_observation(self, task: str, verbose: bool = False):
        try:
            start_time = time.perf_counter()
            raw_observation = self.robot.get_observation()
            raw_observation["task"] = task
            with self.latest_action_lock:
                latest_action = self.latest_action
            observation = TimedObservation(
                timestamp=time.time(),
                observation=raw_observation,
                timestep=max(latest_action, 0),
            )
            with self.action_queue_lock:
                current_queue_size = self.action_queue.qsize()
                threshold_reached = (
                    self.action_chunk_size > 0
                    and current_queue_size / self.action_chunk_size
                    <= self._chunk_size_threshold
                )
                observation.must_go = self.must_go.is_set() and (
                    self.action_chunk_size <= 0 or threshold_reached
                )
            sent = self.send_observation(observation)
            if sent and observation.must_go:
                self.must_go.clear()
            if verbose:
                elapsed = time.perf_counter() - start_time
                self.logger.info(
                    "Obs #%s queue=%s must_go=%s capture_send_ms=%.2f",
                    observation.get_timestep(),
                    current_queue_size,
                    observation.must_go,
                    elapsed * 1000,
                )
            return raw_observation
        except Exception as exc:
            self.logger.error("Error in Armstrong observation sender: %s", exc)
            return None


@draccus.wrap()
def main(cfg: RobotClientConfig) -> None:
    logging.info(pformat(asdict(cfg)))
    client = ArmstrongRobotClient(cfg)
    if not client.start():
        raise SystemExit(2)
    action_receiver = threading.Thread(target=client.receive_actions, daemon=True)
    action_receiver.start()
    try:
        client.control_loop(task=cfg.task)
    finally:
        client.stop()
        action_receiver.join()
        if cfg.debug_visualize_queue_size:
            visualize_action_queue_size(client.action_queue_size)
        client.logger.info("Client stopped")


if __name__ == "__main__":
    register_third_party_plugins()
    main()
