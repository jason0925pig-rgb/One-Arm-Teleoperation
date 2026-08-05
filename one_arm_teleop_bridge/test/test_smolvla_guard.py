import math
import unittest

from one_arm_teleop_bridge.smolvla_guard import (
    PolicySafetyConfig,
    PolicySafetyError,
    guard_policy_action,
    validate_initial_pose,
)


def config() -> PolicySafetyConfig:
    return PolicySafetyConfig(
        task_lower=(-1.0,) * 7,
        task_upper=(1.0,) * 7,
        initial_lower=(-0.2,) * 7,
        initial_upper=(0.2,) * 7,
        max_target_error_rad=0.25,
        small_envelope_overshoot_rad=0.03,
    )


class SmolVLAGuardTests(unittest.TestCase):
    def test_initial_pose_and_open_gripper(self):
        self.assertEqual(validate_initial_pose((0.0,) * 8, config()), (0.0,) * 8)
        with self.assertRaises(PolicySafetyError):
            validate_initial_pose((0.3,) + (0.0,) * 7, config())
        with self.assertRaises(PolicySafetyError):
            validate_initial_pose((0.0,) * 7 + (1.0,), config())

    def test_action_is_absolute_and_hysteretic(self):
        joints, gripper = guard_policy_action(
            (0.1,) * 7 + (0.5,),
            (0.0,) * 8,
            True,
            config(),
        )
        self.assertEqual(joints, (0.1,) * 7)
        self.assertTrue(gripper)

    def test_small_envelope_overshoot_is_clamped(self):
        joints, _ = guard_policy_action(
            (1.02,) + (0.0,) * 6 + (0.0,),
            (0.9,) + (0.0,) * 7,
            False,
            config(),
        )
        self.assertEqual(joints[0], 1.0)

    def test_large_overshoot_target_jump_and_nonfinite_are_rejected(self):
        with self.assertRaises(PolicySafetyError):
            guard_policy_action((1.04,) + (0.0,) * 7, (0.9,) + (0.0,) * 7, False, config())
        with self.assertRaises(PolicySafetyError):
            guard_policy_action((0.3,) + (0.0,) * 7, (0.0,) * 8, False, config())
        with self.assertRaises(PolicySafetyError):
            guard_policy_action((math.nan,) + (0.0,) * 7, (0.0,) * 8, False, config())


if __name__ == "__main__":
    unittest.main()
