import math
import unittest

from one_arm_teleop_bridge.smolvla_guard import (
    GripperTemporalConfig,
    GripperTemporalFilter,
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

    def test_gripper_requires_consecutive_decisive_frames(self):
        filter_ = GripperTemporalFilter(
            GripperTemporalConfig(
                confirmation_frames=3,
                min_state_dwell_seconds=0.0,
                contact_hold_seconds=0.0,
            )
        )
        self.assertFalse(filter_.update(0.90, now=0.0, contact_active=False).transitioned)
        # An ambiguous frame clears the candidate sequence.
        self.assertFalse(filter_.update(0.50, now=0.1, contact_active=False).transitioned)
        self.assertFalse(filter_.update(0.90, now=0.2, contact_active=False).transitioned)
        self.assertFalse(filter_.update(0.90, now=0.3, contact_active=False).transitioned)
        result = filter_.update(0.90, now=0.4, contact_active=False)
        self.assertTrue(result.transitioned)
        self.assertTrue(result.command_closed)

    def test_gripper_close_dwell_and_contact_hold_block_reopen(self):
        filter_ = GripperTemporalFilter(
            GripperTemporalConfig(
                confirmation_frames=2,
                min_state_dwell_seconds=1.0,
                contact_hold_seconds=3.0,
            ),
            now=-2.0,
        )
        filter_.update(1.0, now=0.0, contact_active=False)
        self.assertTrue(filter_.update(1.0, now=0.1, contact_active=False).transitioned)
        filter_.note_contact(True, now=0.5)

        filter_.update(0.0, now=1.2, contact_active=True)
        held = filter_.update(0.0, now=1.3, contact_active=True)
        self.assertFalse(held.transitioned)
        self.assertEqual(held.blocked_reason, "contact_hold")

        opened = filter_.update(0.0, now=3.6, contact_active=True)
        self.assertTrue(opened.transitioned)
        self.assertFalse(opened.command_closed)


if __name__ == "__main__":
    unittest.main()
