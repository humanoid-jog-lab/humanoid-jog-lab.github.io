import unittest

from sailor_sdk.command_sync import build_humanoid_publish_order, resolve_arm_run_time


class CommandSyncTests(unittest.TestCase):
    def test_resolve_arm_run_time_prefers_configured_value(self) -> None:
        self.assertEqual(
            resolve_arm_run_time([0.0, 0.2, 0.4], 0, configured_run_time=0.35),
            0.35,
        )

    def test_resolve_arm_run_time_uses_forward_sample_delta(self) -> None:
        self.assertEqual(
            resolve_arm_run_time([0.0, 0.2, 0.5], 1, configured_run_time=0.0),
            0.3,
        )

    def test_resolve_arm_run_time_uses_backward_delta_for_last_sample(self) -> None:
        self.assertEqual(
            resolve_arm_run_time([0.0, 0.2, 0.5], 2, configured_run_time=0.0),
            0.3,
        )

    def test_build_humanoid_publish_order_defaults_to_waist_first(self) -> None:
        self.assertEqual(
            build_humanoid_publish_order(publish_waist=True),
            ('waist', 'arm'),
        )

    def test_build_humanoid_publish_order_skips_waist_when_disabled(self) -> None:
        self.assertEqual(
            build_humanoid_publish_order(publish_waist=False),
            ('arm',),
        )


if __name__ == '__main__':
    unittest.main()
