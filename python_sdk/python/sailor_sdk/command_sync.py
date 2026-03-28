from __future__ import annotations

from typing import Sequence, Tuple


def resolve_arm_run_time(
    sample_times: Sequence[float],
    index: int,
    *,
    configured_run_time: float = 0.0,
) -> float:
    configured = float(configured_run_time)
    if configured > 0.0:
        return configured

    values = [float(value) for value in sample_times]
    if not values:
        return 0.0

    if 0 <= index < len(values) - 1:
        forward_dt = values[index + 1] - values[index]
        if forward_dt > 1e-9:
            return forward_dt

    if 0 < index < len(values):
        backward_dt = values[index] - values[index - 1]
        if backward_dt > 1e-9:
            return backward_dt

    return 0.0


def build_humanoid_publish_order(
    *,
    publish_waist: bool,
    waist_first: bool = True,
) -> Tuple[str, ...]:
    if not publish_waist:
        return ('arm',)
    return ('waist', 'arm') if waist_first else ('arm', 'waist')
