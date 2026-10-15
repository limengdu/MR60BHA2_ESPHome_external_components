#!/usr/bin/env python3
"""Model of seeed_mr60bha2 sensor publish throttling.

This mirrors publish_sensor_state_() in seeed_mr60bha2.cpp so the interval
rules can be checked without compiling firmware.
"""

from __future__ import annotations


def publish_sensor_state(
    state: float | None,
    value: float,
    last_publish_ms: int,
    now_ms: int,
    interval_ms: int,
    force: bool = False,
) -> tuple[bool, float | None, int]:
    """Return (published, new_state, new_last_publish_ms)."""
    if force:
        return True, value, 0
    if state is not None and state == value:
        return False, state, last_publish_ms
    if interval_ms != 0 and last_publish_ms != 0 and now_ms - last_publish_ms < interval_ms:
        return False, state, last_publish_ms
    return True, value, now_ms


def test_omitted_interval_publishes_every_new_value() -> None:
    state = None
    last = 0
    published, state, last = publish_sensor_state(state, 70.0, last, 10, 0)
    assert published and state == 70.0 and last == 10
    published, state, last = publish_sensor_state(state, 71.0, last, 11, 0)
    assert published and state == 71.0 and last == 11


def test_duplicate_value_is_skipped() -> None:
    published, state, last = publish_sensor_state(70.0, 70.0, 10, 20, 0)
    assert not published and state == 70.0 and last == 10


def test_interval_skips_until_window_elapses() -> None:
    state = None
    last = 0
    published, state, last = publish_sensor_state(state, 70.0, last, 1000, 10_000)
    assert published and last == 1000
    published, state, last = publish_sensor_state(state, 72.0, last, 2000, 10_000)
    assert not published and state == 70.0
    published, state, last = publish_sensor_state(state, 72.0, last, 10_999, 10_000)
    assert not published and state == 70.0
    published, state, last = publish_sensor_state(state, 72.0, last, 11_000, 10_000)
    assert published and state == 72.0 and last == 11_000


def test_force_clears_immediately_and_resets_throttle() -> None:
    published, state, last = publish_sensor_state(72.0, 0.0, 11_000, 11_100, 10_000, force=True)
    assert published and state == 0.0 and last == 0
    published, state, last = publish_sensor_state(state, 68.0, last, 11_200, 10_000)
    assert published and state == 68.0 and last == 11_200


def main() -> None:
    test_omitted_interval_publishes_every_new_value()
    test_duplicate_value_is_skipped()
    test_interval_skips_until_window_elapses()
    test_force_clears_immediately_and_resets_throttle()
    print("ok")


if __name__ == "__main__":
    main()
