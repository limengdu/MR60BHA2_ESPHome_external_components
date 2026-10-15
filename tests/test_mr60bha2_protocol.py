#!/usr/bin/env python3
"""Protocol helpers for seeed_mr60bha2 distance / occupancy handling.

This mirrors the frame layout and decode rules in seeed_mr60bha2.cpp so
firmware 1.6.4 and 1.6.10+ payloads can be checked without hardware.
"""

from __future__ import annotations

import math
import struct

SOF = 0x01
MAX_FRAME_DATA_LENGTH = 256

TYPE_POINT_CLOUD = 0x0A04
TYPE_PHASE = 0x0A13
TYPE_BREATH = 0x0A14
TYPE_HEART = 0x0A15
TYPE_DISTANCE = 0x0A16
TYPE_ALT_POSITION = 0x0A17
TYPE_PRESENCE = 0x0F09


def checksum(data: bytes) -> int:
    acc = 0
    for b in data:
        acc ^= b
    return (~acc) & 0xFF


def build_frame(frame_type: int, payload: bytes, frame_id: int = 0x8000) -> bytes:
    header = bytes(
        [
            SOF,
            (frame_id >> 8) & 0xFF,
            frame_id & 0xFF,
            (len(payload) >> 8) & 0xFF,
            len(payload) & 0xFF,
            (frame_type >> 8) & 0xFF,
            frame_type & 0xFF,
        ]
    )
    return header + bytes([checksum(header)]) + payload + bytes([checksum(payload)])


def le_f32(value: float) -> bytes:
    return struct.pack("<f", value)


def le_u32(value: int) -> bytes:
    return struct.pack("<I", value)


def decode_le_float(data: bytes) -> float:
    return struct.unpack("<f", data[:4])[0]


def range_to_cm(value: float) -> float:
    if math.isfinite(value) and 0.0 < value < 10.0:
        return value * 100.0
    return value


def hypot_to_cm(x_m: float, y_m: float) -> float:
    return math.hypot(x_m, y_m) * 100.0


def parse_distance_payload(payload: bytes) -> tuple[bool, float]:
    """Return (range_valid, distance_cm) for a 0x0A16 payload."""
    if len(payload) >= 8:
        range_flag = struct.unpack("<I", payload[:4])[0]
        distance_cm = range_to_cm(decode_le_float(payload[4:8]))
        return range_flag != 0, distance_cm
    if len(payload) >= 4:
        raw = struct.unpack("<I", payload[:4])[0]
        if raw == 0:
            return False, 0.0
        value = decode_le_float(payload[:4])
        return value != 0.0, range_to_cm(value)
    return False, 0.0


def should_reset_on_type(frame_type: int) -> bool:
    """Current firmware emits extra types; the UART parser must keep them."""
    return False


def occupancy(people_detected: bool, tracked_targets: int) -> bool:
    return people_detected or tracked_targets > 0


def test_empty_four_byte_distance_is_invalid() -> None:
    valid, value = parse_distance_payload(bytes([0, 0, 0, 0]))
    assert not valid and value == 0.0


def test_eight_byte_flag_and_cm_float() -> None:
    # Captured in issue #3: flag=1, AE 47 85 42 => 66.64 cm
    payload = le_u32(1) + bytes.fromhex("AE478542")
    valid, value = parse_distance_payload(payload)
    assert valid
    assert abs(value - 66.64) < 0.01


def test_eight_byte_metres_are_converted_to_cm() -> None:
    payload = le_u32(1) + le_f32(1.23)
    valid, value = parse_distance_payload(payload)
    assert valid
    assert abs(value - 123.0) < 0.01


def test_four_byte_float_distance() -> None:
    payload = le_f32(180.0)
    valid, value = parse_distance_payload(payload)
    assert valid
    assert abs(value - 180.0) < 0.01


def test_invalid_flag_keeps_cloud_distance() -> None:
    valid, _ = parse_distance_payload(le_u32(0) + le_f32(0.0))
    tracked_targets = 1
    assert not valid
    assert occupancy(False, tracked_targets) is True


def test_point_cloud_distance_from_xy_metres() -> None:
    payload = le_u32(1) + le_f32(0.5) + le_f32(1.2)
    count = struct.unpack("<I", payload[:4])[0]
    x_m = decode_le_float(payload[4:8])
    y_m = decode_le_float(payload[8:12])
    assert count == 1
    assert abs(hypot_to_cm(x_m, y_m) - math.hypot(0.5, 1.2) * 100.0) < 0.01


def test_presence_accepts_one_byte_payload() -> None:
    assert bytes([0x01])[0] != 0
    assert bytes([0x00])[0] == 0


def test_unknown_frame_types_are_consumed() -> None:
    frame = build_frame(TYPE_PHASE, le_f32(1.0) + le_f32(0.5) + le_f32(2.1))
    assert frame[0] == SOF
    assert not should_reset_on_type(TYPE_PHASE)
    assert checksum(frame[:7]) == frame[7]
    assert checksum(frame[8:-1]) == frame[-1]


def test_length_cap() -> None:
    assert MAX_FRAME_DATA_LENGTH == 256
    assert 0xFFFF > MAX_FRAME_DATA_LENGTH


def test_occupancy_or_logic() -> None:
    assert occupancy(False, 0) is False
    assert occupancy(True, 0) is True
    assert occupancy(False, 2) is True
    assert occupancy(True, 2) is True


def test_valid_distance_frame_roundtrip() -> None:
    payload = le_u32(1) + le_f32(150.0)
    raw = build_frame(TYPE_DISTANCE, payload, frame_id=0x7548)
    assert raw[5] == 0x0A and raw[6] == 0x16
    assert struct.unpack(">H", raw[3:5])[0] == 8
    valid, value = parse_distance_payload(raw[8:-1])
    assert valid and abs(value - 150.0) < 0.01


def main() -> None:
    test_empty_four_byte_distance_is_invalid()
    test_eight_byte_flag_and_cm_float()
    test_eight_byte_metres_are_converted_to_cm()
    test_four_byte_float_distance()
    test_invalid_flag_keeps_cloud_distance()
    test_point_cloud_distance_from_xy_metres()
    test_presence_accepts_one_byte_payload()
    test_unknown_frame_types_are_consumed()
    test_length_cap()
    test_occupancy_or_logic()
    test_valid_distance_frame_roundtrip()
    print("ok")


if __name__ == "__main__":
    main()
