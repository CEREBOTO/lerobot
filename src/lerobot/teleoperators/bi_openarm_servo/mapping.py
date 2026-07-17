# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Map raw 8-channel servo angles to OpenArm follower joint actions (degrees)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .servo_reader import SERVO_COUNT

ARM_ORDER = ("left", "right")
JOINT_NAMES = [f"joint_{i}" for i in range(1, 8)]
DEFAULT_MAPPING_PATH = Path(__file__).with_name("joint_mapping_lerobot.json")


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def signed_angle_delta_deg(angle: float, reference: float) -> float:
    """Return the shortest signed angle delta, including across 0/360 degrees."""
    return (float(angle) - float(reference) + 180.0) % 360.0 - 180.0


def calibrated_trigger_ratio(
    angle: float,
    released_angle: float,
    pressed_span_deg: float,
    deadband_ratio: float,
) -> float:
    """Map a calibrated trigger angle to a clamped 0 (released) to 1 (pressed)."""
    if abs(pressed_span_deg) < 1e-9:
        raise ValueError("Trigger calibration span must be non-zero")
    ratio = clamp(
        signed_angle_delta_deg(angle, released_angle) / pressed_span_deg,
        0.0,
        1.0,
    )
    if ratio <= deadband_ratio:
        return 0.0
    return (ratio - deadband_ratio) / (1.0 - deadband_ratio)


def load_mapping(path: Path | None = None) -> dict[str, dict[str, Any]]:
    mapping_path = path or DEFAULT_MAPPING_PATH
    raw = json.loads(mapping_path.read_text(encoding="utf-8"))
    result: dict[str, dict[str, Any]] = {}
    for arm in ARM_ORDER:
        records = raw[arm]["joints"]
        by_name = {record["joint_name"]: record for record in records}
        if set(by_name) != set(JOINT_NAMES):
            raise ValueError(f"Invalid {arm} joint names in {mapping_path}")
        joints = []
        used_ids = []
        for name in JOINT_NAMES:
            record = by_name[name]
            servo_id = int(record["servo_id"])
            sign = int(record["sign"])
            if servo_id not in range(SERVO_COUNT) or sign not in (-1, 1):
                raise ValueError(f"Invalid servo_id/sign for {name}")
            joints.append(
                {
                    "name": name,
                    "servo_id": servo_id,
                    "sign": sign,
                    "scale": float(record["scale"]),
                    "home_deg": float(record["home_deg"]),
                    "max_delta_deg": float(record["max_delta_deg"]),
                }
            )
            used_ids.append(servo_id)

        gripper = raw[arm]["gripper"]
        gripper_id = int(gripper["servo_id"])
        if sorted(used_ids + [gripper_id]) != list(range(SERVO_COUNT)):
            raise ValueError(f"{arm} must use servo IDs 0-7 exactly once")

        open_deg = float(gripper.get("open_deg", 0.0))
        closed_deg = float(gripper.get("closed_deg", -65.0))
        if open_deg == closed_deg:
            raise ValueError(f"Invalid open/closed gripper degrees for {arm}")

        deadband_ratio = float(gripper.get("deadband_ratio", 0.05))
        minimum_travel_deg = float(gripper.get("minimum_travel_deg", 8.0))
        hold_seconds = float(gripper.get("hold_seconds", 0.75))
        stability_deg = float(gripper.get("stability_deg", 1.0))
        if not 0.0 <= deadband_ratio < 1.0:
            raise ValueError(f"Invalid deadband_ratio for {arm} gripper")
        if not 0.0 < minimum_travel_deg < 180.0:
            raise ValueError(f"Invalid minimum_travel_deg for {arm} gripper")
        if hold_seconds <= 0.0 or stability_deg <= 0.0:
            raise ValueError(f"Invalid hold/stability setting for {arm} gripper")

        result[arm] = {
            "joints": joints,
            "gripper": {
                "servo_id": gripper_id,
                "open_deg": open_deg,
                "closed_deg": closed_deg,
                "deadband_ratio": deadband_ratio,
                "minimum_travel_deg": minimum_travel_deg,
                "hold_seconds": hold_seconds,
                "stability_deg": stability_deg,
            },
        }
    return result


def arm_action_from_angles(
    arm: str,
    angles: list[float],
    zero: list[float],
    trigger_span: float,
    mapping: dict[str, dict[str, Any]],
) -> dict[str, float]:
    """Convert one arm's raw servo angles into follower action keys (degrees)."""
    arm_map = mapping[arm]
    action: dict[str, float] = {}
    for record in arm_map["joints"]:
        servo_id = record["servo_id"]
        delta = signed_angle_delta_deg(angles[servo_id], zero[servo_id])
        delta = clamp(
            record["sign"] * record["scale"] * delta,
            -record["max_delta_deg"],
            record["max_delta_deg"],
        )
        action[f"{record['name']}.pos"] = record["home_deg"] + delta

    gripper = arm_map["gripper"]
    pressed_ratio = calibrated_trigger_ratio(
        angles[gripper["servo_id"]],
        zero[gripper["servo_id"]],
        trigger_span,
        gripper["deadband_ratio"],
    )
    action["gripper.pos"] = gripper["open_deg"] + pressed_ratio * (
        gripper["closed_deg"] - gripper["open_deg"]
    )
    return action
