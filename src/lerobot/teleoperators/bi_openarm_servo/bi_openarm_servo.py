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

"""Bimanual PWM servo leader teleoperator for OpenArm followers."""

from __future__ import annotations

import json
import logging
import statistics
import threading
import time
from collections import deque
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.types import RobotAction
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_bi_openarm_servo import BiOpenArmServoConfig
from .mapping import (
    ARM_ORDER,
    JOINT_NAMES,
    arm_action_from_angles,
    load_mapping,
    signed_angle_delta_deg,
)
from .servo_reader import SERVO_COUNT, ArmReader, ArmState

logger = logging.getLogger(__name__)


class BiOpenArmServo(Teleoperator):
    """Dual 8-DOF PWM servo leaders mapped to OpenArm follower joint actions."""

    config_class = BiOpenArmServoConfig
    name = "bi_openarm_servo"

    def __init__(self, config: BiOpenArmServoConfig):
        # Custom calibration JSON (zero + trigger spans), so skip Teleoperator's
        # MotorCalibration loader by not calling super().__init__.
        self.id = config.id
        self.calibration_dir = (
            config.calibration_dir
            if config.calibration_dir
            else HF_LEROBOT_CALIBRATION / TELEOPERATORS / self.name
        )
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
        self.calibration: dict[str, Any] = {}

        self.config = config
        mapping_path = Path(config.mapping_file) if config.mapping_file else None
        self.mapping = load_mapping(mapping_path)

        self._stop_event = threading.Event()
        self._states = {arm: ArmState() for arm in ARM_ORDER}
        self._readers: list[ArmReader] = []
        self._connected = False
        self._last_action: RobotAction | None = None
        self._smoothed_action: RobotAction | None = None
        self._last_action_time: float | None = None
        self._zero: dict[str, list[float]] | None = None
        self._trigger_spans: dict[str, float] | None = None

        if self.calibration_fpath.is_file():
            self._load_calibration()

    @cached_property
    def action_features(self) -> dict[str, type]:
        keys = [f"{name}.pos" for name in JOINT_NAMES] + ["gripper.pos"]
        return {
            **{f"left_{k}": float for k in keys},
            **{f"right_{k}": float for k in keys},
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return (
            self._zero is not None
            and self._trigger_spans is not None
            and set(self._zero) == set(ARM_ORDER)
            and set(self._trigger_spans) == set(ARM_ORDER)
        )

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        data = json.loads(fpath.read_text(encoding="utf-8"))
        zero = data.get("zero", {})
        spans = data.get("trigger_spans", {})
        if set(zero) != set(ARM_ORDER) or set(spans) != set(ARM_ORDER):
            logger.warning("Ignoring incomplete calibration file at %s", fpath)
            return
        for arm in ARM_ORDER:
            if len(zero[arm]) != SERVO_COUNT:
                logger.warning("Ignoring calibration with wrong zero length for %s", arm)
                return
        self._zero = {arm: [float(v) for v in zero[arm]] for arm in ARM_ORDER}
        self._trigger_spans = {arm: float(spans[arm]) for arm in ARM_ORDER}
        self.calibration = data

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        if self._zero is None or self._trigger_spans is None:
            raise RuntimeError("Cannot save calibration before zero/trigger spans are collected")
        data = {
            "zero": self._zero,
            "trigger_spans": self._trigger_spans,
        }
        fpath.write_text(json.dumps(data, indent=4) + "\n", encoding="utf-8")
        self.calibration = data
        logger.info("Calibration saved to %s", fpath)

    def configure(self) -> None:
        return

    def setup_motors(self) -> None:
        logger.info("%s has no motor setup step (passive PWM leaders).", self)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return

    def _check_reader_failures(self) -> None:
        failures = [reader for reader in self._readers if reader.startup_error is not None]
        if failures:
            details = "; ".join(f"{r.arm_name} ({r.port}): {r.startup_error}" for r in failures)
            raise RuntimeError(f"Servo reader failed: {details}")

    def _snapshot(self, arm: str) -> tuple[list[float] | None, int, float]:
        angles, scans, _, updated = self._states[arm].snapshot()
        if any(value is None for value in angles):
            return None, scans, updated
        return [float(value) for value in angles], scans, updated

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        left_port = self.config.left_arm_config.port
        right_port = self.config.right_arm_config.port
        if left_port.upper() == right_port.upper():
            raise ValueError("Left and right ports must be different.")

        self._stop_event.clear()
        self._readers = [
            ArmReader(
                "Left",
                left_port,
                self.config.baudrate,
                self.config.startup_delay,
                self.config.response_wait,
                self._states["left"],
                self._stop_event,
                self.config.release_torque,
                self.config.torque_release_settle,
                self.config.interleave_gripper,
            ),
            ArmReader(
                "Right",
                right_port,
                self.config.baudrate,
                self.config.startup_delay,
                self.config.response_wait,
                self._states["right"],
                self._stop_event,
                self.config.release_torque,
                self.config.torque_release_settle,
                self.config.interleave_gripper,
            ),
        ]
        for reader in self._readers:
            reader.start()

        # Wait until both arms produce at least one full valid scan.
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self._check_reader_failures()
            ready = True
            for arm in ARM_ORDER:
                angles, _, updated = self._snapshot(arm)
                if angles is None or updated <= 0:
                    ready = False
                    break
            if ready:
                break
            time.sleep(0.05)
        else:
            self.disconnect()
            raise TimeoutError("Timed out waiting for both servo leaders to produce valid scans")

        self._connected = True
        logger.info("%s connected (left=%s, right=%s).", self, left_port, right_port)

        if calibrate and not self.is_calibrated:
            self.calibrate()
        elif calibrate and self.is_calibrated:
            user_input = input(
                f"Press ENTER to use existing calibration for {self.id}, or type 'c' and ENTER to recalibrate: "
            )
            if user_input.strip().lower() == "c":
                self.calibrate()

    @check_if_not_connected
    def calibrate(self) -> None:
        print(
            "Keep both leader arms still in the hanging/zero pose with both triggers fully released."
        )
        self._zero = self._collect_zero(self.config.calibration_seconds)
        print("Press both triggers fully and hold them still for calibration.")
        self._trigger_spans = self._collect_trigger_spans(self.config.trigger_calibration_timeout)
        self._save_calibration()
        print(f"Calibration complete and saved to {self.calibration_fpath}")

    def _collect_zero(self, seconds: float) -> dict[str, list[float]]:
        samples: dict[str, list[list[float]]] = {arm: [] for arm in ARM_ORDER}
        last_scan = {arm: -1 for arm in ARM_ORDER}
        valid_since = None
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            self._check_reader_failures()
            all_valid = True
            for arm in ARM_ORDER:
                angles, scan, updated = self._snapshot(arm)
                if angles is None or time.monotonic() - updated > self.config.signal_timeout:
                    all_valid = False
                    continue
                if scan != last_scan[arm]:
                    samples[arm].append(angles)
                    last_scan[arm] = scan
            if all_valid:
                valid_since = valid_since or time.monotonic()
                if time.monotonic() - valid_since >= seconds:
                    return {
                        arm: [statistics.median(channel) for channel in zip(*samples[arm], strict=True)]
                        for arm in ARM_ORDER
                    }
            else:
                valid_since = None
            time.sleep(0.01)
        raise TimeoutError("Timed out waiting for both 8-channel leaders during zero calibration")

    def _collect_trigger_spans(self, timeout: float) -> dict[str, float]:
        if self._zero is None:
            raise RuntimeError("Zero calibration must be collected before trigger spans")

        recent: dict[str, deque[tuple[float, float]]] = {arm: deque() for arm in ARM_ORDER}
        last_scan = {arm: -1 for arm in ARM_ORDER}
        spans: dict[str, float] = {}
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            self._check_reader_failures()
            now = time.monotonic()
            for arm in ARM_ORDER:
                if arm in spans:
                    continue
                angles, scan, updated = self._snapshot(arm)
                if angles is None or now - updated > self.config.signal_timeout or scan == last_scan[arm]:
                    continue
                last_scan[arm] = scan
                gripper = self.mapping[arm]["gripper"]
                servo_id = gripper["servo_id"]
                travel = signed_angle_delta_deg(angles[servo_id], self._zero[arm][servo_id])
                samples = recent[arm]
                if abs(travel) < gripper["minimum_travel_deg"]:
                    samples.clear()
                    continue

                samples.append((now, travel))
                hold_seconds = gripper["hold_seconds"]
                while samples and now - samples[0][0] > hold_seconds:
                    samples.popleft()
                if len(samples) < 3 or samples[-1][0] - samples[0][0] < hold_seconds * 0.9:
                    continue
                values = [sample[1] for sample in samples]
                if max(values) - min(values) > gripper["stability_deg"]:
                    continue

                spans[arm] = statistics.median(values)
                direction = "increases" if spans[arm] > 0.0 else "decreases"
                logger.info(
                    "%s trigger calibrated: angle %s by %.2f deg",
                    arm.capitalize(),
                    direction,
                    abs(spans[arm]),
                )

            if len(spans) == len(ARM_ORDER):
                print("Trigger calibration complete; release the triggers.")
                return spans
            time.sleep(0.01)

        missing = ", ".join(arm for arm in ARM_ORDER if arm not in spans)
        raise TimeoutError(f"Timed out calibrating fully pressed triggers: {missing}")

    def _slew_limit_action(self, target: RobotAction, now: float) -> RobotAction:
        """Blend + slew-limit so sparse leader samples do not snap the follower."""
        if self._smoothed_action is None or self._last_action_time is None:
            self._smoothed_action = dict(target)
            self._last_action_time = now
            return dict(target)

        dt = max(now - self._last_action_time, 1e-3)
        blend = float(self.config.action_blend)
        blend = min(max(blend, 0.05), 1.0)
        max_rate = self.config.max_joint_delta_deg_per_sec
        max_delta = None if max_rate is None or max_rate <= 0 else max_rate * dt

        smoothed: RobotAction = {}
        for key, value in target.items():
            prev = float(self._smoothed_action.get(key, value))
            blended = prev + blend * (float(value) - prev)
            if max_delta is not None:
                delta = blended - prev
                if delta > max_delta:
                    blended = prev + max_delta
                elif delta < -max_delta:
                    blended = prev - max_delta
            smoothed[key] = blended
        self._smoothed_action = smoothed
        self._last_action_time = now
        return smoothed

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        if not self.is_calibrated or self._zero is None or self._trigger_spans is None:
            raise RuntimeError(f"{self} is connected but not calibrated. Run calibrate() first.")

        self._check_reader_failures()
        now = time.monotonic()
        snapshots: dict[str, list[float]] = {}
        for arm in ARM_ORDER:
            angles, _, updated = self._snapshot(arm)
            age = now - updated if updated else float("inf")
            if angles is None or age > self.config.signal_timeout:
                if self._smoothed_action is not None:
                    return self._smoothed_action
                if self._last_action is not None:
                    return self._last_action
                raise TimeoutError(f"No valid servo data for {arm} (age={age:.3f}s)")
            snapshots[arm] = angles

        action: RobotAction = {}
        for arm in ARM_ORDER:
            arm_action = arm_action_from_angles(
                arm,
                snapshots[arm],
                self._zero[arm],
                self._trigger_spans[arm],
                self.mapping,
            )
            for key, value in arm_action.items():
                action[f"{arm}_{key}"] = value

        self._last_action = action
        return self._slew_limit_action(action, now)

    def disconnect(self) -> None:
        self._stop_event.set()
        for reader in self._readers:
            reader.join(timeout=1.0)
        self._readers = []
        was_connected = self._connected
        self._connected = False
        self._smoothed_action = None
        self._last_action_time = None
        if was_connected:
            logger.info("%s disconnected.", self)
