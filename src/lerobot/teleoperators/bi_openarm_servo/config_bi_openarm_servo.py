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

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@dataclass
class OpenArmServoArmConfig:
    """Per-arm serial port for a PWM servo leader."""

    port: str


@TeleoperatorConfig.register_subclass("bi_openarm_servo")
@dataclass
class BiOpenArmServoConfig(TeleoperatorConfig):
    """Configuration for dual 8-channel PWM servo leaders."""

    left_arm_config: OpenArmServoArmConfig
    right_arm_config: OpenArmServoArmConfig
    baudrate: int = 1_000_000
    release_torque: bool = True
    # None uses the packaged joint_mapping_lerobot.json.
    mapping_file: str | None = None
    signal_timeout: float = 0.5
    startup_delay: float = 0.2
    response_wait: float = 0.03
    torque_release_settle: float = 1.0
    calibration_seconds: float = 2.0
    trigger_calibration_timeout: float = 30.0
    # Interleave gripper reads between joints (fresher trigger, slower joints). Off by default.
    interleave_gripper: bool = False
    # Soften staircase targets: max degrees the published action may change per second.
    # Set <=0 to disable slew limiting.
    max_joint_delta_deg_per_sec: float = 720.0
    # Blend toward the latest leader sample each control tick (1.0 = no blend).
    action_blend: float = 0.55
    id: str | None = field(default="bi_openarm_servo", kw_only=True)
