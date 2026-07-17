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

"""Background serial readers for 8-channel PWM servo leaders."""

from __future__ import annotations

import logging
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Pattern

import serial

logger = logging.getLogger(__name__)

SERVO_COUNT = 8
GRIPPER_SERVO_ID = 7
PWM_PATTERN = re.compile(r"P(\d{4})")


def build_read_order(gripper_id: int = GRIPPER_SERVO_ID) -> list[int]:
    """Interleave gripper reads so the trigger updates more often (slows joint refresh)."""
    order: list[int] = []
    for servo_id in range(SERVO_COUNT):
        if servo_id == gripper_id:
            continue
        order.append(servo_id)
        order.append(gripper_id)
    return order


def send_query(
    ser: serial.Serial,
    command: str,
    response_wait: float,
    expected_pattern: Pattern[str] | None = None,
    *,
    reset_buffer: bool = False,
) -> str:
    """Send a command and collect bytes until the expected reply or timeout."""
    if reset_buffer:
        ser.reset_input_buffer()
    ser.write(command.encode("ascii"))
    ser.flush()
    deadline = time.perf_counter() + response_wait
    response = bytearray()
    decoded = ""
    while time.perf_counter() < deadline:
        waiting = ser.in_waiting
        if waiting:
            response.extend(ser.read(waiting))
            decoded = response.decode("ascii", errors="ignore")
            if expected_pattern is None or expected_pattern.search(decoded):
                break
        else:
            # Brief yield; keep this tiny so fast servos are not artificially slowed.
            time.sleep(0.00005)
    return decoded


def pwm_to_angle(
    response: str,
    pwm_min: int = 500,
    pwm_max: int = 2500,
    angle_range: float = 270.0,
) -> float | None:
    match = PWM_PATTERN.search(response)
    if not match:
        return None

    pwm = int(match.group(1))
    if not pwm_min <= pwm <= pwm_max:
        return None
    return (pwm - pwm_min) / (pwm_max - pwm_min) * angle_range


@dataclass
class ArmState:
    angles: list[float | None] = field(default_factory=lambda: [None] * SERVO_COUNT)
    scans: int = 0
    errors: int = 0
    frequency_hz: float = 0.0
    channel_update_hz: float = 0.0
    last_update: float = 0.0
    torque_release_complete: bool = False
    torque_release_responses: list[str] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def snapshot(self) -> tuple[list[float | None], int, float, float]:
        with self.lock:
            return (
                self.angles.copy(),
                self.scans,
                self.frequency_hz,
                self.last_update,
            )


class ArmReader(threading.Thread):
    def __init__(
        self,
        name: str,
        port: str,
        baudrate: int,
        startup_delay: float,
        response_wait: float,
        state: ArmState,
        stop_event: threading.Event,
        release_torque: bool = True,
        torque_release_settle: float = 1.0,
        interleave_gripper: bool = False,
    ) -> None:
        super().__init__(name=f"{name}-reader", daemon=True)
        self.arm_name = name
        self.port = port
        self.baudrate = baudrate
        self.startup_delay = startup_delay
        self.response_wait = response_wait
        self.state = state
        self.stop_event = stop_event
        self.release_torque = release_torque
        self.torque_release_settle = torque_release_settle
        self.read_order = (
            build_read_order() if interleave_gripper else list(range(SERVO_COUNT))
        )
        self.startup_error: Exception | None = None

    def run(self) -> None:
        try:
            with serial.Serial(
                self.port,
                self.baudrate,
                timeout=0,
                write_timeout=0.1,
            ) as ser:
                time.sleep(self.startup_delay)
                ser.reset_input_buffer()
                if self.release_torque:
                    responses = []
                    for servo_id in range(SERVO_COUNT):
                        response = send_query(
                            ser,
                            f"#00{servo_id}PULK!",
                            self.response_wait,
                            reset_buffer=True,
                        )
                        responses.append(response.strip())
                    with self.state.lock:
                        self.state.torque_release_responses = responses
                        self.state.torque_release_complete = True
                    time.sleep(self.torque_release_settle)
                    ser.reset_input_buffer()
                    logger.info("%s: torque release sent to 8 servos", self.arm_name)

                previous_channel = time.perf_counter()
                previous_cycle = previous_channel
                cycle_reads = 0
                while not self.stop_event.is_set():
                    for servo_id in self.read_order:
                        if self.stop_event.is_set():
                            return
                        # Only reset buffer on the first read of a cycle to drop stale bytes
                        # without paying reset cost on every servo.
                        response = send_query(
                            ser,
                            f"#00{servo_id}PRAD!",
                            self.response_wait,
                            PWM_PATTERN,
                            reset_buffer=(cycle_reads == 0),
                        )
                        angle = pwm_to_angle(response)
                        now = time.perf_counter()
                        channel_elapsed = now - previous_channel
                        previous_channel = now
                        cycle_reads += 1

                        with self.state.lock:
                            if angle is not None:
                                self.state.angles[servo_id] = angle
                            else:
                                self.state.errors += 1
                            self.state.last_update = time.monotonic()
                            self.state.channel_update_hz = (
                                1.0 / channel_elapsed if channel_elapsed > 0 else 0.0
                            )
                            if cycle_reads >= len(self.read_order):
                                cycle_elapsed = now - previous_cycle
                                previous_cycle = now
                                cycle_reads = 0
                                self.state.scans += 1
                                self.state.frequency_hz = (
                                    1.0 / cycle_elapsed if cycle_elapsed > 0 else 0.0
                                )
                                logger.debug(
                                    "%s scan=%.1fHz channel=%.1fHz",
                                    self.arm_name,
                                    self.state.frequency_hz,
                                    self.state.channel_update_hz,
                                )
        except (serial.SerialException, OSError) as exc:
            self.startup_error = exc
            self.stop_event.set()
