"""Fixed arm mechanism layout for the pick/place system.

This module describes the physical mechanism as it really exists:
two shared lift groups and four end effectors.

Update the constants below if the hardware wiring changes.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class LiftGroupConfig:
    group_index: int
    lift_servo_id: int
    member_arm_indices: tuple[int, int]
    down_angle_deg: float
    up_angle_deg: float
    speed_deg_s: float
    pusher_pwm_channel: int
    pusher_push_deg: float
    pusher_stow_deg: float
    pusher_settle_sec: float


@dataclass(frozen=True)
class EndEffectorConfig:
    arm_index: int
    group_index: int
    pump_id: int
    pwm_channel: int
    pwm_pick_deg: float
    pwm_swap_deg: float
    pwm_place_deg: float
    pwm_stow_deg: float
    pwm_settle_sec: float


ARM_SELECTION_PRIORITY = (0, 1, 2, 3)


# Shared lift groups.
LIFT_GROUPS: dict[int, LiftGroupConfig] = {
    0: LiftGroupConfig(
        group_index=0,
        lift_servo_id=1,
        member_arm_indices=(0, 1),
        down_angle_deg=90.0,
        up_angle_deg=0.0,
        speed_deg_s=50.0,
        pusher_pwm_channel=4,
        pusher_push_deg=45.0,
        pusher_stow_deg=0.0,
        pusher_settle_sec=0.5,
    ),
    1: LiftGroupConfig(
        group_index=1,
        lift_servo_id=2,
        member_arm_indices=(2, 3),
        down_angle_deg=90.0,
        up_angle_deg=0.0,
        speed_deg_s=50.0,
        pusher_pwm_channel=5,
        pusher_push_deg=45.0,
        pusher_stow_deg=0.0,
        pusher_settle_sec=0.5,
    ),
}


# Individual end effectors sharing the lift groups above.
END_EFFECTORS: dict[int, EndEffectorConfig] = {
    0: EndEffectorConfig(
        arm_index=0,
        group_index=0,
        pump_id=0,
        pwm_channel=0,
        pwm_pick_deg=0.0,
        pwm_swap_deg=90.0,
        pwm_place_deg=0.0,
        pwm_stow_deg=0.0,
        pwm_settle_sec=0.5,
    ),
    1: EndEffectorConfig(
        arm_index=1,
        group_index=0,
        pump_id=1,
        pwm_channel=1,
        pwm_pick_deg=0.0,
        pwm_swap_deg=90.0,
        pwm_place_deg=0.0,
        pwm_stow_deg=0.0,
        pwm_settle_sec=0.5,
    ),
    2: EndEffectorConfig(
        arm_index=2,
        group_index=1,
        pump_id=2,
        pwm_channel=2,
        pwm_pick_deg=0.0,
        pwm_swap_deg=90.0,
        pwm_place_deg=0.0,
        pwm_stow_deg=0.0,
        pwm_settle_sec=0.5,
    ),
    3: EndEffectorConfig(
        arm_index=3,
        group_index=1,
        pump_id=3,
        pwm_channel=3,
        pwm_pick_deg=0.0,
        pwm_swap_deg=90.0,
        pwm_place_deg=0.0,
        pwm_stow_deg=0.0,
        pwm_settle_sec=0.5,
    ),
}


def get_end_effector_config(arm_index: int) -> EndEffectorConfig:
    try:
        return END_EFFECTORS[arm_index]
    except KeyError as exc:
        raise ValueError(f'Unknown arm_index {arm_index}') from exc


def get_lift_group_config(group_index: int) -> LiftGroupConfig:
    try:
        return LIFT_GROUPS[group_index]
    except KeyError as exc:
        raise ValueError(f'Unknown group_index {group_index}') from exc