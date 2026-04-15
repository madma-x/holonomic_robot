"""Sequence building for the fixed shared-lift arm mechanism."""

from typing import Iterable, Optional

from aruco_interfaces.msg import ArmAssignment, ClusterPickability
from robot_application.arm_layout import (
    ARM_SELECTION_PRIORITY,
    EndEffectorConfig,
    LiftGroupConfig,
    get_end_effector_config,
    get_lift_group_config,
)

try:
    from robot_actuators.msg import ActuatorStep
    _HAS_ACTUATOR_STEP = True
except ImportError:
    ActuatorStep = None
    _HAS_ACTUATOR_STEP = False


class ArmSequenceBuilder:
    """Build pick/place sequences for a fixed two-group arm mechanism."""

    def __init__(self):
        self._selection_priority = tuple(ARM_SELECTION_PRIORITY)

    def select_arm_indices(self, pickability: ClusterPickability) -> list[int]:
        assigned_arm_indices = {
            int(getattr(arm, 'arm_index', -1))
            for arm in pickability.arms
            if bool(getattr(arm, 'assigned', False))
        }
        return [
            arm_index
            for arm_index in self._selection_priority
            if arm_index in assigned_arm_indices
        ]

    def select_arm_index(self, pickability: ClusterPickability) -> Optional[int]:
        arm_indices = self.select_arm_indices(pickability)
        return arm_indices[0] if arm_indices else None

    def get_assigned_arm(
        self, pickability: ClusterPickability, arm_index: int
    ) -> Optional[ArmAssignment]:
        for arm in pickability.arms:
            if (
                int(getattr(arm, 'arm_index', -1)) == arm_index
                and bool(getattr(arm, 'assigned', False))
            ):
                return arm
        return None

    def get_end_effector(self, arm_index: int) -> EndEffectorConfig:
        return get_end_effector_config(arm_index)

    def get_lift_group(self, arm_index: int) -> LiftGroupConfig:
        end_effector = self.get_end_effector(arm_index)
        return get_lift_group_config(end_effector.group_index)

    def build_pick_sequence(self, arm_indices: Iterable[int]) -> list:
        if not _HAS_ACTUATOR_STEP:
            raise RuntimeError('ActuatorStep message unavailable')

        end_effectors = self._resolve_end_effectors(arm_indices)
        if not end_effectors:
            return []
        return [
            *self._build_lower_steps(end_effectors, use_place_pose=False),
            *self._build_pump_steps(end_effectors, enable=True, parallel_group=2),
            *self._build_raise_steps(end_effectors, parallel_group=3),
        ]

    def build_swap_sequence(self, arm_indices: Iterable[int]) -> list:
        if not _HAS_ACTUATOR_STEP:
            raise RuntimeError('ActuatorStep message unavailable')

        end_effectors = self._resolve_end_effectors(arm_indices)
        swap_targets = [eff for eff in end_effectors if eff.pwm_channel >= 0]
        if not swap_targets:
            return []
        return [
            *self._build_pwm_steps(swap_targets, target='swap', parallel_group=1),
            *self._build_pwm_steps(swap_targets, target='stow', parallel_group=2),
        ]

    def build_place_sequence(
        self,
        arm_indices: Iterable[int],
        push_arm_indices: Optional[Iterable[int]] = None,
    ) -> list:
        return self.build_drop_sequence(arm_indices, push_arm_indices=push_arm_indices)

    def build_drop_sequence(
        self,
        arm_indices: Iterable[int],
        push_arm_indices: Optional[Iterable[int]] = None,
    ) -> list:
        if not _HAS_ACTUATOR_STEP:
            raise RuntimeError('ActuatorStep message unavailable')

        end_effectors = self._resolve_end_effectors(arm_indices)
        if not end_effectors:
            return []

        pusher_groups = self._resolve_pusher_groups(end_effectors, push_arm_indices)

        return [
            *self._build_lower_steps(end_effectors, use_place_pose=True),
            *self._build_pump_steps(end_effectors, enable=False, parallel_group=2),
            *self._build_group_pusher_steps(pusher_groups, target='push', parallel_group=3),
            *self._build_raise_steps(end_effectors, parallel_group=4),
            *self._build_group_pusher_steps(pusher_groups, target='stow', parallel_group=4),
        ]

    def _resolve_end_effectors(self, arm_indices: Iterable[int]) -> list[EndEffectorConfig]:
        unique_arm_indices = []
        seen = set()
        for arm_index in arm_indices:
            normalized_arm_index = int(arm_index)
            if normalized_arm_index not in seen:
                unique_arm_indices.append(normalized_arm_index)
                seen.add(normalized_arm_index)
        return [self.get_end_effector(arm_index) for arm_index in unique_arm_indices]

    def _resolve_pusher_groups(
        self,
        end_effectors: list[EndEffectorConfig],
        push_arm_indices: Optional[Iterable[int]],
    ) -> list[LiftGroupConfig]:
        if push_arm_indices is None:
            return []

        requested_group_indices = {
            self.get_end_effector(arm_index).group_index
            for arm_index in push_arm_indices
        }
        active_group_indices = {eff.group_index for eff in end_effectors}
        return [
            get_lift_group_config(group_index)
            for group_index in sorted(requested_group_indices & active_group_indices)
        ]

    def _build_lower_steps(
        self,
        end_effectors: list[EndEffectorConfig],
        use_place_pose: bool,
    ) -> list:
        steps = []
        for lift_group in self._unique_groups(end_effectors):
            steps.append(
                self._make_move_step(
                    lift_group.lift_servo_id,
                    lift_group.down_angle_deg,
                    lift_group.speed_deg_s,
                    parallel_group=1,
                )
            )
        target_name = 'place' if use_place_pose else 'pick'
        steps.extend(self._build_pwm_steps(end_effectors, target=target_name, parallel_group=1))
        return steps

    def _build_raise_steps(
        self,
        end_effectors: list[EndEffectorConfig],
        parallel_group: int,
        stow_end_effectors: Optional[list[EndEffectorConfig]] = None,
    ) -> list:
        steps = []
        for lift_group in self._unique_groups(end_effectors):
            steps.append(
                self._make_move_step(
                    lift_group.lift_servo_id,
                    lift_group.up_angle_deg,
                    lift_group.speed_deg_s,
                    parallel_group=parallel_group,
                )
            )
        steps.extend(
            self._build_pwm_steps(
                stow_end_effectors or end_effectors,
                target='stow',
                parallel_group=parallel_group,
            )
        )
        return steps

    def _build_pump_steps(
        self,
        end_effectors: list[EndEffectorConfig],
        enable: bool,
        parallel_group: int,
    ) -> list:
        return [
            self._make_pump_step(
                end_effector.pump_id,
                enable=enable,
                parallel_group=parallel_group,
            )
            for end_effector in end_effectors
        ]

    def _build_group_pusher_steps(
        self,
        lift_groups: list[LiftGroupConfig],
        target: str,
        parallel_group: int,
    ) -> list:
        steps = []
        for lift_group in lift_groups:
            if lift_group.pusher_pwm_channel < 0:
                continue
            pwm_target = (
                lift_group.pusher_push_deg
                if target == 'push'
                else lift_group.pusher_stow_deg
            )
            steps.append(
                self._make_pwm_servo_step(
                    lift_group.pusher_pwm_channel,
                    pwm_target,
                    settle_sec=lift_group.pusher_settle_sec,
                    parallel_group=parallel_group,
                )
            )
        return steps

    def _build_pwm_steps(
        self,
        end_effectors: list[EndEffectorConfig],
        target: str,
        parallel_group: int,
    ) -> list:
        steps = []
        for end_effector in end_effectors:
            if end_effector.pwm_channel < 0:
                continue
            steps.append(
                self._make_pwm_servo_step(
                    end_effector.pwm_channel,
                    self._pwm_target_deg(end_effector, target),
                    settle_sec=end_effector.pwm_settle_sec,
                    parallel_group=parallel_group,
                )
            )
        return steps

    def _unique_groups(self, end_effectors: list[EndEffectorConfig]) -> list[LiftGroupConfig]:
        seen = set()
        groups = []
        for end_effector in end_effectors:
            if end_effector.group_index in seen:
                continue
            groups.append(get_lift_group_config(end_effector.group_index))
            seen.add(end_effector.group_index)
        return groups

    def _pwm_target_deg(self, end_effector: EndEffectorConfig, target: str) -> float:
        if target == 'pick':
            return end_effector.pwm_pick_deg
        if target == 'swap':
            return end_effector.pwm_swap_deg
        if target == 'place':
            return end_effector.pwm_place_deg
        if target == 'stow':
            return end_effector.pwm_stow_deg
        raise ValueError(f'Unknown PWM target {target}')

    def _make_move_step(
        self, servo_id: int, target_deg: float, speed_deg_s: float,
        parallel_group: int = 0,
    ):
        step = ActuatorStep()
        step.step_type = ActuatorStep.MOVE_SERVO
        step.servo_id = servo_id
        step.target_deg = target_deg
        step.speed_deg_s = speed_deg_s
        step.parallel_group = parallel_group
        return step

    def _make_pump_step(
        self, pump_id: int, enable: bool, duration_sec: float = 0.0,
        parallel_group: int = 0,
    ):
        step = ActuatorStep()
        step.step_type = ActuatorStep.CONTROL_PUMP
        step.pump_id = pump_id
        step.pump_enable = enable
        step.pump_duration_sec = duration_sec
        step.parallel_group = parallel_group
        return step

    def _make_pwm_servo_step(
        self, channel: int, target_deg: float, settle_sec: float = 0.5,
        parallel_group: int = 0,
    ):
        step = ActuatorStep()
        step.step_type = ActuatorStep.MOVE_PWM_SERVO
        step.pwm_channel = channel
        step.pwm_target_deg = target_deg
        step.pwm_settle_sec = settle_sec
        step.parallel_group = parallel_group
        return step