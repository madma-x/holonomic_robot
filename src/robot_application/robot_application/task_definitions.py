"""Task definitions for strategic planning."""

from dataclasses import dataclass, field
from typing import Callable, List, Dict, Any
from enum import Enum
import math


class TaskType(Enum):
    """Types of tasks the robot can perform."""
    PICK_CUBE = "pick_cube"
    PLACE_CUBE = "place_cube"
    MOVE_OBJECT = "move_object"
    DEFEND_AREA = "defend_area"
    STEAL_CUBE = "steal_cube"
    RETURN_BASE = "return_base"
    PATROL = "patrol"
    WAIT = "wait"


class TaskStatus(Enum):
    """Task execution status."""
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELED = "canceled"


@dataclass
class Task:
    """Represents a task with priority and utility calculation."""
    
    # Basic properties
    task_id: str
    task_type: TaskType
    name: str
    description: str = ""
    
    # Scoring
    base_points: float = 10.0           # Base point value
    time_estimate: float = 15.0         # Estimated completion time (seconds)
    success_probability: float = 0.9     # Probability of success (0-1)
    
    # Priority calculation
    base_priority: int = 5              # Base priority (1-10, higher = more important)
    priority_function: Callable = None  # Custom priority calculation
    
    # Task parameters
    target_location: Dict[str, float] = field(default_factory=dict)  # {x, y, theta}
    parameters: Dict[str, Any] = field(default_factory=dict)
    
    # Execution
    behavior_tree: str = None           # Path to behavior tree XML
    mission_class: str = None           # Mission class name
    
    # State
    status: TaskStatus = TaskStatus.PENDING
    attempts: int = 0
    max_attempts: int = 3
    start_time: float = None
    
    def calculate_utility(self, time_remaining: float, current_score: int, 
                         game_state: Any) -> float:
        """
        Calculate task utility score for prioritization.
        
        Utility = (expected_points * success_prob * time_multiplier) / time_cost
        """
        if self.priority_function:
            return self.priority_function(time_remaining, current_score, game_state, self)
        
        # Default utility calculation
        time_multiplier = self._calculate_time_multiplier(time_remaining)
        expected_points = self.base_points * self.success_probability
        time_cost = max(1.0, self.time_estimate)  # Prevent division by zero
        
        utility = (expected_points * time_multiplier) / time_cost
        
        # Apply base priority multiplier
        utility *= (self.base_priority / 5.0)
        
        return utility
    
    def _calculate_time_multiplier(self, time_remaining: float) -> float:
        """Calculate time-based multiplier for urgency."""
        # If task can't be completed in time, heavily penalize
        if time_remaining < self.time_estimate:
            return 0.1
        
        # If barely enough time, slightly penalize
        if time_remaining < self.time_estimate * 1.5:
            return 0.7
        
        # Plenty of time
        return 1.0
    
    def can_execute(self, time_remaining: float) -> bool:
        """Check if task can be executed given time constraints."""
        return time_remaining >= self.time_estimate
    
    def is_available(self) -> bool:
        """Check if task is available for execution."""
        return (self.status == TaskStatus.PENDING and 
                self.attempts < self.max_attempts)


# Predefined priority functions for different game phases

def early_game_priority(time_remaining: float, current_score: int, 
                       game_state: Any, task: Task) -> float:
    """Priority function for early game (>60s): Focus on high-value tasks."""
    base_utility = (task.base_points * task.success_probability) / task.time_estimate
    
    # Bonus for high-value tasks
    if task.base_points > 20:
        base_utility *= 1.5
    
    # Penalty for risky tasks early on
    if task.success_probability < 0.7:
        base_utility *= 0.8
    
    return base_utility


def mid_game_priority(time_remaining: float, current_score: int, 
                     game_state: Any, task: Task) -> float:
    """Priority function for mid game (30-60s): Balanced approach."""
    base_utility = (task.base_points * task.success_probability) / task.time_estimate
    
    # If behind in score, take more risks
    score_diff = current_score - game_state.opponent_score
    if score_diff < -10:
        if task.base_points > 15:
            base_utility *= 1.3  # Go for high value
    
    return base_utility


def late_game_priority(time_remaining: float, current_score: int, 
                      game_state: Any, task: Task) -> float:
    """Priority function for late game (10-30s): Quick wins."""
    # Heavily favor quick tasks
    time_penalty = task.time_estimate / time_remaining
    base_utility = (task.base_points * task.success_probability) / (task.time_estimate ** 1.5)
    
    # Avoid tasks that might not complete
    if task.time_estimate > time_remaining * 0.6:
        return 0.01
    
    return base_utility * (1.0 - time_penalty)


def endgame_priority(time_remaining: float, current_score: int, 
                    game_state: Any, task: Task) -> float:
    """Priority function for endgame (<10s): Return to base, secure score."""
    # Massively prioritize return to base
    if task.task_type == TaskType.RETURN_BASE:
        return 1000.0
    
    # Only consider very quick tasks
    if task.time_estimate > time_remaining * 0.5:
        return 0.0
    
    # Quick opportunistic tasks only
    if task.time_estimate < 3.0 and task.success_probability > 0.95:
        return task.base_points * 2.0
    
    return 0.0


# Task factory functions

def create_pick_cube_task(cube_id: str, location: Dict[str, float], 
                         points: float = 10.0) -> Task:
    """Create a pick cube task."""
    return Task(
        task_id=f"pick_cube_{cube_id}",
        task_type=TaskType.PICK_CUBE,
        name=f"Pick Cube {cube_id}",
        description=f"Pick up cube {cube_id} from field",
        base_points=points,
        time_estimate=12.0,
        success_probability=0.85,
        base_priority=6,
        target_location=location,
        behavior_tree="pick_place.xml",
        parameters={'cube_id': cube_id, 'method': 'gripper'}
    )


def create_place_cube_task(cube_id: str, location: Dict[str, float], 
                          points: float = 15.0) -> Task:
    """Create a place cube task."""
    return Task(
        task_id=f"place_cube_{cube_id}",
        task_type=TaskType.PLACE_CUBE,
        name=f"Place Cube {cube_id}",
        description=f"Place cube {cube_id} in scoring zone",
        base_points=points,
        time_estimate=10.0,
        success_probability=0.90,
        base_priority=7,
        target_location=location,
        behavior_tree="pick_place.xml",
        parameters={'cube_id': cube_id}
    )


def create_defend_task(area_id: str, location: Dict[str, float], 
                      duration: float = 20.0) -> Task:
    """Create a defense task."""
    return Task(
        task_id=f"defend_{area_id}",
        task_type=TaskType.DEFEND_AREA,
        name=f"Defend {area_id}",
        description=f"Defend area {area_id} from opponents",
        base_points=5.0,  # Defensive bonus
        time_estimate=duration,
        success_probability=0.95,
        base_priority=4,
        target_location=location,
        behavior_tree="patrol.xml",
        parameters={'area_id': area_id, 'duration': duration}
    )


def create_steal_cube_task(cube_id: str, location: Dict[str, float], 
                          points: float = 20.0) -> Task:
    """Create a steal cube task (high risk, high reward)."""
    return Task(
        task_id=f"steal_cube_{cube_id}",
        task_type=TaskType.STEAL_CUBE,
        name=f"Steal Cube {cube_id}",
        description=f"Steal cube {cube_id} from opponent",
        base_points=points,
        time_estimate=18.0,
        success_probability=0.60,  # Riskier
        base_priority=8,
        target_location=location,
        behavior_tree="pick_place.xml",
        parameters={'cube_id': cube_id, 'method': 'vacuum', 'aggressive': True}
    )


def create_return_base_task(base_location: Dict[str, float]) -> Task:
    """Create return to base task."""
    return Task(
        task_id="return_base",
        task_type=TaskType.RETURN_BASE,
        name="Return to Base",
        description="Return to home base before match ends",
        base_points=10.0,  # End game bonus
        time_estimate=8.0,
        success_probability=0.98,
        base_priority=10,  # Highest priority in endgame
        target_location=base_location,
        behavior_tree="patrol.xml",
        priority_function=endgame_priority,
        parameters={'is_final_return': True}
    )


def create_move_object_task(object_id: str, from_loc: Dict[str, float], 
                           to_loc: Dict[str, float], points: float = 12.0) -> Task:
    """Create a move object task."""
    return Task(
        task_id=f"move_object_{object_id}",
        task_type=TaskType.MOVE_OBJECT,
        name=f"Move Object {object_id}",
        description=f"Move object {object_id} to new location",
        base_points=points,
        time_estimate=20.0,
        success_probability=0.80,
        base_priority=5,
        target_location=from_loc,
        behavior_tree="pick_place.xml",
        parameters={
            'object_id': object_id,
            'from_location': from_loc,
            'to_location': to_loc
        }
    )
