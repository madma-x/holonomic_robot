"""Task definitions for strategic planning."""

from dataclasses import dataclass, field
from typing import Callable, List, Dict, Any
from enum import Enum
import math


class TaskType(Enum):
    """Types of tasks the robot can perform."""
    FLIP_OBJECT = "flip_object"
    MOVE_OBJECT = "move_object"
    RETURN_BASE = "return_base"


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
    parameters: Dict[str, Any] = field(default_factory=dict)
    
    
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
        priority_function=endgame_priority,
        parameters={'is_final_return': True}
    )


def create_pick_place_task(task_id: str,pick_location: Dict[str, Any],drop_location: Dict[str, Any]) -> Task:
        """Create a pick-place task using priority-ordered pick/drop lists."""

        return Task(
                task_id=task_id,
                task_type=TaskType.MOVE_OBJECT,
                name=f"PickPlace {task_id}",
                description='Select pick and drop targets by priority',
                time_estimate=18.0,
                success_probability=0.85,
                base_priority=7,
                parameters={
                        'pick_location': pick_location,
                        'drop_location': drop_location
                }
        )
