"""Season-specific task composition for a generic TaskPlanner."""

from typing import Any, Dict, List

from robot_application.task_definitions import Task, create_pick_place_task, create_return_base_task


class BaseSeasonTaskComposer:
    """Builds the initial task set for a match or planner reset."""

    def compose_initial_tasks(self, *, world_state, base_location) -> List[Task]:
        raise NotImplementedError


class PickPlaceSeasonTaskComposer(BaseSeasonTaskComposer):
    """Current-season composer for pick/place tasks plus return-to-base."""

    @staticmethod
    def _location_anchor(location: Dict[str, Any]) -> Dict[str, float]:
        pose = location.get('location')
        if isinstance(pose, dict) and pose:
            return pose

        approach_positions = location.get('approach_positions', [])
        if isinstance(approach_positions, list) and approach_positions:
            first_approach = approach_positions[0]
            if isinstance(first_approach, dict):
                return first_approach

        return {}

    def compose_initial_tasks(self, *, world_state, base_location) -> List[Task]:
        tasks: List[Task] = []
        pick_locations = world_state.all_pick_locations()
        available_drop_locations = list(world_state.all_drop_locations())

        sorted_picks = sorted(
            pick_locations,
            key=lambda pick: pick.get('priority', 0),
            reverse=True,
        )

        for pick in sorted_picks:
            if not available_drop_locations:
                break

            pick_anchor = self._location_anchor(pick)
            linked_drop = min(
                available_drop_locations,
                key=lambda drop: (
                    world_state.distance_between_locations(
                        pick_anchor,
                        self._location_anchor(drop),
                    ),
                    int(drop.get('priority', 999)),
                    str(drop.get('id', '')),
                ),
            )
            tasks.append(
                create_pick_place_task(
                    task_id=f"pick_place_{pick['id']}",
                    pick_location=pick,
                    drop_location=linked_drop,
                )
            )
            available_drop_locations = [
                drop for drop in available_drop_locations
                if str(drop.get('id', '')) != str(linked_drop.get('id', ''))
            ]

        tasks.append(create_return_base_task(base_location))
        return tasks
