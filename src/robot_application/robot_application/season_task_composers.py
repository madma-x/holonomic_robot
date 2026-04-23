"""Season-specific task composition for a generic TaskPlanner."""

from typing import List

from robot_application.task_definitions import Task, create_pick_place_task, create_return_base_task


class BaseSeasonTaskComposer:
    """Builds the initial task set for a match or planner reset."""

    def compose_initial_tasks(self, *, world_state, base_location) -> List[Task]:
        raise NotImplementedError


class PickPlaceSeasonTaskComposer(BaseSeasonTaskComposer):
    """Current-season composer for pick/place tasks plus return-to-base."""

    def compose_initial_tasks(self, *, world_state, base_location) -> List[Task]:
        tasks: List[Task] = []
        pick_locations = world_state.all_pick_locations()
        drop_locations = world_state.all_drop_locations()

        sorted_picks = sorted(
            pick_locations,
            key=lambda pick: pick.get('priority', 0),
            reverse=True,
        )

        for pick in sorted_picks:
            linked_drop = min(
                drop_locations,
                key=lambda drop: world_state.distance_between_locations(
                    pick.get('location', {}),
                    drop.get('location', {}),
                ),
            )
            tasks.append(
                create_pick_place_task(
                    task_id=f"pick_place_{pick['id']}",
                    pick_location=pick,
                    drop_location=linked_drop,
                )
            )

        tasks.append(create_return_base_task(base_location))
        return tasks
