from robot_application.season_task_composers import PickPlaceSeasonTaskComposer
from robot_application.task_definitions import TaskType


class FakeWorldState:
    def all_pick_locations(self):
        return [
            {
                'id': 'pick_low',
                'priority': 10,
                'approach_positions': [{'x': 0.0, 'y': 0.0, 'theta': 0.0}],
            },
            {
                'id': 'pick_high',
                'priority': 100,
                'approach_positions': [{'x': 3.0, 'y': 0.0, 'theta': 0.0}],
            },
        ]

    def all_drop_locations(self):
        return [
            {
                'id': 'drop_near_high',
                'approach_positions': [{'x': 3.1, 'y': 0.0, 'theta': 0.0}],
                'priority': 1,
            },
            {
                'id': 'drop_near_low',
                'approach_positions': [{'x': 0.2, 'y': 0.0, 'theta': 0.0}],
                'priority': 1,
            },
        ]

    @staticmethod
    def distance_between_locations(from_location, to_location):
        dx = float(from_location.get('x', 0.0)) - float(to_location.get('x', 0.0))
        dy = float(from_location.get('y', 0.0)) - float(to_location.get('y', 0.0))
        return (dx * dx + dy * dy) ** 0.5


def test_pick_place_season_composer_builds_prioritized_tasks_and_unique_drops():
    composer = PickPlaceSeasonTaskComposer()
    tasks = composer.compose_initial_tasks(
        world_state=FakeWorldState(),
        base_location={'x': 1.0, 'y': 2.0, 'theta': 0.5},
    )

    assert [task.task_id for task in tasks] == [
        'pick_place_pick_high',
        'pick_place_pick_low',
        'return_base',
    ]
    assert tasks[0].parameters['drop_location']['id'] == 'drop_near_high'
    assert tasks[1].parameters['drop_location']['id'] == 'drop_near_low'
    assert tasks[0].parameters['drop_location']['id'] != tasks[1].parameters['drop_location']['id']
    assert tasks[2].task_type == TaskType.RETURN_BASE
    assert tasks[2].parameters['target_location'] == {'x': 1.0, 'y': 2.0, 'theta': 0.5}