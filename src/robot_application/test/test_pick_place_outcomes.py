import json

from robot_application.pick_place_outcome_processor import PickPlaceOutcomeProcessor
from robot_application.pick_place_payload_builder import PickPlacePayloadBuilder
from robot_application.task_adapters import PickPlaceTaskAdapter
from robot_application.task_definitions import Task, TaskType


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warn(self, message):
        self.messages.append(('warn', message))

    def error(self, message):
        self.messages.append(('error', message))


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeWorldState:
    def __init__(self):
        self.full_drop_ids = []
        self.empty_picks = []
        self.full_drops = []
        self.occupied_drops = []
        self.available_drops = set()

    def get_full_drop_ids(self):
        return list(self.full_drop_ids)

    def mark_pick_empty(self, pick_id):
        self.empty_picks.append(pick_id)

    def mark_drop_full(self, drop_id):
        self.full_drops.append(drop_id)

    def mark_drop_occupied(self, drop_id):
        self.occupied_drops.append(drop_id)

    def is_drop_available(self, drop_id):
        return drop_id in self.available_drops


def _task_pick_id(task):
    return str(task.parameters.get('pick_location', {}).get('id', ''))


def _task_drop_candidates(task):
    drop = task.parameters.get('drop_location', {})
    return [drop] if drop else []


def test_payload_builder_creates_initial_pick_place_payload():
    builder = PickPlacePayloadBuilder()
    payload = builder.build_assignment_payload(
        task_name='PickPlace pick_a',
        task_id='pick_place_pick_a',
        task_type='move_object',
        task_priority=7,
        task_parameters={
            'pick_location': {'id': 'pick_a'},
            'drop_location': {'id': 'drop_a'},
        },
        pick_id='pick_a',
        drop_candidates=[{'id': 'drop_a', 'priority': 1}],
        full_drop_ids=['drop_full'],
    )

    assert payload['source_pick_id'] == 'pick_a'
    assert payload['target_drop_id'] == 'drop_a'
    assert payload['drop_positions'] == [{'id': 'drop_a', 'priority': 1}]
    assert payload['full_drop_ids'] == ['drop_full']


def test_outcome_processor_marks_pick_empty_and_filters_related_tasks():
    processor = PickPlaceOutcomeProcessor()
    world_state = FakeWorldState()
    task_queue = [
        Task('task_a', TaskType.MOVE_OBJECT, 'A', parameters={'pick_location': {'id': 'pick_a'}}),
        Task('task_b', TaskType.MOVE_OBJECT, 'B', parameters={'pick_location': {'id': 'pick_b'}}),
        Task('task_c', TaskType.MOVE_OBJECT, 'C', parameters={'pick_location': {'id': 'pick_a'}}),
    ]
    task_context_by_id = {'task_a': {'task_id': 'task_a'}}

    result = processor.process_outcome(
        outcome={
            'task_id': 'task_a',
            'status': 'FAILED',
            'outcome_reason': 'PICK_EMPTY',
            'source_pick_id': 'pick_a',
        },
        world_state=world_state,
        task_queue=task_queue,
        task_context_by_id=task_context_by_id,
        task_pick_id_getter=_task_pick_id,
    )

    assert world_state.empty_picks == ['pick_a']
    assert [task.task_id for task in task_queue] == ['task_b']
    assert 'task_a' not in task_context_by_id
    assert result['replan_tasks'] is True


def test_outcome_processor_marks_drop_full_for_replan_required():
    processor = PickPlaceOutcomeProcessor()
    world_state = FakeWorldState()

    result = processor.process_outcome(
        outcome={
            'task_id': 'task_a',
            'status': 'REPLAN_REQUIRED',
            'outcome_reason': 'DROP_FULL',
            'target_drop_id': 'drop_a',
        },
        world_state=world_state,
        task_queue=[],
        task_context_by_id={},
        task_pick_id_getter=_task_pick_id,
    )

    assert world_state.full_drops == ['drop_a']
    assert result['needs_replan'] is True


def test_outcome_processor_marks_completed_pick_and_drop_state():
    processor = PickPlaceOutcomeProcessor()
    world_state = FakeWorldState()
    task_context_by_id = {'task_a': {'task_id': 'task_a'}}

    result = processor.process_outcome(
        outcome={
            'task_id': 'task_a',
            'status': 'COMPLETED',
            'outcome_reason': 'PLACED',
            'source_pick_id': 'pick_a',
            'target_drop_id': 'drop_a',
        },
        world_state=world_state,
        task_queue=[],
        task_context_by_id=task_context_by_id,
        task_pick_id_getter=_task_pick_id,
    )

    assert world_state.empty_picks == ['pick_a']
    assert world_state.occupied_drops == ['drop_a']
    assert task_context_by_id == {}
    assert result['replan_tasks'] is False


def test_pick_place_adapter_dispatches_assignment_and_waits_for_result():
    logger = FakeLogger()
    publisher = FakePublisher()
    world_state = FakeWorldState()
    world_state.full_drop_ids = ['drop_full']
    task_context_by_id = {}
    task_queue = []
    task = Task(
        'task_a',
        TaskType.MOVE_OBJECT,
        'Move A',
        time_estimate=18.0,
        base_priority=7,
        parameters={
            'pick_location': {'id': 'pick_a'},
            'drop_location': {'id': 'drop_a', 'priority': 1},
        },
    )

    adapter = PickPlaceTaskAdapter(
        logger=logger,
        mission_assignment_pub=publisher,
        world_state=world_state,
        task_context_by_id=task_context_by_id,
        task_queue=task_queue,
        task_pick_id_getter=_task_pick_id,
        task_drop_candidates_getter=_task_drop_candidates,
        start_mission_executor=lambda: True,
        wait_for_mission_executor_result=lambda timeout: timeout >= 43.0,
    )

    success = adapter.execute(task)

    assert success is True
    assert 'task_a' in task_context_by_id
    payload = json.loads(publisher.messages[0].data)
    assert payload['source_pick_id'] == 'pick_a'
    assert payload['target_drop_id'] == 'drop_a'
    assert payload['full_drop_ids'] == ['drop_full']


def test_pick_place_adapter_replans_to_next_available_drop():
    logger = FakeLogger()
    publisher = FakePublisher()
    world_state = FakeWorldState()
    world_state.available_drops = {'drop_b'}
    task_context_by_id = {
        'task_a': {
            'task_id': 'task_a',
            'drop_positions': [
                {'id': 'drop_a', 'priority': 1},
                {'id': 'drop_b', 'priority': 2},
            ],
            'excluded_drop_ids': [],
            'source_pick_id': 'pick_a',
            'active_arm_indices': [],
        }
    }

    adapter = PickPlaceTaskAdapter(
        logger=logger,
        mission_assignment_pub=publisher,
        world_state=world_state,
        task_context_by_id=task_context_by_id,
        task_queue=[],
        task_pick_id_getter=_task_pick_id,
        task_drop_candidates_getter=_task_drop_candidates,
        start_mission_executor=lambda: True,
        wait_for_mission_executor_result=lambda timeout: True,
    )

    handled = adapter.handle_replan_required(
        {
            'task_id': 'task_a',
            'task_type': 'move_object',
            'status': 'REPLAN_REQUIRED',
            'outcome_reason': 'DROP_FULL',
            'carry_object': True,
            'target_drop_id': 'drop_a',
            'source_pick_id': 'pick_a',
        }
    )

    assert handled is True
    payload = json.loads(publisher.messages[0].data)
    assert payload['target_drop_id'] == 'drop_b'
    assert payload['excluded_drop_ids'] == ['drop_a']