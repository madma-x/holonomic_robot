import threading
import time

from robot_application.task_planner import TaskPlanner


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


def test_wait_for_mission_executor_result_ignores_stale_previous_completion():
    planner = TaskPlanner.__new__(TaskPlanner)
    planner.stop_requested = False
    planner.mission_executor_status = 'STARTING'
    planner.last_mission_outcome = {
        'task_id': 'task_2',
        'status': 'COMPLETED',
        'outcome_seq': 5,
    }
    planner.get_logger = lambda: FakeLogger()
    planner._adapter_for_outcome = lambda outcome: None

    def publish_new_task_outcome():
        time.sleep(0.05)
        planner.mission_executor_status = 'RUNNING'
        time.sleep(0.08)
        planner.last_mission_outcome = {
            'task_id': 'task_3',
            'status': 'COMPLETED',
            'outcome_seq': 6,
        }
        planner.mission_executor_status = 'COMPLETED'

    worker = threading.Thread(target=publish_new_task_outcome)
    worker.start()

    started_at = time.time()
    try:
        assert planner._wait_for_mission_executor_result('task_3', 0.5) is True
    finally:
        worker.join()

    assert time.time() - started_at >= 0.1