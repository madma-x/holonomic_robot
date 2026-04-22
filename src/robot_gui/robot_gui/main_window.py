"""Main window for the vertical match GUI."""

from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from robot_gui.ros_interface import RobotGuiRosInterface


@dataclass
class LocalState:
    """Local-only controls not yet wired to robot backend endpoints."""

    team_color: str = 'Blue'
    strategy: str = 'Balanced'
    match_ready: bool = False
    latch_armed: bool = False
    latch_status: str = 'Idle'


class MainWindow(QMainWindow):
    """Portrait-oriented match control GUI."""

    def __init__(self, ros: RobotGuiRosInterface):
        super().__init__()
        self.ros = ros
        self.local = LocalState()

        self._build_ui()
        self._apply_styles()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.refresh_from_ros)
        self.poll_timer.start(max(50, self.ros.poll_interval_ms))

    def _build_ui(self):
        self.setWindowTitle('Holonomic Match Control')
        self.setMinimumSize(720, 1080)

        root = QWidget(self)
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        self.header_time = QLabel('00:00')
        self.header_time.setObjectName('TimeValue')
        self.header_phase = QLabel('PHASE: SETUP')
        self.header_score = QLabel('SCORE: 0')

        header_box = QGroupBox('Match')
        header_layout = QHBoxLayout(header_box)
        header_layout.addWidget(self.header_time, 2)
        header_layout.addWidget(self.header_phase, 1)
        header_layout.addWidget(self.header_score, 1)
        layout.addWidget(header_box)

        self.safety_value = QLabel('UNKNOWN')
        self.safety_value.setObjectName('SafetyValue')
        safety_box = QGroupBox('Safety State (hardware authoritative)')
        safety_layout = QVBoxLayout(safety_box)
        safety_layout.addWidget(self.safety_value)
        layout.addWidget(safety_box)

        setup_box = QGroupBox('Pre-Match Setup (local app state)')
        setup_layout = QGridLayout(setup_box)

        self.team_combo = QComboBox()
        self.team_combo.addItems(['Blue', 'Yellow'])
        self.team_combo.currentTextChanged.connect(self._set_team)

        self.strategy_combo = QComboBox()
        self.strategy_combo.addItems(['Aggressive', 'Balanced', 'Defensive'])
        self.strategy_combo.currentTextChanged.connect(self._set_strategy)

        self.ready_button = QPushButton('Match Ready: OFF')
        self.ready_button.clicked.connect(self._toggle_ready)

        self.reset_ui_button = QPushButton('Reset UI State')
        self.reset_ui_button.clicked.connect(self._reset_local_state)

        self.latch_state_label = QLabel('Latch: Idle')

        setup_layout.addWidget(QLabel('Team'), 0, 0)
        setup_layout.addWidget(self.team_combo, 0, 1)
        setup_layout.addWidget(QLabel('Strategy'), 1, 0)
        setup_layout.addWidget(self.strategy_combo, 1, 1)
        setup_layout.addWidget(self.ready_button, 2, 0, 1, 2)
        setup_layout.addWidget(self.reset_ui_button, 3, 0, 1, 2)
        setup_layout.addWidget(self.latch_state_label, 4, 0, 1, 2)
        layout.addWidget(setup_box)

        controls_box = QGroupBox('Match Controls')
        controls_layout = QGridLayout(controls_box)

        self.start_btn = QPushButton('Start Match')
        self.stop_btn = QPushButton('Stop Match')
        self.plan_start_btn = QPushButton('Planner Start')
        self.plan_stop_btn = QPushButton('Planner Stop')
        self.plan_replan_btn = QPushButton('Planner Replan')

        self.start_btn.clicked.connect(lambda: self.ros.call_named_service('start_match'))
        self.stop_btn.clicked.connect(self._confirm_stop_match)
        self.plan_start_btn.clicked.connect(lambda: self.ros.call_named_service('planner_start'))
        self.plan_stop_btn.clicked.connect(lambda: self.ros.call_named_service('planner_stop'))
        self.plan_replan_btn.clicked.connect(lambda: self.ros.call_named_service('planner_replan'))

        controls_layout.addWidget(self.start_btn, 0, 0)
        controls_layout.addWidget(self.stop_btn, 0, 1)
        controls_layout.addWidget(self.plan_start_btn, 1, 0)
        controls_layout.addWidget(self.plan_stop_btn, 1, 1)
        controls_layout.addWidget(self.plan_replan_btn, 2, 0, 1, 2)
        layout.addWidget(controls_box)

        diag_box = QGroupBox('Diagnostics')
        diag_layout = QGridLayout(diag_box)

        self.match_active_value = QLabel('False')
        self.current_task_value = QLabel('-')
        self.queue_size_value = QLabel('0')
        self.mission_status_value = QLabel('UNKNOWN')
        self.servo_value = QLabel('No data')
        self.pump_value = QLabel('No data')

        diag_layout.addWidget(QLabel('Match Active'), 0, 0)
        diag_layout.addWidget(self.match_active_value, 0, 1)
        diag_layout.addWidget(QLabel('Current Task'), 1, 0)
        diag_layout.addWidget(self.current_task_value, 1, 1)
        diag_layout.addWidget(QLabel('Queue Size'), 2, 0)
        diag_layout.addWidget(self.queue_size_value, 2, 1)
        diag_layout.addWidget(QLabel('Mission Status'), 3, 0)
        diag_layout.addWidget(self.mission_status_value, 3, 1)
        diag_layout.addWidget(QLabel('Servo State'), 4, 0)
        diag_layout.addWidget(self.servo_value, 4, 1)
        diag_layout.addWidget(QLabel('Pump State'), 5, 0)
        diag_layout.addWidget(self.pump_value, 5, 1)
        layout.addWidget(diag_box)

        footer_frame = QFrame()
        footer_layout = QHBoxLayout(footer_frame)
        footer_layout.setContentsMargins(8, 8, 8, 8)
        self.feedback_label = QLabel('Ready')
        self.feedback_label.setWordWrap(True)
        self.local_notice_label = QLabel('Local setup state is not applied to robot backend yet.')
        self.local_notice_label.setWordWrap(True)
        footer_layout.addWidget(self.feedback_label, 1)
        footer_layout.addWidget(self.local_notice_label, 1)
        layout.addWidget(footer_frame)

    def _apply_styles(self):
        self.setStyleSheet(
            """
            QWidget {
                background-color: #f5f4ef;
                color: #132335;
                font-size: 20px;
            }
            QGroupBox {
                border: 2px solid #d3c9b7;
                border-radius: 10px;
                margin-top: 12px;
                padding-top: 16px;
                font-weight: 600;
                background-color: #fffdf8;
            }
            QLabel {
                font-size: 21px;
            }
            QLabel#TimeValue {
                font-size: 42px;
                font-weight: 700;
            }
            QLabel#SafetyValue {
                font-size: 34px;
                font-weight: 700;
            }
            QPushButton {
                min-height: 58px;
                border-radius: 10px;
                border: 2px solid #af9b7a;
                background-color: #f0e3cd;
                font-size: 22px;
                font-weight: 600;
            }
            QPushButton:pressed {
                background-color: #e4cfac;
            }
            QComboBox {
                min-height: 48px;
                border: 2px solid #af9b7a;
                border-radius: 8px;
                padding-left: 8px;
                background-color: #fffaf0;
            }
            """
        )
        self.setFont(QFont('DejaVu Sans', 14))

    def _set_team(self, value: str):
        self.local.team_color = value
        self.local_notice_label.setText(f'Local state: team color set to {value}.')

    def _set_strategy(self, value: str):
        self.local.strategy = value
        self.local_notice_label.setText(f'Local state: strategy set to {value}.')

    def _toggle_ready(self):
        self.local.match_ready = not self.local.match_ready
        if self.local.match_ready:
            self.local.latch_armed = True
            self.local.latch_status = 'Armed (waiting latch removal)'
            self.ready_button.setText('Match Ready: ON')
            self.local_notice_label.setText('Ready armed: latch removal will start match in future GPIO mode.')
        else:
            self.local.latch_armed = False
            self.local.latch_status = 'Idle'
            self.ready_button.setText('Match Ready: OFF')
            self.local_notice_label.setText('Ready disarmed: latch removal ignored.')
        self._refresh_local_labels()

    def on_latch_removed_event(self):
        """Future GPIO hook: only start match when ready latch is armed."""
        if not self.local.match_ready:
            self.local.latch_status = 'Latch removed while not ready (ignored)'
            self.local_notice_label.setText('Latch removal ignored because Match Ready is OFF.')
            self._refresh_local_labels()
            return

        self.local.latch_status = 'Latch removed -> starting match'
        self.local_notice_label.setText('Latch removed while armed: sending match start.')
        self._refresh_local_labels()
        self.ros.call_named_service('start_match')

    def _reset_local_state(self):
        self.local = LocalState()
        self.team_combo.setCurrentText(self.local.team_color)
        self.strategy_combo.setCurrentText(self.local.strategy)
        self.ready_button.setText('Match Ready: OFF')
        self.local_notice_label.setText('Local UI state reset.')
        self._refresh_local_labels()

    def _confirm_stop_match(self):
        answer = QMessageBox.question(
            self,
            'Confirm Stop',
            'Stop the current match?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.ros.call_named_service('stop_match')

    def _refresh_local_labels(self):
        self.latch_state_label.setText(f'Latch: {self.local.latch_status}')

    def refresh_from_ros(self):
        snap = self.ros.snapshot()

        remaining = max(0.0, snap.time_remaining)
        mins = int(remaining) // 60
        secs = int(remaining) % 60
        self.header_time.setText(f'{mins:02d}:{secs:02d}')

        self.header_phase.setText(f'PHASE: {snap.phase}')
        self.header_score.setText(f'SCORE: {snap.score}')

        self.safety_value.setText(snap.safety_state)
        if 'SAFE_ON' in snap.safety_state:
            self.safety_value.setStyleSheet('color: #1f7a2a; font-weight: 700;')
        elif 'SAFE_OFF' in snap.safety_state:
            self.safety_value.setStyleSheet('color: #a10f0f; font-weight: 700;')
        else:
            self.safety_value.setStyleSheet('color: #8a6a00; font-weight: 700;')

        self.match_active_value.setText(str(snap.match_active))
        self.current_task_value.setText(snap.current_task)
        self.queue_size_value.setText(str(snap.queue_size))
        self.mission_status_value.setText(snap.mission_status)
        self.servo_value.setText(snap.servo_summary)
        self.pump_value.setText(snap.pump_summary)
        self.feedback_label.setText(snap.service_feedback)
