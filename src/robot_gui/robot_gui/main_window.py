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
    QTabWidget,
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

    def __init__(self, ros: RobotGuiRosInterface, target_width: int = 480, target_height: int = 800):
        super().__init__()
        self.ros = ros
        self.local = LocalState()
        self.target_width = max(320, target_width)
        self.target_height = max(480, target_height)
        self.compact_mode = self.target_width <= 480 or self.target_height <= 800

        self._build_ui()
        self._apply_styles()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.refresh_from_ros)
        self.poll_timer.start(max(50, self.ros.poll_interval_ms))

    def _build_ui(self):
        self.setWindowTitle('Holonomic Match Control')

        root = QWidget(self)
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)

        self.pages = QTabWidget(self)
        self.pages.setTabPosition(QTabWidget.North)
        root_layout.addWidget(self.pages)

        controls_page = QWidget(self)
        diagnostics_page = QWidget(self)
        self.pages.addTab(controls_page, 'Controls')
        self.pages.addTab(diagnostics_page, 'Diagnostics')

        margin = 10 if self.compact_mode else 18
        spacing = 8 if self.compact_mode else 12
        controls_page_layout = QVBoxLayout(controls_page)
        controls_page_layout.setContentsMargins(margin, margin, margin, margin)
        controls_page_layout.setSpacing(spacing)

        diagnostics_layout = QVBoxLayout(diagnostics_page)
        diagnostics_layout.setContentsMargins(margin, margin, margin, margin)
        diagnostics_layout.setSpacing(spacing)

        self.header_time = QLabel('00:00')
        self.header_time.setObjectName('TimeValue')
        self.header_phase = QLabel('PHASE: SETUP')
        self.header_score = QLabel('SCORE: 0')

        header_box = QGroupBox('Match')
        header_layout = QHBoxLayout(header_box)
        header_layout.setContentsMargins(10, 14, 10, 10)
        header_layout.setSpacing(8 if self.compact_mode else 12)
        header_layout.addWidget(self.header_time, 2)
        header_layout.addWidget(self.header_phase, 1)
        header_layout.addWidget(self.header_score, 1)
        controls_page_layout.addWidget(header_box)

        self.safety_value = QLabel('UNKNOWN')
        self.safety_value.setObjectName('SafetyValue')
        safety_box = QGroupBox('Safety State (hardware authoritative)')
        safety_layout = QVBoxLayout(safety_box)
        safety_layout.setContentsMargins(10, 14, 10, 10)
        safety_layout.addWidget(self.safety_value)
        controls_page_layout.addWidget(safety_box)

        setup_box = QGroupBox('Pre-Match Setup (local app state)')
        setup_layout = QGridLayout(setup_box)
        setup_layout.setHorizontalSpacing(8 if self.compact_mode else 12)
        setup_layout.setVerticalSpacing(6 if self.compact_mode else 10)

        self.team_blue_button = QPushButton('Blue')
        self.team_blue_button.setCheckable(True)
        self.team_blue_button.clicked.connect(lambda: self._set_team('Blue'))

        self.team_yellow_button = QPushButton('Yellow')
        self.team_yellow_button.setCheckable(True)
        self.team_yellow_button.clicked.connect(lambda: self._set_team('Yellow'))

        self.strategy_combo = QComboBox()
        self.strategy_combo.addItems(['Aggressive', 'Balanced', 'Defensive'])
        self.strategy_combo.currentTextChanged.connect(self._set_strategy)

        self.ready_button = QPushButton('Match Ready: OFF')
        self.ready_button.clicked.connect(self._toggle_ready)

        self.reset_ui_button = QPushButton('Reset UI State')
        self.reset_ui_button.clicked.connect(self._reset_local_state)

        self.reset_pose_button = QPushButton('Reset Position')
        self.reset_pose_button.clicked.connect(self.ros.reset_to_initial_position)

        self.latch_state_label = QLabel('Latch: Idle')

        setup_layout.addWidget(QLabel('Team'), 0, 0)
        team_button_row = QWidget(self)
        team_button_layout = QHBoxLayout(team_button_row)
        team_button_layout.setContentsMargins(0, 0, 0, 0)
        team_button_layout.setSpacing(8 if self.compact_mode else 10)
        team_button_layout.addWidget(self.team_blue_button)
        team_button_layout.addWidget(self.team_yellow_button)
        setup_layout.addWidget(team_button_row, 0, 1)
        setup_layout.addWidget(QLabel('Strategy'), 1, 0)
        setup_layout.addWidget(self.strategy_combo, 1, 1)
        setup_layout.addWidget(self.ready_button, 2, 0, 1, 2)
        setup_layout.addWidget(self.reset_ui_button, 3, 0, 1, 2)
        setup_layout.addWidget(self.reset_pose_button, 4, 0, 1, 2)
        setup_layout.addWidget(self.latch_state_label, 5, 0, 1, 2)
        controls_page_layout.addWidget(setup_box)

        self._apply_team_selection_state()

        controls_box = QGroupBox('Match Controls')
        match_controls_layout = QGridLayout(controls_box)
        match_controls_layout.setHorizontalSpacing(8 if self.compact_mode else 12)
        match_controls_layout.setVerticalSpacing(6 if self.compact_mode else 10)

        self.start_btn = QPushButton('Start Match')
        self.stop_btn = QPushButton('Stop Match')

        self.start_btn.clicked.connect(lambda: self.ros.call_named_service('start_match'))
        self.stop_btn.clicked.connect(self._confirm_stop_match)

        match_controls_layout.addWidget(self.start_btn, 0, 0)
        match_controls_layout.addWidget(self.stop_btn, 0, 1)
        controls_page_layout.addWidget(controls_box)

        diag_box = QGroupBox('Diagnostics')
        diag_layout = QGridLayout(diag_box)
        diag_layout.setHorizontalSpacing(8 if self.compact_mode else 12)
        diag_layout.setVerticalSpacing(6 if self.compact_mode else 10)

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
        diagnostics_layout.addWidget(diag_box)

        footer_frame = QFrame()
        footer_layout = QHBoxLayout(footer_frame)
        footer_layout.setContentsMargins(8, 8, 8, 8)
        footer_layout.setSpacing(8)
        self.feedback_label = QLabel('Ready')
        self.feedback_label.setWordWrap(True)
        footer_layout.addWidget(self.feedback_label, 1)
        controls_page_layout.addWidget(footer_frame)

        diagnostics_footer = QFrame()
        diagnostics_footer_layout = QVBoxLayout(diagnostics_footer)
        diagnostics_footer_layout.setContentsMargins(8, 8, 8, 8)
        diagnostics_footer_layout.setSpacing(6)
        self.diagnostics_feedback_label = QLabel('Ready')
        self.diagnostics_feedback_label.setWordWrap(True)
        diagnostics_footer_layout.addWidget(self.diagnostics_feedback_label)
        diagnostics_layout.addWidget(diagnostics_footer)

    def _apply_styles(self):
        base_font_size = 16 if self.compact_mode else 20
        label_font_size = 17 if self.compact_mode else 21
        time_font_size = 32 if self.compact_mode else 42
        safety_font_size = 20 if self.compact_mode else 28
        button_min_height = 46 if self.compact_mode else 58
        combo_min_height = 40 if self.compact_mode else 48
        border_radius = 8 if self.compact_mode else 10

        self.setStyleSheet(
            f"""
            QWidget {{
                background-color: #f5f4ef;
                color: #132335;
                font-size: {base_font_size}px;
            }}
            QGroupBox {{
                border: 2px solid #d3c9b7;
                border-radius: {border_radius}px;
                margin-top: 10px;
                padding-top: 14px;
                font-weight: 600;
                background-color: #fffdf8;
            }}
            QLabel {{
                font-size: {label_font_size}px;
            }}
            QLabel#TimeValue {{
                font-size: {time_font_size}px;
                font-weight: 700;
            }}
            QLabel#SafetyValue {{
                font-size: {safety_font_size}px;
                font-weight: 700;
            }}
            QPushButton {{
                min-height: {button_min_height}px;
                border-radius: {border_radius}px;
                border: 2px solid #af9b7a;
                background-color: #f0e3cd;
                font-size: {label_font_size}px;
                font-weight: 600;
            }}
            QPushButton:pressed {{
                background-color: #e4cfac;
            }}
            QPushButton:checked {{
                background-color: #d6e9ff;
                border-color: #6b8fb6;
            }}
            QComboBox {{
                min-height: {combo_min_height}px;
                border: 2px solid #af9b7a;
                border-radius: {border_radius}px;
                padding-left: 8px;
                background-color: #fffaf0;
            }}
            QTabWidget::pane {{
                border: 0;
                top: -1px;
            }}
            QTabBar::tab {{
                background: #efe4d0;
                border: 2px solid #d3c9b7;
                border-bottom: 0;
                padding: 8px 14px;
                margin-right: 4px;
                min-width: 120px;
                font-weight: 600;
            }}
            QTabBar::tab:selected {{
                background: #fffdf8;
                color: #132335;
            }}
            """
        )
        self.setFont(QFont('DejaVu Sans', 11 if self.compact_mode else 14))

    def _set_team(self, value: str):
        self.local.team_color = value
        self._apply_team_selection_state()
        self.feedback_label.setText(f'Team set to {value}.')

    def _apply_team_selection_state(self):
        is_blue = self.local.team_color == 'Blue'
        self.team_blue_button.setChecked(is_blue)
        self.team_yellow_button.setChecked(not is_blue)

    def _set_strategy(self, value: str):
        self.local.strategy = value
        self.feedback_label.setText(f'Strategy set to {value}.')

    def _toggle_ready(self):
        self.local.match_ready = not self.local.match_ready
        if self.local.match_ready:
            self.local.latch_armed = True
            self.local.latch_status = 'Armed (waiting latch removal)'
            self.ready_button.setText('Match Ready: ON')
            self.feedback_label.setText('Ready armed: latch removal will start match in future GPIO mode.')
        else:
            self.local.latch_armed = False
            self.local.latch_status = 'Idle'
            self.ready_button.setText('Match Ready: OFF')
            self.feedback_label.setText('Ready disarmed: latch removal ignored.')
        self._refresh_local_labels()

    def on_latch_removed_event(self):
        """Future GPIO hook: only start match when ready latch is armed."""
        if not self.local.match_ready:
            self.local.latch_status = 'Latch removed while not ready (ignored)'
            self.feedback_label.setText('Latch removal ignored because Match Ready is OFF.')
            self._refresh_local_labels()
            return

        self.local.latch_status = 'Latch removed -> starting match'
        self.feedback_label.setText('Latch removed while armed: sending match start.')
        self._refresh_local_labels()
        self.ros.call_named_service('start_match')

    def _reset_local_state(self):
        self.local = LocalState()
        self._apply_team_selection_state()
        self.strategy_combo.setCurrentText(self.local.strategy)
        self.ready_button.setText('Match Ready: OFF')
        self.feedback_label.setText('Local UI state reset.')
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
        self.diagnostics_feedback_label.setText(snap.service_feedback)
