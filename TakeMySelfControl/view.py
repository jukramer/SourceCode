from PySide6.QtWidgets import QLabel, QPushButton

import tab

class Tab(tab.Tab):
    def __init__(self, uber):
        super().__init__(uber, name="View")

    def init_ui_for_mission(self):
        super().init_ui_for_mission(mode="pygame")

        self.create_input_box("RPM at 100 PWM", "rpm_at_100_pwm")
        self.create_input_box("WASD Move PWM", "wasd_move_pwm")
        self.create_input_box("WASD Turn PWM", "wasd_turn_pwm")

        self.create_input_box("Wheel Radius [cm]", "wheel_radius")
        self.create_input_box("Wheel Base [cm]", "wheel_base")

        self.create_input_box("Reset Pos X [cm]", "reset_pos_x")
        self.create_input_box("Reset Pos Y [cm]", "reset_pos_y")
        self.create_input_box("Reset Pos Theta [deg]", "reset_pos_theta")

        def reset():
            mouse = self.uber.pg_renderer.mouse
            mouse.reset_position(
                self.uber.mission.reset_pos_x,
                self.uber.mission.reset_pos_y,
                self.uber.mission.reset_pos_theta
            )
            self.uber.pg_renderer.walls_seen_aabbs.clear()

        reset_button = QPushButton("Reset")
        reset_button.clicked.connect(reset)
        self.addRow(QLabel(">>> "), reset_button)

        self.separator()



def recalculate(tab: Tab):
    pass