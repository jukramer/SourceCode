#!/usr/bin/env python

import sys
import json
import os
import importlib 
import threading
import queue
import time
import signal
import subprocess
import platform

import numpy as np

os.environ["QT_API"] = "PyQt6"
os.environ["SDL_VIDEODRIVER"] = "dummy"

from PySide6.QtWidgets import (QApplication, QMainWindow, QPushButton, QFileDialog, QTabWidget, QMenuBar, QMenu,
                             QVBoxLayout, QHBoxLayout, QWidget, QPlainTextEdit, QMessageBox, QLabel)
from PySide6.QtGui import (QFont, QAction)
from PySide6.QtCore import QTimer, QObject, Signal

from mission import Mission
from renderer import PgRenderer, pg_surface_to_qimage

import matplotlib.pyplot as plt
plt.style.use('dark_background')

LOAD_DEFAULT = True
DEFAULT_FILE = "default_mission.json"

DONT_PROMPT_UNSAVED = False

tabs = ['view', 'config']
for m in tabs:
    globals()[m] = __import__(m)

if platform.system() == "Darwin":
    example_mission = json.load(open(os.path.join(os.path.dirname(__file__), 'example_mission_mac.json')))
else:
    example_mission = json.load(open(os.path.join(os.path.dirname(__file__), 'example_mission.json')))


class MissionValueError(BaseException):
    def __init__(self, msg):
        self.msg = msg

class TextEmitter(QObject):
    append_text = Signal(str)
    
class TextBoxWriter:
    def __init__(self, text_box):
        self.text_box = text_box

    def write(self, text):
        self.text_box.insertPlainText(text)
        self.text_box.verticalScrollBar().setValue(self.text_box.verticalScrollBar().maximum())

    def flush(self):
        pass


rpm_response_queue = queue.Queue()

class TakeMySelfControl(QMainWindow):
    def __init__(self):
        super().__init__()

        self.pg_renderer = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(16) 

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start(2)

        self.last_update_time = None

        self.tabs = [globals()[m].Tab(self) for m in tabs]

        self.mission = None
        self.unsaved_changes = False

        self.run_thread = None
        self.running = False
        self.process = None
        self.should_stop = False

        self.setMinimumSize(1500, 700) 

        self.show()
        self.setWindowTitle('TakeMySelfControl')
        self.setGeometry(300, 300, 800, 600)

        menuBar = QMenuBar(self)
        self.setMenuBar(menuBar)

        if platform.system() == "Darwin":
            menuBar.setNativeMenuBar(False)

        missionMenu = QMenu("&File", self)
        menuBar.addMenu(missionMenu)
        
        action = QAction(self)
        action.setText("&New")
        action.triggered.connect(self.new_mission)
        missionMenu.addAction(action)

        action = QAction(self)
        action.setText("&Load Mission")
        action.triggered.connect(self.load_mission)
        missionMenu.addAction(action)

        action = QAction(self)
        action.setText("&Save Mission")
        action.triggered.connect(self.save_mission)
        missionMenu.addAction(action)

        if LOAD_DEFAULT:
            try:
                with open(os.path.join(os.path.dirname(__file__), DEFAULT_FILE)) as f:
                    data = json.load(f)
            except FileNotFoundError:
                data = example_mission

            self.mission = Mission(data)
            self.init_ui_for_mission()

    def update(self):
        if not self.pg_renderer or not self.mission:
            return
        
        dt = self.pg_renderer.clock.tick() / 1000.0  # Convert milliseconds to seconds
        self.pg_renderer.update(dt)

    def update_frame(self):
        if not self.pg_renderer or not self.mission:
            return
        
        surface = self.pg_renderer.render()
        self.pg_image = pg_surface_to_qimage(surface)

        for t in self.tabs:
            try:
                t.update_frame()
            except Exception as e:
                print(e)

    def mouse_pressed_in_game(self, x, y, drag_id=None):
        if not self.pg_renderer or not self.mission:
            return
        # self.pg_renderer.mouse_pressed_in_game(x, y, drag_id)
        
    def keyPressEvent(self, event):
        if not self.pg_renderer or not self.mission:
            return
        self.pg_renderer.keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not self.pg_renderer or not self.mission:
            return
        self.pg_renderer.keyReleaseEvent(event)

    def init_ui_for_mission(self):
        self.pg_renderer = PgRenderer(self)

        self.setCentralWidget(None)
        self.tabs = [globals()[m].Tab(self) for m in tabs]

        main_layout = QHBoxLayout()

        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)

        output_and_buttons = QVBoxLayout()

        self.console_text_box = QPlainTextEdit(self)
        
        if platform.system() == "Darwin":
            font = QFont("Monaco", 10)
        else:
            font = QFont("Courier", 10)
        font.setStyleHint(QFont.Monospace)

        self.console_text_box.setFont(font)  
        self.console_text_box.setReadOnly(True)
        self.console_text_box.setLineWrapMode(QPlainTextEdit.WidgetWidth) 
        self.console_text_box.setFixedWidth(400)

        self.text_box = TextBoxWriter(self.console_text_box)
        self.text_emitter = TextEmitter()
        self.text_emitter.append_text.connect(lambda text: self.text_box.write(text))

        output_and_buttons.addWidget(self.console_text_box)

        build_and_run = QVBoxLayout()

        build_and_build_status = QHBoxLayout()

        build_button = QPushButton('Build', self)
        build_and_build_status.addWidget(build_button)

        build_status = QLabel(self)
        build_and_build_status.addWidget(build_status)

        build_and_run.addLayout(build_and_build_status)

        run_button = QPushButton('Run', self)
        build_and_run.addWidget(run_button)

        stop_button = QPushButton('Stop', self)
        stop_button.setEnabled(False)
        build_and_run.addWidget(stop_button)

        output_and_buttons.addLayout(build_and_run)

        def on_run_complete():
            self.running = False
            build_button.setEnabled(True)
            run_button.setEnabled(True)
            stop_button.setEnabled(False)
            build_status.setText("Run complete.")
            build_status.setStyleSheet("color: black;")

        def run():
            if not self.mission:
                return
            if not self.mission:
                return
            
            if self.running:
                print("Already running.")
                return
            
            self.recalculate()

            run_cmd = self.mission.run_command
            dir = self.mission.directory
            run_cmd = os.path.join(dir, run_cmd)
            self.text_box.write("Running command: " + run_cmd + "\n\n")
            if not os.path.exists(run_cmd):
                self.text_box.write("Path not found.\n")
                return
            
            self.running = True
            build_button.setEnabled(False)
            run_button.setEnabled(False)
            stop_button.setEnabled(True)

            build_status.setText("Running...")
            build_status.setStyleSheet("color: green;")

            self.should_stop = False

            output_buffer = []
            # Adjust these values based on desired responsiveness vs. performance
            
            MAX_BUFFER_LINES = 1  # Max lines to buffer before an update
            MAX_BUFFER_TIME_S = 0.1 # Max time (e.g., 100ms) before forcing an update
            
            last_emit_time = time.monotonic()

            def write(text):
                if self.process:
                    self.process.stdin.write(text)
                    self.process.stdin.flush()

            def handle_line(line):
                nonlocal last_emit_time

                # --- Special handling for ">>>" commands ---
                if line.startswith(">>>"):
                    # print("Handling command:", line.strip())
                    parts = line.split()
                    if len(parts) < 2:
                        output_buffer.append(f">>> Error parsing command: {line.strip()}\n") 
                        return
                    command = parts[1]
                    if command == "setPWM":
                        if len(parts) < 4:
                            output_buffer.append(f">>> Error parsing setPWM command: {line.strip()}\n")
                            return

                        motor = parts[2].lower()
                        if motor != "left" and motor != "right":
                            output_buffer.append(f">>> Unknown motor: {motor}\n")
                            return

                        pwm = parts[3]
                        try:
                            pwm = float(pwm)
                            pwm = max(min(pwm, 100.0), -100.0)
                        except ValueError:
                            output_buffer.append(f">>> Invalid PWM value: {pwm}\n")
                            return

                        if motor == "left":
                            self.pg_renderer.mouse.left_pwm = pwm
                        else:
                            self.pg_renderer.mouse.right_pwm = pwm
                        return
                    elif command == "readRPM":
                        if len(parts) < 3:
                            output_buffer.append(f">>> Error parsing readRPM command: {line.strip()}\n")
                            return
                        
                        motor = parts[2].lower()
                        if motor != "left" and motor != "right":
                            output_buffer.append(f">>> Unknown motor: {motor}\n")
                            return

                        rpm_value = self.pg_renderer.mouse.left_rpm if motor == "left" else self.pg_renderer.mouse.right_rpm

                        write(f"{rpm_value}\n")
                        return
                    elif command == "readPOS":
                        if len(parts) < 3:
                            print(f">>> Error parsing readPOS command: {line.strip()}\n")
                            return
                        
                        motor = parts[2].lower()
                        if motor != "left" and motor != "right":
                            print(f">>> Unknown motor: {motor}\n")
                            return

                        pos_value = (self.pg_renderer.mouse.left_pos if motor == "left" else self.pg_renderer.mouse.right_pos) * 10

                        write(f"{pos_value}\n")
                        return
                    elif command == "readTOF":
                        if len(parts) < 3:
                            output_buffer.append(f">>> Error parsing readTOF command: {line.strip()}\n")
                            return
                        
                        try:
                            sensor = int(parts[2])
                            if sensor < 0 or sensor >= 5:
                                raise ValueError("Sensor index out of range")
                        except ValueError:
                            output_buffer.append(f">>> Invalid sensor index: {parts[2]}\n")
                            return

                        tof_reading, tof_valid = self.pg_renderer.mouse.get_tof_reading(sensor)
                        write(f"{int(tof_reading)} {'t' if tof_valid else 'f'}\n")
                        return
                    elif command == "setWall":
                        if len(parts) < 5:
                            output_buffer.append(f">>> Error parsing setWall command: {line.strip()}\n")
                            return
                        
                        try:
                            x = int(parts[2])
                            y = int(parts[3])
                            direction = int(parts[4])
                            if direction not in [0, 1, 2, 3]:
                                raise ValueError("Invalid direction")
                        except ValueError as e:
                            output_buffer.append(f">>> Invalid wall data: {e}\n")
                            return
                        print(f"Setting wall at ({x}, {y}) in direction {direction}")
                        self.pg_renderer.set_wall_seen(x, 15 - y, direction)
                        return
                    elif command == "readIMU":
                        AX = 0.0
                        AY = 0.0
                        GYRO_Z = self.pg_renderer.mouse.ang_vel * 180.0 / np.pi  # Convert radians to degrees

                        write(f"{AX} {AY} {GYRO_Z}\n")
                    elif command == "vizPARTICLES":
                        particles = []
                        # read triplet of points for each particle, floats are in "parts"
                        for i in range(2, len(parts), 3):
                            if i + 2 >= len(parts):
                                break
                            try:
                                x = float(parts[i]) / 10 # Convert to cm
                                y = float(parts[i + 1]) / 10 # Convert to cm
                                rot = float(parts[i + 2])
                                particles.append((x, y, rot))
                            except ValueError:
                                output_buffer.append(f">>> Invalid particle data: {parts[i:i+3]}\n")
                                break
                        self.pg_renderer.set_particles(particles)
                    elif command == "vizPOSE":
                        if len(parts) < 5:
                            output_buffer.append(f">>> Error parsing vizPOSE command: {line.strip()}\n")
                            return
                        
                        try:
                            x = float(parts[2]) / 10  # Convert to cm
                            y = float(parts[3]) / 10  # Convert to cm
                            theta = float(parts[4])
                        except ValueError:
                            output_buffer.append(f">>> Invalid pose data: {parts[2:5]}\n")
                            return
                        
                        self.pg_renderer.set_viz_pose(x, y, theta)
                    elif command == "vizTARGET":
                        if len(parts) < 5:
                            output_buffer.append(f">>> Error parsing vizTARGET command: {line.strip()}\n")
                            return
                        
                        try:
                            x = float(parts[2]) / 10  # Convert to cm
                            y = float(parts[3]) / 10  # Convert to cm
                            theta = float(parts[4])
                        except ValueError:
                            output_buffer.append(f">>> Invalid target data: {parts[2:5]}\n")
                            return
                        
                        self.pg_renderer.set_viz_target(x, y, theta)
                    else:
                        output_buffer.append(f"Unknown command: {line.strip()}\n")
                        return                             
                else:
                    # --- Regular line: add to buffer ---
                    output_buffer.append(line)
                    print(line, end="")

                current_time = time.monotonic()
                if (len(output_buffer) >= MAX_BUFFER_LINES or (current_time - last_emit_time) >= MAX_BUFFER_TIME_S):
                    if output_buffer:
                        self.text_emitter.append_text.emit("".join(output_buffer))
                        output_buffer.clear()
                        last_emit_time = current_time

            def worker():
                try:
                    if platform.system() == "Darwin":
                        self.process = subprocess.Popen(
                            run_cmd,
                            cwd=dir,
                            stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            preexec_fn=os.setsid,
                            text=True
                        )
                    else:
                        self.process = subprocess.Popen(
                            run_cmd,
                            cwd=dir,
                            stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
                            text=True
                        )
                    self.should_stop = False

                    print("Started process:", self.process)

                    def read_stream():
                        while True:
                            line = self.process.stdout.readline()
                            if not line:
                                break
                            if self.should_stop:
                                break
                            handle_line(line)   

                        # --- Emit any remaining lines after the loop ---
                        if output_buffer:
                            self.text_emitter.append_text.emit("".join(output_buffer))
                            output_buffer.clear()
        
                    read_thread = threading.Thread(target=read_stream)
                    read_thread.start()
                    
                    self.process.wait()
                    read_thread.join()
                    self.process = None
                except Exception as e:
                    print(f"Exception in run(): {e}")
                    self.text_box.write(f"Exception in run(): {e}\n")
                    
                on_run_complete()
            threading.Thread(target=worker, daemon=True).start()

        run_button.clicked.connect(run)
        
        def stop():
            if self.process:
                try:
                    self.should_stop = True
                    if platform.system() == "Darwin":
                        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    else:
                        self.process.send_signal(signal.CTRL_BREAK_EVENT)
                    self.process.wait()
                except Exception as e:
                    self.text_box.write(f"Error stopping process: {e}\n")
                on_run_complete()

        stop_button.clicked.connect(stop)

        def build():
            self.recalculate()

            run_button.setEnabled(False)
            build_button.setEnabled(False)

            build_cmd = self.mission.build_command
            dir = self.mission.directory
            self.text_box.write("Running command: " + build_cmd + "\n\n")

            build_status.setText("Building...")
            build_status.setStyleSheet("color: orange;")

            def on_complete(success):
                if success:
                    build_status.setText("Complete.")
                    build_status.setStyleSheet("color: green;")
                else:
                    build_status.setText("Failed.")
                    build_status.setStyleSheet("color: red;")
                run_button.setEnabled(True)
                build_button.setEnabled(True)

            def worker():
                try:
                    process = subprocess.Popen(
                        build_cmd,
                        shell=True,
                        cwd=dir,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True
                    )

                    # Read both streams concurrently using threads
                    def read_stream(stream, label):
                        for line in iter(stream.readline, ''):
                            self.text_emitter.append_text.emit(f"{line.strip()}\n")

                    stdout_thread = threading.Thread(target=read_stream, args=(process.stdout, "stdout"))
                    stderr_thread = threading.Thread(target=read_stream, args=(process.stderr, "stderr"))

                    stdout_thread.start()
                    stderr_thread.start()
                    stdout_thread.join()
                    stderr_thread.join()

                    process.stdout.close()
                    process.stderr.close()
                    process.wait()

                    success = process.returncode == 0
                except Exception as e:
                    success = False
                    self.text_box.write(f"Exception running command: {e}\n")

                on_complete(success)

            threading.Thread(target=worker, daemon=True).start()
         

        build_button.clicked.connect(build)
        output_and_buttons.addWidget(build_button)

        main_layout.addLayout(output_and_buttons)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        for t in self.tabs:
            try:
                t.init_ui()
            except Exception as e:
                print(e)
        
        for t in self.tabs:
            try:
                t.init_ui_for_mission()
            except Exception as e:
                print(e)
        self.recalculate()

    def recalculate(self):
        if not self.mission:
            return
        
        old = sys.stdout
        sys.stdout = self.text_box

        self.console_text_box.clear()

        try:
            print("#######################################")
            print("#        MISSION INFORMATION          #")
            print("#######################################")
            for i, t in enumerate(self.tabs):
                print()
                m = globals()[tabs[i]]
                importlib.reload(m)
                m.recalculate(t)
                print()
                print("---------------------------------------")

        except MissionValueError as v:
            self.console_text_box.clear()
            print(v.msg)

        sys.stdout = old

    def new_mission(self):
        if self.prompt_unsaved_should_continue():
            self.mission = Mission({})
            self.init_ui_for_mission()
            self.unsaved_changes = False

    def prompt_unsaved_should_continue(self):
        if DONT_PROMPT_UNSAVED:
            return True
        
        accept = False
        if self.unsaved_changes:
            reply = QMessageBox.question(self, 'Unsaved Changes',
                                         "You have unsaved changes. Do you want to save before exiting?",
                                         QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel, QMessageBox.Cancel)
            if reply == QMessageBox.Yes:
                if self.save_mission():
                    accept = True
            elif reply == QMessageBox.No:
                accept = True
        else:
            accept = True
        return accept

    def load_mission(self):
        if self.prompt_unsaved_should_continue():
            options = QFileDialog.Options()
            filename, _ = QFileDialog.getOpenFileName(self, "Load Mission", "", "JSON Files (*.json);;All Files (*)", options=options)
            if filename:
                with open(filename, 'r') as file:
                    data = json.load(file)
                    self.mission = Mission(data)
                    self.init_ui_for_mission()
                    self.unsaved_changes = False

    def save_mission(self):
        options = QFileDialog.Options()
        filename, _ = QFileDialog.getSaveFileName(self, "Save Mission", "", "JSON Files (*.json);;All Files (*)", options=options)
        if filename:
            with open(filename, 'w') as file:
                data = example_mission.copy()
                for key, _ in data.items():
                    if not key.startswith("__comment_"):
                        data[key] = getattr(self.mission, key)
                json.dump(data, file, indent=4)
                self.unsaved_changes = False
            return True
        else:
            return False

    def closeEvent(self, event):
        if self.prompt_unsaved_should_continue():
            event.accept()
        else:
            event.ignore()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    window = TakeMySelfControl()
    window.show()  

    app.exec()
