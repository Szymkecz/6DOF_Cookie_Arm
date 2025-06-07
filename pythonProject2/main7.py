import sys
import math
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QIntValidator
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel,
    QLineEdit, QPushButton, QComboBox, QTabWidget, QHBoxLayout
)
import serial
from PyQt5.QtCore import QEventLoop, QTimer

import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph import PlotWidget

# Stałe
Time_start = 20
start_c_press = 10
start_press = 10
start_press = 10
MAX_DATA_POINTS = 8000
SAMPLE_RATE = 5  # Próbkowanie co 5 punktów


class RobotJogApp(QWidget):
    def __init__(self):
        super().__init__()
        self.ser = None
        self.initUI()

        self.serial_update_timer = QTimer(self)
        self.serial_update_timer.timeout.connect(self.read_serial_data)
        self.serial_update_timer.start(Time_start)

        self.send_values_timer = QTimer(self)
        self.send_values_timer.timeout.connect(self.send_joint_values)

        self.send_c_values_timer = QTimer(self)
        self.send_c_values_timer.timeout.connect(self.send_c_values)

        self.send_s_values_timer = QTimer(self)
        self.send_s_values_timer.timeout.connect(self.send_s_values)
        self.data_counter = 0

    def initUI(self):
        self.setWindowTitle("Robot Jog Control")
        layout = QVBoxLayout()

        port_layout = QGridLayout()
        port_label = QLabel("Port:")
        baud_label = QLabel("Baudrate:")

        self.port_combo = QComboBox()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

        if not ports:
            self.port_combo.addItem("COM1")

        self.baudrate_input = QLineEdit("115200")
        self.baudrate_input.setValidator(QIntValidator(1, 10000000))

        self.connect_button = QPushButton("Połącz")
        self.connect_button.clicked.connect(self.connect_serial)

        port_layout.addWidget(port_label, 0, 0)
        port_layout.addWidget(self.port_combo, 0, 1)
        port_layout.addWidget(baud_label, 0, 2)
        port_layout.addWidget(self.baudrate_input, 0, 3)
        port_layout.addWidget(self.connect_button, 0, 4)

        layout.addLayout(port_layout)

        self.tabs = QTabWidget()

        self.jog_tab = QWidget()
        self.jog_layout = QVBoxLayout(self.jog_tab)
        self.tabs.addTab(self.jog_tab, "JOG")
        self.init_jog_tab()

        self.chart_tab = QWidget()
        self.chart_layout = QVBoxLayout(self.chart_tab)
        self.tabs.addTab(self.chart_tab, "Wykres")
        self.init_chart_tab()

        self.config_tab = QWidget()
        self.config_layout = QVBoxLayout(self.config_tab)
        self.tabs.addTab(self.config_tab, "Config")
        self.init_config_tab()

        layout.addWidget(self.tabs)

        bottom_layout = QVBoxLayout()

        self.reset_button = QPushButton("Reset")
        self.reset_button.setStyleSheet("background-color: white")
        self.reset_button.clicked.connect(self.send_reset_command)
        bottom_layout.addWidget(self.reset_button)

        self.power_button = QPushButton("ON")
        self.power_button.setStyleSheet("background-color: white")
        self.power_state = True
        self.power_button.clicked.connect(self.toggle_power)
        bottom_layout.addWidget(self.power_button)

        exit_button = QPushButton("Exit")
        exit_button.clicked.connect(self.close)
        bottom_layout.addWidget(exit_button)

        layout.addLayout(bottom_layout)
        self.setLayout(layout)

    def init_jog_tab(self):
        grid = QGridLayout()
        grid.setSpacing(5)

        self.joint_labels = []
        self.joint_values = []
        joints = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
        default_values = ["0"] * 7
        self.joint_press_states = [0] * 7
        self.pending_press_states = [0] * 7
        self.press_delay_timers = [QTimer(self) for _ in range(7)]

        for i, joint in enumerate(joints):
            self.joint_labels.append(QLabel(joint))
            value_field = QLineEdit(default_values[i])
            value_field.setReadOnly(True)
            self.joint_values.append(value_field)

            inc_button = QPushButton("+")
            dec_button = QPushButton("-")

            self.press_delay_timers[i].setSingleShot(True)
            self.press_delay_timers[i].timeout.connect(lambda i=i: self.apply_pending_state(i))

            inc_button.pressed.connect(lambda i=i: self.start_press_delay(i, 1))
            inc_button.released.connect(lambda i=i: self.cancel_press(i))
            dec_button.pressed.connect(lambda i=i: self.start_press_delay(i, 2))
            dec_button.released.connect(lambda i=i: self.cancel_press(i))

            grid.addWidget(self.joint_labels[i], i, 0)
            grid.addWidget(self.joint_values[i], i, 1)
            grid.addWidget(inc_button, i, 2)
            grid.addWidget(dec_button, i, 3)

        c_labels = ["C1", "C2", "C3", "C4", "C5", "C6"]
        self.c_labels = []
        self.c_values = []
        self.c_press_states = [0] * 6
        self.c_pending_press_states = [0] * 6
        self.c_press_delay_timers = [QTimer(self) for _ in range(6)]

        for i, label in enumerate(c_labels):
            c_label = QLabel(label)
            self.c_labels.append(c_label)
            c_value_field = QLineEdit("0")
            c_value_field.setReadOnly(True)
            self.c_values.append(c_value_field)

            inc_button = QPushButton("+")
            dec_button = QPushButton("-")

            self.c_press_delay_timers[i].setSingleShot(True)
            self.c_press_delay_timers[i].timeout.connect(lambda i=i: self.apply_c_pending_state(i))

            inc_button.pressed.connect(lambda i=i: self.start_c_press_delay(i, 1))
            inc_button.released.connect(lambda i=i: self.cancel_c_press(i))
            dec_button.pressed.connect(lambda i=i: self.start_c_press_delay(i, 2))
            dec_button.released.connect(lambda i=i: self.cancel_c_press(i))

            grid.addWidget(c_label, i, 4)
            grid.addWidget(c_value_field, i, 5)
            grid.addWidget(inc_button, i, 6)
            grid.addWidget(dec_button, i, 7)

        angular_speed_label = QLabel("Angular Speed:")
        self.angular_speed_input = QLineEdit("5")
        self.angular_speed_input.setValidator(QIntValidator(0, 100))
        grid.addWidget(angular_speed_label, len(joints), 0)
        grid.addWidget(self.angular_speed_input, len(joints), 1)

        linear_speed_label = QLabel("Linear Speed:")
        self.linear_speed_input = QLineEdit("5")
        self.linear_speed_input.setValidator(QIntValidator(0, 100))
        grid.addWidget(linear_speed_label, len(joints), 4)
        grid.addWidget(self.linear_speed_input, len(joints), 5)

        button_layout = QHBoxLayout()

        self.record_button = QPushButton("Record Pos")
        self.record_button.setStyleSheet("background-color: white")
        self.record_button.clicked.connect(self.send_record_command)
        button_layout.addWidget(self.record_button)

        self.play_button = QPushButton("Play")
        self.play_button.setStyleSheet("background-color: white")
        self.play_button.clicked.connect(self.send_play_command)
        button_layout.addWidget(self.play_button)

        self.stop_button = QPushButton("Stop")
        self.stop_button.setStyleSheet("background-color: white")
        self.stop_button.clicked.connect(self.send_stop_command)
        button_layout.addWidget(self.stop_button)

        grid.addLayout(button_layout, len(joints) + 1, 0, 1, 8)

        self.jog_layout.addLayout(grid)

    def init_chart_tab(self):
        self.graphWidget = pg.GraphicsLayoutWidget()
        self.chart_layout.addWidget(self.graphWidget)

        self.pos_plot = self.graphWidget.addPlot(title="Servo Position", row=0, col=0)
        self.pos_plot.setLabel('left', "Value")
        self.pos_plot.setLabel('bottom', "Time")
        self.pos_plot.addLegend()

        zero_line_pos = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('w', width=1))
        self.pos_plot.addItem(zero_line_pos)

        self.vel_plot = self.graphWidget.addPlot(title="Servo Speed", row=1, col=0)
        self.vel_plot.setLabel('left', "Value")
        self.vel_plot.setLabel('bottom', "Time")
        self.vel_plot.addLegend()

        zero_line_vel = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('w', width=1))
        self.vel_plot.addItem(zero_line_vel)

        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

        self.pos_curves = []
        self.vel_curves = []

        for i in range(7):
            pos_curve = self.pos_plot.plot([], [], name=f"J{i + 1}", pen=colors[i])
            vel_curve = self.vel_plot.plot([], [], name=f"J{i + 1}", pen=colors[i])
            self.pos_curves.append(pos_curve)
            self.vel_curves.append(vel_curve)

        # Dane
        self.x_data = []
        self.joint_data = [[] for _ in range(7)]  # Dla 7 jointów
        self.diff_data = [[] for _ in range(7)]  # Różnice (prędkości)

    def init_config_tab(self):
        grid = QGridLayout()
        grid.setSpacing(5)

        self.s_labels = []
        self.s_values = []
        s_labels = ["S1", "S2", "S3", "S4", "S5", "S6", "S7"]
        default_values = ["0"] * 7

        self.s_press_states = [0] * 7
        self.s_pending_press_states = [0] * 7
        self.s_press_delay_timers = [QTimer(self) for _ in range(7)]

        for i, label in enumerate(s_labels):
            self.s_labels.append(QLabel(label))
            value_field = QLineEdit(default_values[i])
            value_field.setReadOnly(True)
            self.s_values.append(value_field)

            inc_button = QPushButton("+")
            dec_button = QPushButton("-")

            self.s_press_delay_timers[i].setSingleShot(True)
            self.s_press_delay_timers[i].timeout.connect(lambda i=i: self.apply_s_pending_state(i))

            inc_button.pressed.connect(lambda i=i: self.start_s_press_delay(i, 1))
            inc_button.released.connect(lambda i=i: self.cancel_s_press(i))
            dec_button.pressed.connect(lambda i=i: self.start_s_press_delay(i, 2))
            dec_button.released.connect(lambda i=i: self.cancel_s_press(i))

            grid.addWidget(self.s_labels[i], i + 1, 0)
            grid.addWidget(self.s_values[i], i + 1, 1)
            grid.addWidget(inc_button, i + 1, 2)
            grid.addWidget(dec_button, i + 1, 3)

        speed_label = QLabel("Speed:")
        self.speed_input = QLineEdit("5")
        self.speed_input.setValidator(QIntValidator(0, 100))
        grid.addWidget(speed_label, len(s_labels) + 1, 0)
        grid.addWidget(self.speed_input, len(s_labels) + 1, 1)

        self.config_state = False
        self.config_button = QPushButton("Start Config")
        self.config_button.clicked.connect(self.toggle_config_state)
        grid.addWidget(self.config_button, len(s_labels) + 2, 0, 1, 3)

        self.config_layout.addLayout(grid)

    def send_record_command(self):
        if self.ser:
            try:
                message = 'RECORD_POS;\n'
                message = message.ljust(21)
                self.ser.write(message.encode())
                self.ser.write(message.encode())
                self.ser.write(message.encode())

                print(f"Wysłana komenda RECORD_POS: {message.strip()}")
            except Exception as e:
                print(f"Błąd przy wysyłaniu RECORD_POS: {e}")
        else:
            print("Błąd: Port szeregowy nie jest połączony!")

    def send_play_command(self):
        if self.ser:
            try:
                message = 'PLAY_POS;\n'
                message = message.ljust(21)
                self.ser.write(message.encode())
                self.ser.write(message.encode())
                self.ser.write(message.encode())

                print(f"Wysłana komenda PLAY_POS: {message.strip()}")
            except Exception as e:
                print(f"Błąd przy wysyłaniu PLAY_POS: {e}")
        else:
            print("Błąd: Port szeregowy nie jest połączony!")

    def send_stop_command(self):
        if self.ser:
            try:
                message = 'STOP_POS;\n'
                message = message.ljust(21)
                self.ser.write(message.encode())
                self.ser.write(message.encode())
                self.ser.write(message.encode())

                print(f"Wysłana komenda STOP_POS: {message.strip()}")
            except Exception as e:
                print(f"Błąd przy wysyłaniu STOP_POS: {e}")
        else:
            print("Błąd: Port szeregowy nie jest połączony!")

    def toggle_config_state(self):
        if self.ser:
            try:
                message = 'ON_CONFIG;\n' if not self.config_state else 'OFF_CONFIG;\n'
                message = message.ljust(21)  # Lepsze niż ręczne dodawanie spacji
                self.ser.write(message.encode())
                print(f"Wysłana wiadomość CONFIG: {message.strip()}")  # Poprawione
                self.config_button.setText("Off Config" if not self.config_state else "Start Config")
                self.config_state = not self.config_state
            except Exception as e:
                print(f"Błąd CONFIG: {e}")

    def connect_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.connect_button.setText("Połącz")
            # print("Rozłączono.")
        else:
            port = self.port_combo.currentText()
            try:
                baudrate = int(self.baudrate_input.text())
            except ValueError:
                # print("Nieprawidłowa wartość baudrate")
                return

            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.05)
                self.connect_button.setText("Rozłącz")
            except Exception as e:
                self.ser = None
                # print(f"Błąd połączenia: {e}")

    def toggle_power(self):
        if self.ser:
            try:
                if not self.power_state:
                    message = 'POWER_ON;\n'
                    self.power_button.setText("ON")
                    self.power_button.setStyleSheet("background-color: white")
                else:
                    message = 'POWER_OFF;\n'
                    self.power_button.setText("OFF")
                    self.power_button.setStyleSheet("background-color: white")

                message = message.ljust(21)
                for _ in range(3):
                    self.ser.write(message.encode())
                    self.ser.flush()
                    print(f"Wysłano: {message.strip()}")
                    QApplication.processEvents()

                self.power_state = not self.power_state

            except Exception as e:
                print(f"Błąd przy wysyłaniu POWER: {e}")
        else:
            print("Błąd: Port szeregowy nie jest połączony!")

    def set_press_state(self, index, state):
        self.joint_press_states[index] = state
        self.update_joint_timer()

    def set_c_press_state(self, index, state):
        self.c_press_states[index] = state
        self.update_cartesian_timer()

    def set_s_press_state(self, index, state):
        self.s_press_states[index] = state
        self.update_s_timer()

    def update_joint_timer(self):
        if any(self.joint_press_states):
            if not self.send_values_timer.isActive():
                self.send_values_timer.start(5)
        else:
            self.send_values_timer.stop()

    def update_cartesian_timer(self):
        if any(self.c_press_states):
            if not self.send_c_values_timer.isActive():
                self.send_c_values_timer.start(5)
        else:
            self.send_c_values_timer.stop()

    def update_s_timer(self):
        if any(self.s_press_states):
            if not self.send_s_values_timer.isActive():
                self.send_s_values_timer.start(5)
        else:
            self.send_s_values_timer.stop()

    def start_press_delay(self, index, state):
        self.pending_press_states[index] = state
        self.press_delay_timers[index].start(start_press)

    def start_c_press_delay(self, index, state):
        self.c_pending_press_states[index] = state
        self.c_press_delay_timers[index].start(start_c_press)

    def start_s_press_delay(self, index, state):
        self.s_pending_press_states[index] = state
        self.s_press_delay_timers[index].start(start_press)

    def apply_pending_state(self, index):
        self.set_press_state(index, self.pending_press_states[index])

    def apply_c_pending_state(self, index):
        self.set_c_press_state(index, self.c_pending_press_states[index])

    def apply_s_pending_state(self, index):
        self.set_s_press_state(index, self.s_pending_press_states[index])

    def cancel_press(self, index):
        self.press_delay_timers[index].stop()
        self.pending_press_states[index] = 0
        self.set_press_state(index, 0)
        self.send_joint_values()

    def cancel_c_press(self, index):
        self.c_press_delay_timers[index].stop()
        self.c_pending_press_states[index] = 0
        self.set_c_press_state(index, 0)
        self.send_c_values()

    def cancel_s_press(self, index):
        self.s_press_delay_timers[index].stop()
        self.s_pending_press_states[index] = 0
        self.set_s_press_state(index, 0)
        self.send_s_values()

    def get_scaled_angular_speed(self):
        try:
            value = int(self.angular_speed_input.text())
            return max(5, min(100, value)) / 50.0
        except ValueError:
            return 0.0

    def get_scaled_linear_speed(self):
        try:
            value = int(self.linear_speed_input.text())
            return max(5, min(100, value)) / 50.0
        except ValueError:
            return 0.0

    def get_scaled_speed(self):
        try:
            value = int(self.speed_input.text())
            return max(5, min(100, value)) / 50.0
        except ValueError:
            return 0.0

    def read_serial_data(self):
        try:
            if self.ser and self.ser.in_waiting > 0:
                # Czyszczenie bufora przed odczytem
                self.ser.reset_input_buffer()

                raw_data = self.ser.read_until(b';')
                line = raw_data.decode('utf-8', errors='ignore').strip()

                if line.endswith(";"):
                    if line.startswith("JC[") and "]" in line:
                        values_str = line[line.find("[") + 1:line.find("]")]
                        values = values_str.split(",")

                        if len(values) == 7:
                            validated_values = []
                            for val in values:
                                try:
                                    num = round(float(val), 2)
                                    if not math.isnan(num) and math.isfinite(num):
                                        validated_values.append(num)
                                    else:
                                        validated_values.append(0.0)
                                except (ValueError, TypeError):
                                    validated_values.append(0.0)

                            for i in range(7):
                                self.joint_values[i].setText(f"{validated_values[i]:.2f}")

                            self.data_counter += 1
                            if self.data_counter % SAMPLE_RATE != 0:
                                return

                            self.x_data.append(len(self.x_data))

                            for i in range(7):
                                self.joint_data[i].append(validated_values[i])

                                if len(self.joint_data[i]) > 1:
                                    value_diff = self.joint_data[i][-1] - self.joint_data[i][-2]
                                    diff = round(value_diff / SAMPLE_RATE, 4)

                                    if abs(diff) > 0.1:
                                        diff = round((self.diff_data[i][-1] * 0.9 + diff * 0.1),
                                                     3)

                                    self.diff_data[i].append(diff)
                                else:
                                    self.diff_data[i].append(0.0)

                                if len(self.x_data) > MAX_DATA_POINTS:
                                    self.x_data = self.x_data[-MAX_DATA_POINTS:]
                                    self.joint_data[i] = self.joint_data[i][-MAX_DATA_POINTS:]
                                    self.diff_data[i] = self.diff_data[i][-MAX_DATA_POINTS:]

                            for i in range(7):
                                x_vals = self.x_data[-len(self.joint_data[i]):]
                                y_vals = self.joint_data[i]
                                self.pos_curves[i].setData(x_vals, y_vals)

                            for i in range(7):
                                x_vals = self.x_data[-len(self.diff_data[i]):]
                                y_vals = self.diff_data[i]
                                self.vel_curves[i].setData(x_vals, y_vals)

                    # Przetwarzanie ramki CC (Cartesian Current) - NOWA POPRAWNA LOGIKA
                    elif line.startswith("CC[") and "]" in line:
                        values_str = line[line.find("[") + 1:line.find("]")]
                        values = values_str.split(",")
                        if len(values) == 6:
                            for i in range(6):
                                try:
                                    val = round(float(values[i]), 2)
                                    self.c_values[i].setText(f"{val:.2f}")
                                except (ValueError, TypeError):
                                    self.c_values[i].setText("0.00")

                    # Przetwarzanie ramki CPWM
                    elif line.startswith("CPWM[") and "]" in line:
                        values_str = line[line.find("[") + 1:line.find("]")]
                        values = values_str.split(",")
                        if len(values) == 7:
                            for i in range(7):
                                if i < len(self.s_values):
                                    try:
                                        val = round(float(values[i]), 2)
                                        self.s_values[i].setText(f"{val:.2f}")
                                    except (ValueError, TypeError):
                                        self.s_values[i].setText("0.00")

        except serial.SerialException as se:
            print(f"Błąd portu szeregowego: {se}")
        except UnicodeDecodeError as ude:
            print(f"Błąd dekodowania danych: {ude}")
        except Exception as e:
            print(f"Nieoczekiwany błąd: {type(e).__name__}: {str(e)[:100]}")

    def send_joint_values(self):
        try:
            if self.ser:
                speed = 0 if all(state == 0 for state in self.joint_press_states) else self.get_scaled_angular_speed()
                state_str = ",".join(str(s) for s in self.joint_press_states)
                i = 1
                time = 0

                if all(state == 0 for state in self.joint_press_states):
                    state_str = "0,0,0,0,0,0,0"
                    i = 3
                    time = 5

                for a in range(i):
                    message = f"JN[{speed},{state_str}];\n"
                    message = message[:21]
                    message += ' ' * (21 - len(message))
                    print(f"Wysłana wiadomość: {message.strip()}")
                    self.ser.write(message.encode())
                    # Czekaj
                    loop = QEventLoop()
                    QTimer.singleShot(time, loop.quit)
                    loop.exec_()
        except Exception as e:
            print(f"Błąd: {e}")

    def send_c_values(self):
        try:
            if self.ser:
                speed = 0 if all(state == 0 for state in self.c_press_states) else self.get_scaled_linear_speed()
                state_str = ",".join(str(s) for s in self.c_press_states)

                if all(state == 0 for state in self.c_press_states):
                    state_str = "0,0,0,0,0,0"

                message = f"CN[{speed},{state_str}];\n"
                message = message[:21]
                message += ' ' * (21 - len(message))
                self.ser.write(message.encode())
        except Exception as e:
            print(f"Błąd CN: {e}")

    def send_s_values(self):
        try:
            if self.ser:
                speed = 0 if all(state == 0 for state in self.s_press_states) else self.get_scaled_speed()
                state_str = ",".join(str(s) for s in self.s_press_states)

                if all(state == 0 for state in self.s_press_states):
                    state_str = "0,0,0,0,0,0,0"

                message = f"NPWM[{speed},{state_str}];\n"
                message = message[:21]
                message += ' ' * (21 - len(message))
                # print(f"Wysłana wiadomość NPWM: {message.strip()}")
                self.ser.write(message.encode())
        except Exception as e:
            print(f"Błąd: {e}")

    def send_reset_command(self):
        if self.ser:
            try:
                message = 'RESET;\n'
                message = message.ljust(21)
                self.ser.write(message.encode())
                print(f"Wysłana komenda RESET: {message.strip()}")
            except Exception as e:
                print(f"Błąd przy wysyłaniu RESET: {e}")
        else:
            print("Błąd: Port szeregowy nie jest połączony!")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    robot_jog_app = RobotJogApp()
    robot_jog_app.show()
    sys.exit(app.exec_())