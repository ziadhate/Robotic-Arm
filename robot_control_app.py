import sys
import json
import threading
import time
from functools import partial

import numpy as np
import serial

from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton,
    QSlider, QSpinBox, QFileDialog, QMessageBox, QCheckBox, QLineEdit, QListWidget
)
from PySide6.QtCore import Qt, QTimer, QEvent

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# ---------------- Serial Manager ----------------
class SerialManager:
    """
    Manages Serial Communication with Arduino.
    Can connect, disconnect, and send joint angles.
    Thread-safe using a lock.
    """
    def __init__(self, port=None, baud=9600):
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()  # Ensures thread-safe serial access
        self.ser = None
        if port:
            self.connect(port)

    def connect(self, port):
        """
        Connect to Arduino on given COM port.
        Returns (success, message).
        """
        try:
            with self.lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(port, self.baud, timeout=0.5)
                self.port = port
            return True, f"Connected {port}"
        except Exception as e:
            self.ser = None
            return False, str(e)

    def disconnect(self):
        """
        Safely disconnect from Arduino.
        """
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None
                self.port = None

    def send_angles(self, angles):
        """
        Send a list of 6 joint angles to Arduino.
        Returns (success, message)
        """
        if not self.ser:
            return False, "Not connected"
        try:
            msg = ",".join(str(int(round(a))) for a in angles[:6]) + "\n"
            with self.lock:
                self.ser.write(msg.encode())
            return True, msg.strip()
        except Exception as e:
            return False, str(e)


# ---------------- Kinematics ----------------
class ArmKinematics:
    """
    Simple forward kinematics for a 4-link robotic arm.
    Calculates X,Y,Z positions of each joint based on angles.
    """
    def __init__(self, link_lengths=None):
        if link_lengths is None:
            link_lengths = [80, 100, 90, 50]  # Default link lengths in mm
        self.link_lengths = link_lengths

    def forward(self, angles_deg):
        """
        Compute forward kinematics.
        angles_deg: list of joint angles in degrees [base, th1, th2, th3, th4, th5]
        Returns arrays xs, ys, zs of joint positions.
        """
        a = np.radians(angles_deg)  # convert degrees to radians
        th_base, th1, th2, th3, th4 = a[:5]
        L1, L2, L3, L4 = self.link_lengths

        points = [(0, 0, 0)]  # base point

        # Compute each joint position using planar kinematics + base rotation
        r1 = L1*np.cos(th1)
        z1 = L1*np.sin(th1)
        x1 = r1*np.cos(th_base)
        y1 = r1*np.sin(th_base)
        points.append((x1, y1, z1))

        r2 = r1 + L2*np.cos(th1+th2)
        z2 = z1 + L2*np.sin(th1+th2)
        x2 = r2*np.cos(th_base)
        y2 = r2*np.sin(th_base)
        points.append((x2, y2, z2))

        r3 = r2 + L3*np.cos(th1+th2+th3)
        z3 = z2 + L3*np.sin(th1+th2+th3)
        x3 = r3*np.cos(th_base)
        y3 = r3*np.sin(th_base)
        points.append((x3, y3, z3))

        r4 = r3 + L4*np.cos(th1+th2+th3+th4)
        z4 = z3 + L4*np.sin(th1+th2+th3+th4)
        x4 = r4*np.cos(th_base)
        y4 = r4*np.sin(th_base)
        points.append((x4, y4, z4))

        xs, ys, zs = zip(*points)
        return np.array(xs), np.array(ys), np.array(zs)


# ---------------- Custom Qt Events ----------------
class _AngUpdateEvent(QEvent):
    """
    Custom event to update angles in GUI from threads.
    """
    TYPE = QEvent.Type(QEvent.registerEventType())
    def __init__(self, angles):
        super().__init__(self.TYPE)
        self.angles = angles

class _PauseButtonResetEvent(QEvent):
    """
    Custom event to reset pause button after sequence playback ends.
    """
    TYPE = QEvent.Type(QEvent.registerEventType())
    def __init__(self):
        super().__init__(self.TYPE)


# ---------------- GUI ----------------
class ProArmGUI(QWidget):
    """
    Main GUI for controlling robotic arm.
    Includes sliders, spin boxes, plotting, serial management, and sequence playback.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FABRI CREATOR - PRO (Stable)")
        self.resize(1100, 700)

        self.serial = SerialManager(None, 9600)  # Serial manager instance
        self.kin = ArmKinematics()               # Kinematics instance

        self.angles = [90]*6  # Initial joint angles
        self.sequence = []    # Recorded sequence steps
        self.playing = False
        self.pause_flag = False
        self.stop_flag = False

        self.pending_send = False
        self.pending_angles = self.angles.copy()

        self._build_ui()  # Build all GUI components

        # Timer to redraw plot periodically
        self.redraw_timer = QTimer()
        self.redraw_timer.setInterval(40)
        self.redraw_timer.timeout.connect(self.redraw_plot)
        self.redraw_timer.start()

        # Timer to flush pending serial sends
        self.send_timer = QTimer()
        self.send_timer.setInterval(50)
        self.send_timer.timeout.connect(self._flush_send)
        self.send_timer.start()


    # ---------------- Build UI ----------------
    def _build_ui(self):
        """
        Build all GUI widgets: sliders, buttons, plot, and layouts.
        """
        main_layout = QHBoxLayout(self)
        ctrl_layout = QVBoxLayout()
        main_layout.addLayout(ctrl_layout, 0)

        # COM Port input and connect button
        h = QHBoxLayout()
        self.port_input = QLineEdit()
        self.port_input.setPlaceholderText("COM port (e.g., COM7)")
        h.addWidget(self.port_input)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.handle_connect)
        h.addWidget(self.connect_btn)
        ctrl_layout.addLayout(h)

        # Joint sliders and spinboxes
        self.sliders = []
        for i in range(6):
            lbl = QLabel(f"Joint {i+1}")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0,180)
            slider.setValue(self.angles[i])
            spin = QSpinBox()
            spin.setRange(0,180)
            spin.setValue(self.angles[i])

            # Connect slider and spinbox signals
            slider.valueChanged.connect(partial(self.on_slider_change,i))
            slider.valueChanged.connect(spin.setValue)
            spin.valueChanged.connect(slider.setValue)
            spin.valueChanged.connect(partial(self.on_spin_change,i))
            row = QHBoxLayout()
            row.addWidget(lbl)
            row.addWidget(slider)
            row.addWidget(spin)
            ctrl_layout.addLayout(row)
            self.sliders.append((slider,spin))

        # Smooth motion options
        smooth_row = QHBoxLayout()
        self.smooth_chk = QCheckBox("Smooth")
        self.smooth_chk.setChecked(True)
        smooth_row.addWidget(self.smooth_chk)
        smooth_row.addWidget(QLabel("Interp Steps:"))
        self.interp_steps = QSpinBox()
        self.interp_steps.setRange(2,200)
        self.interp_steps.setValue(30)
        smooth_row.addWidget(self.interp_steps)
        smooth_row.addWidget(QLabel("Step delay ms:"))
        self.step_delay = QSpinBox()
        self.step_delay.setRange(1,2000)
        self.step_delay.setValue(15)
        smooth_row.addWidget(self.step_delay)
        ctrl_layout.addLayout(smooth_row)

        # Sequence buttons: record and clear
        seq_row = QHBoxLayout()
        self.record_btn = QPushButton("Record Step")
        self.record_btn.clicked.connect(self.record_step)
        self.clear_seq_btn = QPushButton("Clear Seq")
        self.clear_seq_btn.clicked.connect(self.clear_sequence)
        seq_row.addWidget(self.record_btn)
        seq_row.addWidget(self.clear_seq_btn)
        ctrl_layout.addLayout(seq_row)

        # Sequence list display
        self.seq_list = QListWidget()
        ctrl_layout.addWidget(self.seq_list)

        # Save/Load buttons
        file_row = QHBoxLayout()
        self.save_btn = QPushButton("Save Seq")
        self.save_btn.clicked.connect(self.save_sequence)
        self.load_btn = QPushButton("Load Seq")
        self.load_btn.clicked.connect(self.load_sequence)
        file_row.addWidget(self.save_btn)
        file_row.addWidget(self.load_btn)
        ctrl_layout.addLayout(file_row)

        # Playback buttons: play, pause, stop
        play_row = QHBoxLayout()
        self.play_btn = QPushButton("Play")
        self.play_btn.clicked.connect(self.play_sequence)
        self.pause_btn = QPushButton("Pause")
        self.pause_btn.clicked.connect(self.pause_sequence)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_sequence)
        play_row.addWidget(self.play_btn)
        play_row.addWidget(self.pause_btn)
        play_row.addWidget(self.stop_btn)
        ctrl_layout.addLayout(play_row)

        # Extra buttons: send now, reset, home
        extra_row = QHBoxLayout()
        self.send_btn = QPushButton("Send Now")
        self.send_btn.clicked.connect(self.send_now_manual)
        self.reset_btn = QPushButton("Reset 90")
        self.reset_btn.clicked.connect(self.reset_angles)
        self.home_btn = QPushButton("Home")
        self.home_btn.clicked.connect(self.home_angles)
        extra_row.addWidget(self.send_btn)
        extra_row.addWidget(self.reset_btn)
        extra_row.addWidget(self.home_btn)
        ctrl_layout.addLayout(extra_row)

        # Status label
        self.status_lbl = QLabel("Disconnected")
        ctrl_layout.addWidget(self.status_lbl)
        ctrl_layout.addStretch(1)

        # Plot area
        plot_layout = QVBoxLayout()
        main_layout.addLayout(plot_layout, 1)
        self.fig = Figure(figsize=(6,6))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        plot_layout.addWidget(self.canvas)
        self.redraw_plot()


    # ---------------- Serial / UI Handlers ----------------
    def handle_connect(self):
        """
        Connect to COM port when Connect button clicked.
        """
        port = self.port_input.text().strip()
        if not port:
            QMessageBox.warning(self, "Port required", "Enter COM port (e.g., COM7)")
            return
        ok, msg = self.serial.connect(port)
        if ok:
            self.status_lbl.setText(f"Connected {port}")
            self.connect_btn.setText("Disconnect")
            self.connect_btn.clicked.disconnect()
            self.connect_btn.clicked.connect(self.handle_disconnect)
        else:
            QMessageBox.critical(self, "Connect Error", msg)
            self.status_lbl.setText("Connect failed")

    def handle_disconnect(self):
        """
        Disconnect when Disconnect button clicked.
        """
        self.serial.disconnect()
        self.status_lbl.setText("Disconnected")
        self.connect_btn.setText("Connect")
        self.connect_btn.clicked.disconnect()
        self.connect_btn.clicked.connect(self.handle_connect)

    # ---------------- Angle Change Handlers ----------------
    def on_slider_change(self, idx, value):
        self.update_angle(idx, value)

    def on_spin_change(self, idx, value):
        self.update_angle(idx, value)

    def update_angle(self, idx, value):
        """
        Update internal angle array when slider or spinbox changed.
        """
        if idx == 3:  # special scale for joint 4
            value = int(value * 0.4)
        self.angles[idx] = int(value)
        self.pending_angles = self.angles.copy()
        self.pending_send = True

    # --------------- Updated Serial Send in Thread ----------------
    def _flush_send(self):
        """
        Periodically called to send pending angles without blocking GUI.
        """
        if self.pending_send and not self.playing:
            self.pending_send = False
            threading.Thread(target=self._send_thread, args=(self.pending_angles.copy(),), daemon=True).start()

    def send_now_manual(self):
        """
        Send angles immediately when button clicked.
        """
        threading.Thread(target=self._send_thread, args=(self.angles.copy(),), daemon=True).start()

    def _send_thread(self, angles_to_send):
        """
        Actual thread function to send angles to Arduino.
        Updates status label.
        """
        ok, msg = self.serial.send_angles(angles_to_send)
        if not ok:
            self.status_lbl.setText(f"Send failed: {msg}")
        else:
            self.status_lbl.setText(f"Sent: {msg}")

    # ---------------- Reset/Home ----------------
    def reset_angles(self):
        """
        Reset all joints to 90 degrees.
        """
        self.angles = [90]*6
        for s, spin in self.sliders:
            s.blockSignals(True)
            spin.blockSignals(True)
            s.setValue(90)
            spin.setValue(90)
            s.blockSignals(False)
            spin.blockSignals(False)
        self.pending_angles = self.angles.copy()
        self.pending_send = True

    def home_angles(self):
        """
        Set joints to predefined home positions.
        """
        home = [90,100,60,120,90,60]
        self.angles = home.copy()
        for i, (s, spin) in enumerate(self.sliders):
            s.blockSignals(True)
            spin.blockSignals(True)
            s.setValue(home[i])
            spin.setValue(home[i])
            s.blockSignals(False)
            spin.blockSignals(False)
        self.pending_angles = self.angles.copy()
        self.pending_send = True

    # ---------------- Sequence Functions ----------------
    def record_step(self):
        """
        Record current angles into sequence list.
        """
        ang = self.angles.copy()
        self.sequence.append(ang)
        self.seq_list.addItem(",".join(str(a) for a in ang))

    def clear_sequence(self):
        """
        Clear recorded sequence.
        """
        self.sequence = []
        self.seq_list.clear()

    def save_sequence(self):
        """
        Save sequence to JSON file.
        """
        if not self.sequence:
            QMessageBox.information(self, "Empty", "No sequence")
            return
        fname, _ = QFileDialog.getSaveFileName(self, "Save Sequence", "", "JSON Files (*.json)")
        if fname:
            json.dump(self.sequence, open(fname, "w"))
            QMessageBox.information(self, "Saved", f"{len(self.sequence)} steps")

    def load_sequence(self):
        """
        Load sequence from JSON file.
        """
        fname, _ = QFileDialog.getOpenFileName(self, "Load Sequence", "", "JSON Files (*.json)")
        if fname:
            seq = json.load(open(fname, "r"))
            self.sequence = seq
            self.seq_list.clear()
            for s in seq:
                self.seq_list.addItem(",".join(str(int(x)) for x in s))

    # ---------------- Playback ----------------
    def play_sequence(self):
        """
        Start playback of recorded sequence in a separate thread.
        """
        if not self.sequence or self.playing:
            return
        self.playing = True
        self.pause_flag = False
        self.stop_flag = False
        threading.Thread(target=self._play_thread, daemon=True).start()

    def pause_sequence(self):
        """
        Toggle pause/resume during playback.
        """
        self.pause_flag = not self.pause_flag
        self.pause_btn.setText("Resume" if self.pause_flag else "Pause")

    def stop_sequence(self):
        """
        Stop playback immediately.
        """
        self.stop_flag = True
        self.playing = False

    def _play_thread(self):
        """
        Thread function to playback sequence with optional smoothing.
        """
        steps = self.sequence.copy()
        idx = 0
        while not self.stop_flag and self.playing and idx < len(steps):
            target = steps[idx]
            if self.smooth_chk.isChecked():
                steps_count = self.interp_steps.value()
                for tstep in range(1, steps_count + 1):
                    if self.stop_flag: break
                    while self.pause_flag:
                        time.sleep(0.05)
                        if self.stop_flag: break
                    frac = tstep / steps_count
                    interp = [self.angles[i] + (target[i] - self.angles[i]) * frac for i in range(6)]
                    QApplication.instance().postEvent(self, _AngUpdateEvent(interp))
                    threading.Thread(target=self._send_thread, args=(interp,), daemon=True).start()
                    time.sleep(self.step_delay.value() / 1000.0)
            self.angles = target.copy()
            QApplication.instance().postEvent(self, _AngUpdateEvent(self.angles))
            threading.Thread(target=self._send_thread, args=(self.angles.copy(),), daemon=True).start()
            time.sleep(self.step_delay.value() / 1000.0)
            idx += 1
        self.playing = False
        self.pause_flag = False
        QApplication.instance().postEvent(self, _PauseButtonResetEvent())

    # ---------------- Plot ----------------
    def redraw_plot(self):
        """
        Redraw 3D arm plot based on current joint angles.
        """
        self.ax.cla()
        xs, ys, zs = self.kin.forward(self.angles)
        self.ax.plot(xs, ys, zs, "-o", linewidth=3, markersize=6)
        R = sum(self.kin.link_lengths[:3]) + 20
        self.ax.set_xlim(-R, R)
        self.ax.set_ylim(-R, R)
        self.ax.set_zlim(0, R * 0.8)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.canvas.draw_idle()

    def customEvent(self, event):
        """
        Handle custom Qt events: update angles and reset pause button.
        """
        if isinstance(event, _AngUpdateEvent):
            vals = event.angles
            self.angles = [int(round(x)) for x in vals]
            for i, (s, spin) in enumerate(self.sliders):
                s.blockSignals(True)
                spin.blockSignals(True)
                s.setValue(self.angles[i])
                spin.setValue(self.angles[i])
                s.blockSignals(False)
                spin.blockSignals(False)
        elif isinstance(event, _PauseButtonResetEvent):
            self.pause_btn.setText("Pause")
            self.status_lbl.setText("Stopped")


# ---------------- Run Application ----------------
def main():
    """
    Run the PySide6 GUI application.
    """
    app = QApplication(sys.argv)
    w = ProArmGUI()
    w.show()
    app.aboutToQuit.connect(lambda: w.serial.disconnect())
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
