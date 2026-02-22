#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5 import QtWidgets
from collections import deque
import sys
import signal

def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        mfr = (port.manufacturer or "").lower()
        if any(name in mfr for name in ['silicon labs', 'espressif', 'cp210', 'ch340']):
            return port.device
    return ports[0].device if ports else None

PORT = find_esp32_port()
BAUD = 115200
WINDOW_SIZE = 200

print(f"Connecting to {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=0.01)

# Data buffers
accel_x = deque(maxlen=WINDOW_SIZE)
accel_y = deque(maxlen=WINDOW_SIZE)
accel_z = deque(maxlen=WINDOW_SIZE)
vel_x = deque(maxlen=WINDOW_SIZE)
vel_y = deque(maxlen=WINDOW_SIZE)
vel_z = deque(maxlen=WINDOW_SIZE)
pos_x = deque(maxlen=WINDOW_SIZE)
pos_y = deque(maxlen=WINDOW_SIZE)
pos_z = deque(maxlen=WINDOW_SIZE)
gyro_x = deque(maxlen=WINDOW_SIZE)
gyro_y = deque(maxlen=WINDOW_SIZE)
gyro_z = deque(maxlen=WINDOW_SIZE)
attitude_x = deque(maxlen=WINDOW_SIZE)
attitude_y = deque(maxlen=WINDOW_SIZE)
attitude_z = deque(maxlen=WINDOW_SIZE)

# Create application
app = QtWidgets.QApplication(sys.argv)

# Create window
win = pg.GraphicsLayoutWidget(show=True, title="Live IMU Data")
# win = QtWidgets.QWidget()
# layout = QtWidgets.QGridLayout(win)
# layout.addWidget(Qt)
win.resize(1200, 800)

# Create plots
p1 = win.addPlot(row=0, col=0, title="Accelerometer")
p1.setLabel('left', 'Acceleration', units='m/s^2')
p1.addLegend()
curve_ax = p1.plot(pen='r', name='X')
curve_ay = p1.plot(pen='g', name='Y')
curve_az = p1.plot(pen='b', name='Z')

# Create plots
p1 = win.addPlot(row=0, col=1, title="Velocity")
p1.setLabel('left', 'Velocity', units='m/s')
p1.addLegend()
curve_vx = p1.plot(pen='r', name='X')
curve_vy = p1.plot(pen='g', name='Y')
curve_vz = p1.plot(pen='b', name='Z')

# Create plots
p1 = win.addPlot(row=0, col=2, title="Position")
p1.setLabel('left', 'Position', units='m')
p1.addLegend()
curve_rx = p1.plot(pen='r', name='X')
curve_ry = p1.plot(pen='g', name='Y')
curve_rz = p1.plot(pen='b', name='Z')

p2 = win.addPlot(row=1, col=1, title="Gyroscope")
p2.setLabel('left', 'Angular velocity', units='deg/s')
p2.addLegend()
curve_gx = p2.plot(pen='r', name='X')
curve_gy = p2.plot(pen='g', name='Y')
curve_gz = p2.plot(pen='b', name='Z')

p2 = win.addPlot(row=1, col=2, title="Attitude")
p2.setLabel('left', 'Angular position', units='deg')
p2.addLegend()
curve_attx = p2.plot(pen='r', name='Phi')
curve_atty = p2.plot(pen='g', name='Theta')
curve_attz = p2.plot(pen='b', name='Psi')

def update():
    """Read serial and update plots"""
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line[0].isdigit():
            values = line.split(',')
            if len(values) == 16:
                accel_x.append(float(values[1]))
                accel_y.append(float(values[2]))
                accel_z.append(float(values[3]))
                vel_x.append(float(values[4]))
                vel_y.append(float(values[5]))
                vel_z.append(float(values[6]))
                pos_x.append(float(values[7]))
                pos_y.append(float(values[8]))
                pos_z.append(float(values[9]))
                gyro_x.append(float(values[10]))
                gyro_y.append(float(values[11]))
                gyro_z.append(float(values[12]))
                attitude_x.append(float(values[13]))
                attitude_y.append(float(values[14]))
                attitude_z.append(float(values[15]))
                
                # Update curves
                curve_ax.setData(accel_x)
                curve_ay.setData(accel_y)
                curve_az.setData(accel_z)
                curve_vx.setData(vel_x)
                curve_vy.setData(vel_y)
                curve_vz.setData(vel_z)
                curve_rx.setData(pos_x)
                curve_ry.setData(pos_y)
                curve_rz.setData(pos_z)
                curve_gx.setData(gyro_x)
                curve_gy.setData(gyro_y)
                curve_gz.setData(gyro_z)
                curve_attx.setData(attitude_x)
                curve_atty.setData(attitude_y)
                curve_attz.setData(attitude_z)
    except Exception as e:
        pass

# Set up timer for updates
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)  # Update every 10ms

# # Start Qt event loop
# sys.exit(app.exec_())

def cleanup():
    """Clean shutdown"""
    print("\nCleaning up...")
    timer.stop()
    ser.close()
    print("Serial port closed")
    app.quit()

def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    cleanup()
    sys.exit(0)

# Handle Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

# Handle window close
app.aboutToQuit.connect(cleanup)

print("Displaying live data. Press Ctrl+C or close window to exit.")

# Start Qt event loop
try:
    pg.exec()
except KeyboardInterrupt:
    cleanup()
finally:
    sys.exit(0)