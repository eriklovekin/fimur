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
gyro_x = deque(maxlen=WINDOW_SIZE)
gyro_y = deque(maxlen=WINDOW_SIZE)
gyro_z = deque(maxlen=WINDOW_SIZE)

# Create application - CORRECTED
app = QtWidgets.QApplication(sys.argv)  # ← Use QtWidgets

# Create window
win = pg.GraphicsLayoutWidget(show=True, title="Live IMU Data")
win.resize(1200, 800)

# Create plots
p1 = win.addPlot(title="Accelerometer")
p1.setLabel('left', 'Acceleration', units='raw')
p1.addLegend()
curve_ax = p1.plot(pen='r', name='X')
curve_ay = p1.plot(pen='g', name='Y')
curve_az = p1.plot(pen='b', name='Z')

win.nextRow()

p2 = win.addPlot(title="Gyroscope")
p2.setLabel('left', 'Angular velocity', units='raw')
p2.addLegend()
curve_gx = p2.plot(pen='r', name='X')
curve_gy = p2.plot(pen='g', name='Y')
curve_gz = p2.plot(pen='b', name='Z')

def update():
    """Read serial and update plots"""
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line[0].isdigit():
            values = line.split(',')
            if len(values) == 7:
                accel_x.append(int(values[1]))
                accel_y.append(int(values[2]))
                accel_z.append(int(values[3]))
                gyro_x.append(int(values[4]))
                gyro_y.append(int(values[5]))
                gyro_z.append(int(values[6]))
                
                # Update curves
                curve_ax.setData(accel_x)
                curve_ay.setData(accel_y)
                curve_az.setData(accel_z)
                curve_gx.setData(gyro_x)
                curve_gy.setData(gyro_y)
                curve_gz.setData(gyro_z)
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