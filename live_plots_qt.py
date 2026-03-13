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

accel2_x = deque(maxlen=WINDOW_SIZE)
accel2_y = deque(maxlen=WINDOW_SIZE)
accel2_z = deque(maxlen=WINDOW_SIZE)
vel2_x = deque(maxlen=WINDOW_SIZE)
vel2_y = deque(maxlen=WINDOW_SIZE)
vel2_z = deque(maxlen=WINDOW_SIZE)
pos2_x = deque(maxlen=WINDOW_SIZE)
pos2_y = deque(maxlen=WINDOW_SIZE)
pos2_z = deque(maxlen=WINDOW_SIZE)
gyro2_x = deque(maxlen=WINDOW_SIZE)
gyro2_y = deque(maxlen=WINDOW_SIZE)
gyro2_z = deque(maxlen=WINDOW_SIZE)
attitude2_x = deque(maxlen=WINDOW_SIZE)
attitude2_y = deque(maxlen=WINDOW_SIZE)
attitude2_z = deque(maxlen=WINDOW_SIZE)

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
curve_ax2 = p1.plot(pen='m', name='X2')
curve_ay2 = p1.plot(pen='y', name='Y2')
curve_az2 = p1.plot(pen='c', name='Z2')

# Create plots
p1 = win.addPlot(row=0, col=1, title="Velocity")
p1.setLabel('left', 'Velocity', units='m/s')
p1.addLegend()
curve_vx = p1.plot(pen='r', name='X')
curve_vy = p1.plot(pen='g', name='Y')
curve_vz = p1.plot(pen='b', name='Z')
curve_vx2 = p1.plot(pen='m', name='X2')
curve_vy2 = p1.plot(pen='y', name='Y2')
curve_vz2 = p1.plot(pen='c', name='Z2')

# Create plots
p1 = win.addPlot(row=0, col=2, title="Position")
p1.setLabel('left', 'Position', units='m')
p1.addLegend()
curve_rx = p1.plot(pen='r', name='X')
curve_ry = p1.plot(pen='g', name='Y')
curve_rz = p1.plot(pen='b', name='Z')
curve_rx2 = p1.plot(pen='m', name='X2')
curve_ry2 = p1.plot(pen='y', name='Y2')
curve_rz2 = p1.plot(pen='c', name='Z2')

p2 = win.addPlot(row=1, col=1, title="Gyroscope")
p2.setLabel('left', 'Angular velocity', units='deg/s')
p2.addLegend()
curve_gx = p2.plot(pen='r', name='X')
curve_gy = p2.plot(pen='g', name='Y')
curve_gz = p2.plot(pen='b', name='Z')
curve_gx2 = p2.plot(pen='m', name='X2')
curve_gy2 = p2.plot(pen='y', name='Y2')
curve_gz2 = p2.plot(pen='c', name='Z2')

p2 = win.addPlot(row=1, col=2, title="Attitude")
p2.setLabel('left', 'Angular position', units='deg')
p2.addLegend()
curve_attx = p2.plot(pen='r', name='Phi')
curve_atty = p2.plot(pen='g', name='Theta')
curve_attz = p2.plot(pen='b', name='Psi')
curve_attx2 = p2.plot(pen='m', name='Phi2')
curve_atty2 = p2.plot(pen='y', name='Theta2')
curve_attz2 = p2.plot(pen='c', name='Psi2')

def update():
    """Read serial and update plots"""
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line[0].isdigit():
            values = line.split(',')
            if len(values) == 25:
                accel_x.append(float(values[1]))
                accel_y.append(float(values[2]))
                accel_z.append(float(values[3]))
                vel_x.append(float(values[4]))
                vel_y.append(float(values[5]))
                vel_z.append(float(values[6]))
                # pos_x.append(float(values[7]))
                # pos_y.append(float(values[8]))
                # pos_z.append(float(values[9]))
                gyro_x.append(float(values[7]))
                gyro_y.append(float(values[8]))
                gyro_z.append(float(values[9]))
                attitude_x.append(float(values[10]))
                attitude_y.append(float(values[11]))
                attitude_z.append(float(values[12]))

                accel2_x.append(float(values[13]))
                accel2_y.append(float(values[14]))
                accel2_z.append(float(values[15]))
                vel2_x.append(float(values[16]))
                vel2_y.append(float(values[17]))
                vel2_z.append(float(values[18]))
                # pos2_x.append(float(values[7]))
                # pos2_y.append(float(values[8]))
                # pos2_z.append(float(values[9]))
                gyro2_x.append(float(values[19]))
                gyro2_y.append(float(values[20]))
                gyro2_z.append(float(values[21]))
                attitude2_x.append(float(values[22]))
                attitude2_y.append(float(values[23]))
                attitude2_z.append(float(values[24]))
                
                # Update curves
                curve_ax.setData(accel_x)
                curve_ay.setData(accel_y)
                curve_az.setData(accel_z)
                curve_vx.setData(vel_x)
                curve_vy.setData(vel_y)
                curve_vz.setData(vel_z)
                # curve_rx.setData(pos_x)
                # curve_ry.setData(pos_y)
                # curve_rz.setData(pos_z)
                curve_gx.setData(gyro_x)
                curve_gy.setData(gyro_y)
                curve_gz.setData(gyro_z)
                curve_attx.setData(attitude_x)
                curve_atty.setData(attitude_y)
                curve_attz.setData(attitude_z)

                curve_ax2.setData(accel2_x)
                curve_ay2.setData(accel2_y)
                curve_az2.setData(accel2_z)
                curve_vx2.setData(vel2_x)
                curve_vy2.setData(vel2_y)
                curve_vz2.setData(vel2_z)
                # curve_rx2.setData(pos2_x)
                # curve_ry2.setData(pos2_y)
                # curve_rz2.setData(pos2_z)
                curve_gx2.setData(gyro2_x)
                curve_gy2.setData(gyro2_y)
                curve_gz2.setData(gyro2_z)
                curve_attx2.setData(attitude2_x)
                curve_atty2.setData(attitude2_y)
                curve_attz2.setData(attitude2_z)
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