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

AXIS_HUES = {'x': 0, 'y': 120, 'z': 240}  # red, green, blue in HSV degrees

def axis_color(axis_letter, imu_index, n_imus):
    hue = AXIS_HUES[axis_letter]
    # vary value/brightness across IMUs so overlapping traces stay distinguishable
    value = 255 - int(150 * imu_index / max(n_imus - 1, 1))
    return pg.hsvColor(hue / 360.0, sat=1.0, val=value / 255.0)

def update():
    """Read serial and update plots"""
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line[0].isdigit():
            parts = [p for p in line.split(',') if p != '']
            if len(parts) != 6 * N_IMUS+1:
                return  # skip malformed line, wait for next one
            values = [float(v) for v in parts]         
            for i in range(N_IMUS):
                for k, ch in enumerate(CHANNELS):
                    data[i][ch].append(values[6*i + k+1])
            
                    # Update curves
                    curve[i][ch].setData(list(data[i][ch]))
    except Exception as e:
        print(f"update error: {e}")
        pass

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

if __name__ == "__main__":
    PORT = find_esp32_port()
    BAUD = 115200
    WINDOW_SIZE = 200
    N_IMUS = 12
    CHANNELS = ["ax", "ay", "az", "gx", "gy", "gz"]
    COLORS = ["r", "g", "b", "r", "g", "b"]

    print(f"Connecting to {PORT}...")
    ser = serial.Serial(PORT, BAUD, timeout=0.01)

    # Data buffers
    data = {
        i: {ch: deque(maxlen=WINDOW_SIZE) for ch in CHANNELS}
        for i in range(N_IMUS)
    }

    # Create application
    app = QtWidgets.QApplication(sys.argv)

    # Create window
    win = pg.GraphicsLayoutWidget(show=True, title="Live IMU Data")
    # win = QtWidgets.QWidget()
    # layout = QtWidgets.QGridLayout(win)
    # layout.addWidget(Qt)
    win.resize(1200, 800)

    # Create plots
    paccel = win.addPlot(row=0, col=0, title="Accelerometer")
    paccel.setLabel('left', 'Acceleration', units='m/s^2')
    paccel.addLegend()
    # Create plots
    pgyro = win.addPlot(row=1, col=0, title="Gyroscope")
    pgyro.setLabel('left', 'Rate', units='rad/s')
    pgyro.addLegend()

    curve = {}
    for i in range(12):
    # for i in range(N_IMUS):
        curve[i] = {}
        c = 0
        for ch in CHANNELS[0:3]:
            axis_letter=ch[-1]
            curve[i][ch] = paccel.plot(pen=pg.mkPen(axis_color(axis_letter, i, N_IMUS), width=1))
        for ch in CHANNELS[3:6]:
            axis_letter=ch[-1]
            curve[i][ch] = pgyro.plot(pen=pg.mkPen(axis_color(axis_letter, i, N_IMUS), width=1))

    # Set up timer for updates
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(10)  # Update every 10ms

    # # Start Qt event loop
    # sys.exit(app.exec_())

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