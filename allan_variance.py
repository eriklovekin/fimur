import pandas as pd
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph.exporters
import sys
import os
import signal
import allantools

def compute_allan_variance(dt,x):
    N = len(x)
    avar = []
    tau = []
    vals = []
    n = 1
    r = 1.1
    while n < N//2:
        vals.append(n)
        n_next = n*r
        n = int(np.round(max(n_next, n+1)))

    j=0
    for n in vals:
        print(f"{100*j/len(vals):.2f}% complete")
        A = 1/(2*n**2*dt**2*(N-2*n))
        d = x[2*n:]-2*x[n:-n] + x[:-2*n]
        avar.append(A*np.sum(d**2))

        tau.append(n*dt) 
        j+=1
    return tau, avar

if __name__ == "__main__":
    path = "./logs/"
    timestamp = "20260725-233150/"
    sensor = ["accel", "gyro"]
    units =  ["g", "deg/s"]
    N_IMUS = 12

    app = QtWidgets.QApplication(sys.argv)
    windows = []

    for i in range(0,N_IMUS):
        for j in range(len(sensor)):
            full = path+timestamp+sensor[j]+str(i+1)+".csv"
            print(f"loading {full}")
            df = pd.read_csv(f"{full}")
            nan_counts = df.isna().sum()
            if nan_counts.any():
                print(df.isna().sum())  # NaN count per column
                nan_rows = df[df.isna().any(axis=1)]
                print(nan_rows.head(20))
                print(f"first NaN at row: {nan_rows.index.min() if len(nan_rows) else 'none'}")
            x_raw = df["x"].to_numpy()
            y_raw = df["y"].to_numpy()
            z_raw = df["z"].to_numpy()
            dt = (df["t_us"][2] - df["t_us"][1])/1e6

            tau_x, adev_x, adev_err_x, n = allantools.oadev(
                x_raw, rate=1/dt, data_type="freq", taus="octave"
            )
            tau_y, adev_y, adev_err_y, n = allantools.oadev(
                y_raw, rate=1/dt, data_type="freq", taus="octave"
            )
            tau_z, adev_z, adev_err_z, n = allantools.oadev(
                z_raw, rate=1/dt, data_type="freq", taus="octave"
            )

            # x = np.cumsum(x_raw) * dt
            # y = np.cumsum(y_raw) * dt
            # z = np.cumsum(z_raw) * dt
            # print("x-axis...")
            # tau_x, avar_x = compute_allan_variance(dt,x)
            # print("y-axis...")
            # tau_y, avar_y = compute_allan_variance(dt,y)
            # print("z-axis...")
            # tau_z, avar_z = compute_allan_variance(dt,z)
            # adev_x = np.sqrt(avar_x)
            # adev_y = np.sqrt(avar_y)
            # adev_z = np.sqrt(avar_z)

            win = pg.GraphicsLayoutWidget(show=True, title=f"Allan Variance - {timestamp}{sensor[j]}{i+1}")
            win.resize(1200, 800)

            p = win.addPlot()
            p.setTitle(f"{timestamp}{sensor[j]}{i+1}")
            p.setLabel('left', 'Allan Deviation &sigma;(&tau;)', units=f"{units[j]}")
            p.setLabel('bottom', 'window size &tau;', units='s')
            p.addLegend(offset = (-10,10))
            p.setLogMode(x=True, y=True)
            p.showGrid(x=True, y=True)
            p.plot(tau_x,adev_x,pen='r', name='X')
            p.plot(tau_y,adev_y,pen='g', name='Y')
            p.plot(tau_z,adev_z,pen='b', name='Z')
            win.show()
            windows.append(win)

            for _ in range(10):
                app.processEvents()

            expath = f"./assets/img/allan/{timestamp}"
            exname = f"{expath}allan_{sensor[j]}{i+1}.png"

            os.makedirs(expath, exist_ok=True)
            exporter = pg.exporters.ImageExporter(p)
            exporter.parameters()['width'] = 1920
            exporter.parameters()['background'] = 'w'
            print(f"writing to {exname}")
            exporter.export(f"{exname}")
            

    print("plotting...")
    # Let Ctrl+C interrupt the Qt event loop
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)  # no-op, just wakes the interpreter
    timer.start(200)  # ms

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        print("Ctrl+C received, closing windows...")
        for w in windows:
            w.close()
        app.quit()

    # print(f"Mean dt [us]: {mean_dt_us}, std: {std_dt_us}")

