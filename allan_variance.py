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
    timestamp = "20260812-234507/"
    # timestamp = "20260810-231852/"
    # timestamp = "20260809-113733/"
    # timestamp = "20260808-235938/"
    # timestamp = "20260725-233150/"
    # timestamp = "20260807-052335/"
    sensor = ["accel", "gyro"]
    units =  ["m/s2", "rad/s"]
    axis = ["x","y","z"]
    color = ["r","g","b"]
    N_IMUS = 11

    index = pd.MultiIndex.from_product(
    [range(1, N_IMUS+1), ["accel", "gyro"], ["x", "y", "z"]],
    names=["imu", "sensor", "axis"]
    )
    superDf = pd.DataFrame(index=index, columns=["tau", "B"], dtype=float)

    # conversion from min allan deviation to bias instability
    C = 1/np.sqrt(2*np.log(2)/np.pi) # per IEEE-STD-952-1997 Annex C 

    dt_diag = []

    app = QtWidgets.QApplication(sys.argv)
    windows = []

    for i in range(0,N_IMUS):
        for j in range(len(sensor)):
            full = path+timestamp+sensor[j]+str(i+1)+".csv"
            print(f"loading {full}")
            df = pd.read_csv(f"{full}")

            z_numeric = pd.to_numeric(df["z"], errors="coerce")
            bad_mask = z_numeric.isna() & df["z"].notna()  # failed to parse, but wasn't already NaN/empty
            print(df.loc[bad_mask, "z"])
            
            nan_counts = df.isna().sum()
            if nan_counts.any():
                print(df.isna().sum())  # NaN count per column
                nan_rows = df[df.isna().any(axis=1)]
                print(nan_rows.head(20))
                print(f"first NaN at row: {nan_rows.index.min() if len(nan_rows) else 'none'}")

            # dt = (df["t_us"][2] - df["t_us"][1])/1e6
            dt = (df["t_us"].iloc[-1] - df["t_us"].iloc[0]) / (1e6 * (df["t_us"].size - 1))            
            dt_diag = np.diff(df["t_us"])
            mean_dt = np.mean(dt_diag*1e-6)
            std_dt = np.std(dt_diag)
              
            win = pg.GraphicsLayoutWidget(show=True, title=f"Allan Variance - {timestamp}{sensor[j]}{i+1}")
            win.resize(1200, 800)
            p = win.addPlot()
            p.setTitle(f"{timestamp}{sensor[j]}{i+1}")
            p.setLabel('left', 'Allan Deviation &sigma;(&tau;)', units=f"{units[j]}")
            p.setLabel('bottom', 'window size &tau;', units='s')
            ax = p.getAxis('left')
            ax.enableAutoSIPrefix(False)
            ax = p.getAxis('bottom')
            ax.enableAutoSIPrefix(False)
            p.addLegend(offset = (-10,10))
            p.setLogMode(x=True, y=True)
            p.showGrid(x=True, y=True)
            
            for a in range(len(axis)):
                raw = df[axis[a]].to_numpy()

                tau, adev, adev_err, n = allantools.oadev(
                    raw, rate=1/mean_dt, data_type="freq", taus="octave"
                )

                # print(f"{axis[a]}-axis...")
                # craw = np.cumsum(raw) * dt
                # tau, avar = compute_allan_variance(dt,craw)
                # adev = np.sqrt(avar)
                # print(f"{type(adev)}, {np.shape(adev)}")
                min_adev = np.min(adev)
                idx = np.argmin(np.abs(adev - min_adev))
                t = tau[idx]
                B = C*min_adev
                superDf.loc[(i+1,f"{sensor[j]}",f"{axis[a]}"),"tau"] = t
                superDf.loc[(i+1,f"{sensor[j]}",f"{axis[a]}"),"B"] = B

                print(f"{axis[a]}:\nmin adev [{units[j]}]: {min_adev:.4f}, " \
                      f"bias instability [{units[j]}]: {B:.4f}, " \
                      f"time constant: {t:.4f}")

                p.plot(tau,adev,pen=f"{color[a]}", name=f"{axis[a]}")
                # err_item = pg.ErrorBarItem(
                #     x=np.log10(tau_x),
                #     y=np.log10(adev_x),
                #     top=np.log10(adev_x + adev_err_x) - np.log10(adev_x),
                #     bottom=np.log10(adev_x) - np.log10(adev_x - adev_err_x),
                #     pen=pg.mkPen('r', width=1)
                # )
                # p.addItem(err_item)

            win.show()
            windows.append(win)

            for _ in range(10):
                app.processEvents()

            expath = f"./assets/img/allan/{timestamp}"
            exname = f"{expath}allan_{sensor[j]}{i+1}.png"

            # os.makedirs(expath, exist_ok=True)
            # exporter = pg.exporters.ImageExporter(p)
            # exporter.parameters()['width'] = 1920
            # exporter.parameters()['background'] = 'w'
            # print(f"writing to {exname}")
            # exporter.export(f"{exname}")

    w = pg.GraphicsLayoutWidget(show=True, title=f"Loop Time histogram - {timestamp}{sensor[j]}{i+1}")
    w.resize(1200, 800)
    p1 = w.addPlot()
    p1.setTitle(f"Loop Time histogram - mean: {mean_dt}, std: {std_dt}")
    # p1.setLabel('left', '', units="s")
    p1.setLabel('bottom', 'Loop time', units="us")
    y, x = np.histogram(dt_diag, bins=30)
    p1.plot(x, y, stepMode="center", fillLevel=0, brush=(0, 0, 255, 100))

    print("Stack stats:")
    for j in range(len(sensor)):
        print(f"{sensor[j]}")
        for a in range(len(axis)):
            set = superDf.xs((sensor[j],axis[a]), level=("sensor","axis"))
            meanB = set["B"].mean()
            medB = set["B"].median()
            stdB = set["B"].std()
            meanTau = set["tau"].mean()
            medTau = set["tau"].median()
            stdTau = set["tau"].std()
            
            print(f"{axis[a]}")
            print(f"bias instability mean [{units[j]}]: {meanB:.4f}, median: {medB:.4f}, std: {stdB:.4f}")
            print(f"correlation time mean [{units[j]}]: {meanTau:.4f}, median: {medTau:.4f}, std: {stdTau:.4f}")
        print("\n")

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

