import pandas as pd
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph.exporters
import sys
import os
import signal
import allantools

pg.setConfigOption('foreground', 'k')
pg.setConfigOption('background', 'w')

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

bad_lines_log = []

def handle_bad_line(line):
    bad_lines_log.append(line)
    return None

def init_allan_variance_plot(title,units,legend):
    w = pg.GraphicsLayoutWidget(show=True, title=f"{title}")
    w.resize(1200, 800)
    p = w.addPlot()
    p.setTitle(f"{title}")
    p.setLabel('left', 'Allan Deviation &sigma;(&tau;)', units=f"{units}")
    p.setLabel('bottom', 'window size &tau;', units='s')
    a = p.getAxis('left')
    a.enableAutoSIPrefix(False)
    a = p.getAxis('bottom')
    a.enableAutoSIPrefix(False)
    p.setLogMode(x=True, y=True)
    p.showGrid(x=True, y=True)
    if legend:
        p.addLegend(offset = (-10,10),labelTextSize='18pt')
    return w, p, a

def save_plot(path,name,plot):
    os.makedirs(path, exist_ok=True)
    exporter = pg.exporters.ImageExporter(plot)
    exporter.parameters()['width'] = 1920
    exporter.parameters()['background'] = 'w'
    print(f"writing to {path}{name}")
    exporter.export(f"{path}{name}")

if __name__ == "__main__":
    save_plots=True

    path = "./logs/"
    # timestamp = "20260814-002623/"
    timestamp = "20260812-234507/virtual/"
    # timestamp = "20260812-234507/"
    # timestamp = "20260810-231852/"
    # timestamp = "20260809-113733/"
    # timestamp = "20260808-235938/"
    # timestamp = "20260725-233150/"
    # timestamp = "20260807-052335/"
    sensor = ["accel", "gyro"]
    units =  ["m/s2", "rad/s"]
    axis = ["x","y","z"]
    color = ["r","g","b"]
    color2 = [0.0,0.3,0.6]
    # which sensors to plot
    # For raw data, this specifies the index of the sensor
    # for virtual data, this specifies the number of sensors fused
    sensor_idx = ["1","2","4","6","8","10"]

    index = pd.MultiIndex.from_product(
    [range(1, len(sensor_idx)), ["accel", "gyro"], ["x", "y", "z"]],
    names=["imu", "sensor", "axis"]
    )
    superDf = pd.DataFrame(index=index, columns=["tau", "B"], dtype=float)

    # conversion from min allan deviation to bias instability
    C = 1/np.sqrt(2*np.log(2)/np.pi) # per IEEE-STD-952-1997 Annex C 

    dt_diag = []

    app = QtWidgets.QApplication(sys.argv)
    windows = []

    combined_windows = [] # windows for combined sensors for all data
    combined_plots = [] # plots for combined sensors for all data

    wax = []# one window for each axis of each sensor type
    pax = []

    win0a, p0a, _ = init_allan_variance_plot(f"Allan Variance - {timestamp} Accelerometer",units[0],True)
    combined_windows.append(win0a)
    combined_plots.append(p0a)
    windows.append(win0a)

    win0g, p0g, _ = init_allan_variance_plot(f"Allan Variance - {timestamp} Gyroscope",units[1],True)
    combined_windows.append(win0g)
    combined_plots.append(p0g)
    windows.append(win0g)

    for i in range(len(sensor_idx)):
        brightness = 0.4 + 0.6 * (i / max(len(sensor_idx) - 1, 1))

        for j in range(len(sensor)):
            full = path+timestamp+sensor[j]+sensor_idx[i]+".csv"
            print(f"loading {full}")
            # Load data while collecting the bad rows
            df = pd.read_csv(f"{full}", on_bad_lines=handle_bad_line, engine='python')

            print("Dropped lines:", bad_lines_log)

            z_numeric = pd.to_numeric(df["z"], errors="coerce")
            bad_mask = z_numeric.isna() & df["z"].notna()  # failed to parse, but wasn't already NaN/empty
            print(df.loc[bad_mask, "z"])
            
            nan_counts = df.isna().sum()
            if nan_counts.any():
                print(df.isna().sum())  # NaN count per column
                nan_rows = df[df.isna().any(axis=1)]
                print(nan_rows.head(20))
                print(f"first NaN at row: {nan_rows.index.min() if len(nan_rows) else 'none'}")

            print("skipping rows containing NaN")
            df = df.dropna()

            dt_diag = np.diff(df["t_us"])
            mean_dt_s = np.mean(dt_diag*1e-6)
            std_dt_s = np.std(dt_diag*1e-6)

            win, p, _ = init_allan_variance_plot(f"Allan Variance - {timestamp}{sensor[j]}{sensor_idx[i]}",units[j],True)
            
            for a in range(len(axis)):
                #create per-axis combined plots
                if i == 0:
                    winaxis, paxis, _ = init_allan_variance_plot(f"Allan Variance - {timestamp} {sensor[j]} {axis[a]}",units[j],True)
                    wax.append(winaxis)
                    pax.append(paxis)
                    windows.append(winaxis)

                raw = df[axis[a]].to_numpy()

                tau, adev, adev_err, n = allantools.oadev(
                    raw, rate=1/mean_dt_s, data_type="freq", taus="octave"
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
                superDf.loc[(f"{sensor_idx[i]}",f"{sensor[j]}",f"{axis[a]}"),"tau"] = t
                superDf.loc[(f"{sensor_idx[i]}",f"{sensor[j]}",f"{axis[a]}"),"B"] = B

                print(f"{axis[a]}:\nmin adev [{units[j]}]: {min_adev:.4f}, " \
                      f"bias instability [{units[j]}]: {B:.4f}, " \
                      f"time constant: {t:.4f}")

                p.plot(tau,adev,pen=pg.mkPen(pg.hsvColor(hue=color2[a]),width=3),sat=1.0, name=f"{axis[a]}")
                pax[a+3*j].plot(tau,adev,pen=pg.mkPen(pg.hsvColor(hue=color2[a], sat=1.0, val=brightness),width=3), name=f"{sensor_idx[i]}")
                # combined_plots[j].plot(tau,adev,pen=f"{color[a]}", name=f"{axis[a]}")
                combined_plots[j].plot(tau,adev,pen=f"{color[a]}", name=f"{sensor_idx[i]}")
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

            if save_plots:
                expath = f"./assets/img/allan/{timestamp}"
                exname = f"allan_{sensor[j]}{sensor_idx[i]}.png"
                save_plot(expath,exname,p)

    w = pg.GraphicsLayoutWidget(show=True, title=f"Loop Time histogram - {timestamp}{sensor[j]}{sensor_idx[i]}")
    w.resize(1200, 800)
    p1 = w.addPlot()
    p1.setTitle(f"Loop Time histogram - mean: {mean_dt_s*1e3}ms, std: {std_dt_s*1e3}ms")
    # p1.setLabel('left', '', units="s")
    p1.setLabel('bottom', 'Loop time', units="us")
    y, x = np.histogram(dt_diag, bins=50)
    p1.plot(x, y, stepMode="center", fillLevel=0, brush=(0, 0, 255, 100))
    if save_plots:
        expath = f"./assets/img/allan/{timestamp}"
        exname = f"dt_histogram_{sensor[j]}{sensor_idx[i]}.png"
        save_plot(expath,exname,p1)

    for w in combined_windows:
        w.show()
        w.raise_()

    for w in wax:
        w.show()
        w.raise_()
    for j in range(len(sensor)):
        path = f"./assets/img/allan/{timestamp}"
        name = f"Allan Variance {sensor[j]} all.png"   
        save_plot(path,name,combined_plots[j])
        
        for a in range(len(axis)):
            path = f"./assets/img/allan/{timestamp}"
            name = f"Allan Variance {sensor[j]} {axis[a]}.png"   
            save_plot(path,name,pax[a+3*j])

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