import pandas as pd
import numpy as np
import pyqtgraph as pg


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
    timestamp = "20260712-225526/"
    filename = "gyro2.csv"

    full = path+timestamp+filename
    print(f"loading {full}")
    df = pd.read_csv(f"{full}")
    print(f"loaded")
    x_raw = df["x"].to_numpy()
    y_raw = df["y"].to_numpy()
    z_raw = df["z"].to_numpy()
    dt = (df["t_us"][2] - df["t_us"][1])/1e6
    x = np.cumsum(x_raw) * dt
    y = np.cumsum(y_raw) * dt
    z = np.cumsum(z_raw) * dt
    print("x-axis...")
    tau_x, avar_x = compute_allan_variance(dt,x)
    print("y-axis...")
    tau_y, avar_y = compute_allan_variance(dt,y)
    print("z-axis...")
    tau_z, avar_z = compute_allan_variance(dt,z)
    adev_x = np.sqrt(avar_x)
    adev_y = np.sqrt(avar_y)
    adev_z = np.sqrt(avar_z)

    print("plotting...")
    win = pg.GraphicsLayoutWidget(show=True, title="Allan Variance")
    win.resize(1200, 800)
    p1 = win.addPlot()
    p1.setTitle(f"{timestamp}{filename}")
    p1.setLabel('left', 'Allan Deviation &sigma;(&tau;)', units='deg/s')
    p1.setLabel('bottom', 'window size &tau;', units='s')
    p1.addLegend(offset = (-10,10))
    p1.setLogMode(x=True, y=True)
    p1.showGrid(x=True, y=True)
    p1.plot(tau_x,adev_x,pen='r', name='X')
    p1.plot(tau_y,adev_y,pen='g', name='Y')
    p1.plot(tau_z,adev_z,pen='b', name='Z')


    pg.exec()


    # print(f"Mean dt [us]: {mean_dt_us}, std: {std_dt_us}")

