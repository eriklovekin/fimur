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
        # tmp0 *= r
        # tmp = np.round(tmp0)
        # if tmp != test[i]:
        #     i += 1
        #     test[i] = tmp
        # print(f"i = {i}, k = {k}, val = {tmp0}, round = {tmp}, test[i] = {test[i]}")
        # print(f"n: {n}")

    # print(f"vals: {vals}")
    j=0
    for n in vals:
        print(f"{100*j/len(vals):.2f}% complete")
        A = 1/(2*n**2*dt**2*(N-2*n))
        d = x[2*n:]-2*x[n:-n] + x[:-2*n]
        avar.append(A*np.sum(d**2))

        # s = 0
        # for i in range(0,N-2*n):
        #     s += (x[i+2*n] - 2*x[i+n] + x[i])**2
        # avar.append(A*s) 

        tau.append(n*dt) 
        j+=1
    # for i in range(len(avar)):
    #         print(f"[{tau[i]:.5f}, {avar[i]}]")
    return tau, avar

if __name__ == "__main__":
    path = "./logs/"
    d = "20260712-225526/"
    filename = "accel1.csv"

    full = path+d+filename
    print(f"loading {full}")
    df = pd.read_csv(f"{full}")
    print(f"loaded")
    # mean_dt_us = np.mean(df["t_us"][1:-1] - df["t_us"][0:-2])
    # std_dt_us = np.std(df["t_us"][1:-1] - df["t_us"][0:-2])
    x_raw = df["x"].to_numpy()
    y_raw = df["y"].to_numpy()
    z_raw = df["z"].to_numpy()
    dt = (df["t_us"][2] - df["t_us"][1])/1e6
    x = np.cumsum(x_raw) * dt
    tau_x, avar_x = compute_allan_variance(dt,x)
    adev_x = np.sqrt(avar_x)
    # tau_y, avar_y = compute_allan_variance(dt,y)
    # tau_z, avar_z = compute_allan_variance(dt,z)

    win = pg.GraphicsLayoutWidget(show=True, title="Allan Variance")
    win.resize(1200, 800)
    p1 = win.addPlot(title="Accelerometer")
    p1.setLabel('left', 'Allan Deviation &sigma;(&tau;)', units='m/s^2')
    p1.setLabel('bottom', 'window size &tau;', units='s')
    p1.addLegend()
    p1.setLogMode(x=True, y=True)
    p1.showGrid(x=True, y=True, alpha=0.3)
    cx = p1.plot(tau_x,adev_x,pen='r', name='X')
    # cy = p1.plot(pen='g', name='Y')
    # cz = p1.plot(pen='b', name='Z')
    # cy.setData(tau_y,avar_y)
    # cz.setData(tau_z,avar_z)

    pg.exec()


    # print(f"Mean dt [us]: {mean_dt_us}, std: {std_dt_us}")

