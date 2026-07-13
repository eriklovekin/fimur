import pandas as pd
import numpy as np

def compute_allan_variance(time,samples):
    pass

if __name__ == "__main__":
    path = "./logs/"
    d = "20260712-221902/"
    filename = "accel1.csv"

    df = pd.read_csv(f"{path}{d}{filename}")
    mean_dt_us = np.mean(df["t_us"][1:-1] - df["t_us"][0:-2])
    std_dt_us = np.std(df["t_us"][1:-1] - df["t_us"][0:-2])
    print(df["t_us"].dtype)
    print(df["t_us"].head())    
    print(f"Mean dt [us]: {mean_dt_us}, std: {std_dt_us}")

