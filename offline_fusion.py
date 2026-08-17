# Ingest logs of raw data and run fusion algorithms on them

import numpy as np
import pandas as pd
import tomllib
from pathlib import Path
import csv
from tqdm import tqdm

import fusion_py as fp

CHUNK_SIZE = 500

def get_sensor_config(config_path):
    with open(config_path, "rb") as f:
        c = tomllib.load(f)
    return c

def get_sensor_poses(config_array):
    n = np.zeros((3*len(config_array),3))
    p = np.zeros((3*len(config_array),1))
    
    for i in range(len(config_array)):
        conf = get_sensor_config(config_array[i])
        n[3*i:3*i+3,0:3] = conf["pose"]["s2f"]
        p[3*i:3*i+3,0]   = conf["pose"]["origin_f"]
    return n,p

def assemble_measurements(row_set, n_imu):
    accel = np.zeros((3 * n_imu,1))
    gyro = np.zeros((3 * n_imu,1))

    for i in range(n_imu):
        a_row = row_set[f"accel{i+1}"]
        g_row = row_set[f"gyro{i+1}"]

        accel[3*i : 3*i+3, 0] = a_row[1:4]  # x, y, z (skip timestamp at index 0)
        gyro[3*i : 3*i+3, 0]  = g_row[1:4]
    ts = a_row[0]

    return ts, accel, gyro

def next_row_set(readers):
    while True:
        raw_rows = {}
        exhausted = False

        #pull the next raw row from every file, in lockstep
        for key, r in readers.items():
            try:
                raw_rows[key] = next(r)
            except StopIteration:
                print(f"\nran out of data in {key}")
                exhausted = True
                break

        if exhausted:
            return None  # at least one file is out of rows

        #try to parse every row; if any fails, discard whole set
        parsed_rows = {}
        valid = True
        for key, row in raw_rows.items():
            if len(row) != 4:
                print(f"malformed line in {key}: wrong field count ({len(row)}): {row}")
                valid=False
                continue
            try:
                parsed_rows[key] = [float(v) for v in row]
            except ValueError:
                print(f"malformed line in {key}: {row} — skipping this row across all files")
                valid=False

        if not valid:
            continue  # skip this row set across all files, try the next one

        #validate that timestamps agree across all files
        timestamps = {key: row[0] for key, row in parsed_rows.items()}
        ref_key, ref_ts = next(iter(timestamps.items()))
        mismatches = {k: ts for k, ts in timestamps.items() if ts != ref_ts}

        if mismatches:
            print(f"timestamp mismatch: {ref_key}={ref_ts}, mismatches={mismatches}")
            continue  # skip this row set, try the next one

        return parsed_rows

if __name__ == "__main__":
    log_path = "./logs/"
    # timestamp = "test2/"
    # timestamp = "20260814-002623/"
    timestamp = "20260812-234507/" # clean data, ~7hours log
    # timestamp = "20260810-231852/"
    # timestamp = "20260809-113733/"
    # timestamp = "20260808-235938/"
    # timestamp = "20260725-233150/"
    # timestamp = "20260807-052335/"
    sensor = ["accel", "gyro"]
    config_path = "./sensor-config/"
    config_name = "imu"

    # which sensors to use in fusion
    sensor_idx = ["1","2","3","4","5",
                  "6","7","8","9","10"]

    # get relative poses of all sensors from their config
    sensor_name = []
    for i in range(len(sensor_idx)):
        sensor_name.append(config_path+config_name+sensor_idx[i]+".toml")

    n,p = get_sensor_poses(sensor_name)
    # print(f"n: \n{n}")
    # print(f"p: \n{p}")

    # Set geometric constants for filter
    f = fp.PyFusionCore(n,p)

    with open(f"{log_path}{timestamp}{sensor[0]}{sensor_idx[0]}.csv", "r") as sf:
        row_count = sum(1 for _ in sf) - 1  # -1 for header

    # Construct virtual data log files in same format as live logs
    wfiles = {}
    writers  = {}
    for s in sensor:
        wfile = Path(f"{log_path}{timestamp}/virtual/{s}1.csv")
        wfile.parent.mkdir(parents=True,exist_ok=True)

        wf = wfile.open("w", newline="")
        w = csv.writer(wf)

        w.writerow(["t_us", "x", "y", "z"])
        wfiles[s] = wf
        writers[s] = w

    # construct library of the live logs to read from
    rfiles = {}
    readers = {}
    for s in sensor:
        for i in range(len(sensor_idx)):
            key = f"{s}{sensor_idx[i]}"
            rf = open(f"{log_path}{timestamp}{key}.csv", "r")
            rfiles[key] = rf
            r = csv.reader(rf)
            next(r)  # skip header row
            readers[key] = r

    # read a single row at a time per sensor
    row_set = next_row_set(readers)

    with tqdm(total=row_count, desc="Processing rows") as pbar:
        while row_set is not None:
            # assemble a and w vector from individual sensor measurements        
            t,a,w = assemble_measurements(row_set,len(sensor_idx))
            # print(f"t: {t}")
            # print(f"a: \n{a}")
            # print(f"w: \n{w}")
            av, wv = f.fuse(a,w)

            # write to new logs
            # writers["accel"].writerow([t,*a.flatten()])
            # writers["gyro"].writerow([t,*w.flatten()])
            writers["accel"].writerow([t,*av.flatten()])
            writers["gyro"].writerow([t,*wv.flatten()])
            row_set = next_row_set(readers)
            pbar.update(1)

    # clean up file handles once done
    for f in rfiles.values():
        f.close()
    for f in wfiles.values():
        f.close()
        