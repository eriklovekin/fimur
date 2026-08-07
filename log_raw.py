#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import csv
from datetime import datetime
from pathlib import Path
import time

def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        mfr = (port.manufacturer or "").lower()
        if any(name in mfr for name in ['silicon labs', 'espressif', 'cp210', 'ch340']):
            return port.device
    return ports[0].device if ports else None

BAUD = 115200
WINDOW_SIZE = 200
N_IMUS = 1
FLUSH_EVERY = 50

def update(ser,writers,files):
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line[0].isdigit():
            values = line.split(',')
            if len(values) != 2 + 6*N_IMUS:
                pass
            for i in range(N_IMUS):
                t = values[0]
                accel = values[1+6*i : 1+6*i+3]
                gyro  = values[1+6*i+3 : 1+6*i+6]
                writers[2*i].writerow([t, *accel])
                writers[2*i+1].writerow([t, *gyro])
        
    except Exception as e:
        print(e)

def main():
    count = 0

    port = find_esp32_port()
    if port is None:
        raise RuntimeError("unable to find esp32 port")    
    print(f"Connecting to {port}...")
    
    ser = serial.Serial(port, BAUD, timeout=0.01)

    path = './logs/'
    files = []
    writers  = []

    run_stamp = datetime.now().strftime("%Y%m%d-%H%M%S")

    for i in range(N_IMUS):
        for sensor in ("accel", "gyro"):
            file = Path(f"{path}{run_stamp}/{sensor}{i+1}.csv")
            file.parent.mkdir(parents=True,exist_ok=True)

            f = file.open("w", newline="")
            w = csv.writer(f)

            w.writerow(["t_us", "x", "y", "z"])
            files.append(f)
            writers.append(w)

    print("Logging data. Press Ctrl+C or close window to exit.")
    start_time = time.perf_counter()
    # Start logging
    try:
        while True:
            update(ser,writers,files)
            count += 1
            if count % FLUSH_EVERY == 0:
                for f in files:
                    f.flush()
    except KeyboardInterrupt:
        pass 
    finally:
        elapsed = time.perf_counter() - start_time

        hours, remainder = divmod(elapsed, 3600)
        minutes, seconds = divmod(remainder, 60)
        print(f"\nTotal runtime: {int(hours):02d}:{int(minutes):02d}:{seconds:06.3f}")

        print("\nCleaning up...")
        for f in files:
            f.close()
        print("log files closed")
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main()
           