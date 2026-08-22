# Fused Inertial Measurement Unit aRray (FIMUR)
Can the performance of high-cost IMUs be matched through the fusion of a large array of MEMS IMUs?

## Build
`cargo build` will fail when run from the root of the repo because there are two compilation targets
This project uses a [justfile](https://just.systems/man/en/) to automate build commands.
- To build all firmware crates (targetting ESP32), run `just build`
- To build the fusion-py crate, run `just pybuild`
- To build everything, run `just all`
- To clean all build artifacts, run `just clean`

## Deploy
To deploy to the ESP32 target, run `just run`

## Hardware
This project targets an [ESP32-C6-DevKitC-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html), connected via the onboard USB-C ports. For best performance, connect to the ESP32-C6 native USB port (The left port looking at the board from the top with the USB ports pointing up).

## Coordinate Frames
### Sensor (S)
The S frame is the frame in which the sensor outputs measurements.

### Filter (F)
The F frame is the frame in which the output of the filter is represented. The origin and orientation of each IMU must be initialized in this frame. 
    Origin: position of the S origin of the IMU as (x_F, y_F, z_F)
    Rotation: Direction cosine matrix from IMU S frame to F frame

### Wake (W)
The W frame is an inertial frame colocated with the F frame at filter startup (wake). It does not move with the sensor, and is the frame in which position estimates are represented.

### ENU (L)
The ENU frame is an inertial frame with origin colocated with the Filter frame origin.

## Filters

### 1: Choi-Dinkel
The fusion-core crate implements the algorithm described in [Ref 8](https://hollydinkel.github.io/assets/pdf/AAS2025.pdf) with some modifications. The filter is essentially an equally weighted average of measurements from all IMUs rotated into the virtual (Filter) frame. There is some fancy math that projects out the $\alpha\times r$ acceleration term by multiplying by its nullspace.

The filter assumes:
- IMUs rigidly connected to eachother
- *i*th accelerometer and gyroscope are colocated
- IMU relative poses known apriori
    - Different from the referenced algorithm which estimates relative poses as part of an initial extrinsic calibration step

This filter is implemented in the fusion-core crate and is accessible for use both in the online Rust environment and an offline python environment via an interface defined in the fusion-py crate (see [offline_fusion.py](./offline_fusion.py)). 
Note: for use in either environment, `N_IMUS` as defined in fusion-core must agree with the number of IMUs the algorithm is being run on.

## Configuration
IMU configurations are defined individually in TOML files in the `sensor-config/` directory. These are parsed by both the online (TODO) and offline fusion processes. The configs include:
- Adressing
- Sensor settings
- Pose

## Dependencies

### Python
Running with python 3.12.3
Ensure the following libraries are installed using `pip install <library>`
- serial
- pyserial
- pandas
- numpy
- pyqtgraph
- pyside6
- allantools
- PyQt5
- tqdm

### PyQt
- `sudo apt update`
- `sudo apt install libxcb-cursor0`

### Just
-  `sudo apt install just`

## References
1 [Interactive Kalman Filter Tutorial](https://arthurlovekin.com/interactive-kalman-filter/index.html)

2 [Extended Kalman Filter](https://mwrona.com/posts/attitude-ekf/)

3 Sarabandi, S; Thomas, F - Accurate Computation of Quaternions from Rotation Matrices

4 [Rotation Quaternions, and How to Use Them](https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html)

5 [AHRS Complimenatary Filter](https://ahrs.readthedocs.io/en/latest/filters/complementary.html)

6 [Building a Virtual Gyro](https://www.nxp.com/company/about-nxp/smarter-world-blog/BL-BUILDING-VIRTUAL-GYRO)

7 [Gauss Markov Theorem](https://www.statlect.com/fundamentals-of-statistics/Gauss-Markov-theorem)

8 [Sensor Fusion for Distributed Inertial Measurement Units](https://hollydinkel.github.io/assets/pdf/AAS2025.pdf)

9 [Understanding ARW: The Hidden Limit to IMU Accuracy (Part 1)](https://guidenav.com/blog/understanding-arw-the-hidden-limit-to-imu-accuracy-part-1/)