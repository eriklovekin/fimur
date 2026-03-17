# Fused Inertial Measurement Unit aRray (FIMUR)
Can the performance of high-cost IMUs be matched through the fusion of a large array of MEMS IMUs?

## Coordinate Frames
### Sensor (S)
The S frame is the frame in which the sensor outputs measurements.

### Filter (F)
The F frame is the frame in which the output of the filter is represented. The origin and orientation of each IMU must be initialized in this frame. 
    Origin: position of the S origin of the IMU as (x_F, y_F, z_F)
    Rotation: Direction cosine matrix from IMU S frame to F frame

## References
[Interactive Kalman Filter Tutorial](https://arthurlovekin.com/interactive-kalman-filter/index.html)