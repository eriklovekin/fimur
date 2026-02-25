#[derive(Debug)]
pub enum ImuError<I2cError> {
    I2c(I2cError),
    InvalidSetAccelerometerScale,
    FailedGetAccelerometerScale,
    InvalidSetGyroscopeScale,
    FailedGetGyroscopeScale,
}

impl<E> From<E> for ImuError<E> {
    fn from(error: E) -> Self {
        ImuError::I2c(error)
    }
}