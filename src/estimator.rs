use core::ops::Sub;

use nalgebra::{UnitQuaternion, UnitVector3, Vector3};

#[derive(Clone, Copy, Debug, Default)]
pub struct USecTimestamp(pub f32);

impl USecTimestamp {
    pub fn to_seconds(self) -> f32 {
        let seconds = self.0 * 1E-6;
        if seconds.is_subnormal() {
            defmt::warn!("Subnormal timestamp conversion");
        }

        seconds
    }

    pub fn dt_seconds(from: USecTimestamp) -> f32 {
        self.
    }
}

impl Sub for USecTimestamp {
    fn sub(self, rhs: Self) -> Self::Output {
        self.0 - rhs.0
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SensorReading {
    pub accel_dir: UnitVector3<f32>,
    pub gyro_rate: Vector3<f32>,
    pub mag_field: Vector3<f32>,
}

impl SensorReading {
    pub fn new(
        accel_dir: UnitVector3<f32>,
        gyro_rate: Vector3<f32>,
        mag_field: Vector3<f32>,
    ) -> Self {
        Self {
            accel_dir,
            gyro_rate,
            mag_field,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TimestampedReading {
    pub time: USecTimestamp,
    pub reading: SensorReading,
}

impl TimestampedReading {
    pub fn new(time: USecTimestamp, reading: SensorReading) -> Self {
        Self { time, reading }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TimestampedAttitude {
    pub time: USecTimestamp,
    pub attitude: UnitQuaternion<f32>,
}

impl TimestampedAttitude {
    pub fn new(time: USecTimestamp, attitude: UnitQuaternion<f32>) -> Self {
        Self { time, attitude }
    }
}

trait Estimator {
    fn step(&mut self, time: USecTimestamp);
    fn correct(&mut self, reading: TimestampedReading);
    fn get_attitude(&self) -> TimestampedAttitude;
}
