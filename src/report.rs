// Standard data reporting format

use nalgebra::UnitQuaternion;

use serde;

use crate::{attitude::Attitude, faults::ArrayStatus};

#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct Report {
    pub ldr_attitude: Attitude,
    pub imu_attitude: UnitQuaternion<f32>,
    pub faults: ArrayStatus,
    pub timestamp: u32,
}

impl Report {
    pub fn new(
        ldr_attitude: Attitude,
        imu_attitude: UnitQuaternion<f32>,
        faults: ArrayStatus,
        timestamp: u32,
    ) -> Self {
        Self {
            ldr_attitude,
            imu_attitude,
            faults,
            timestamp,
        }
    }
}
