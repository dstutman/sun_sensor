// Standard data reporting format

use nalgebra::UnitQuaternion;

use serde;

use crate::{attitude::Attitude, faults::ArrayStatus, DataStatus};

#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct Report {
    pub data_status: DataStatus,
    pub ldr_attitude: Attitude,
    pub imu_attitude: UnitQuaternion<f32>,
    pub faults: ArrayStatus,
    pub timestamp: u32,
}

impl Report {
    pub fn new(
        data_status: DataStatus,
        ldr_attitude: Attitude,
        imu_attitude: UnitQuaternion<f32>,
        faults: ArrayStatus,
        timestamp: u32,
    ) -> Self {
        Self {
            data_status,
            ldr_attitude,
            imu_attitude,
            faults,
            timestamp,
        }
    }
}
