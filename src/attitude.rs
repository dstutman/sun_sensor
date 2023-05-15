use core::f32::consts::PI;

use libm::{acosf, atan2f};
use nalgebra::{SMatrix, SVector};

#[derive(Clone, Copy, Debug, defmt::Format, serde::Serialize)]
pub struct Attitude {
    pub azimuth: f32,
    pub elevation: f32,
}

impl Attitude {
    pub fn from_illuminations(
        face_definition: SMatrix<f32, 3, 3>,
        mut face_illuminations: SVector<f32, 3>,
    ) -> Self {
        face_illuminations.normalize_mut();
        let illumination_vector = face_definition * face_illuminations;

        let elevation =
            PI / 2.0 - acosf(illumination_vector.dot(&SVector::<f32, 3>::new(0.0, 0.0, 1.0)));

        let azimuth = atan2f(
            illumination_vector.dot(&SVector::<f32, 3>::new(0.0, 1.0, 0.0)),
            illumination_vector.dot(&SVector::<f32, 3>::new(1.0, 0.0, 0.0)),
        );

        Self { azimuth, elevation }
    }
}
