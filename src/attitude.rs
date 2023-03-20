use core::f32::consts::PI;

use libm::{atan2f, atanf, sqrtf};
use nalgebra::{SMatrix, Vector2};

use crate::inverse_embedding::{resistive_divider_inverse_embedding, InverseEmbeddingTable};

#[derive(Clone, Copy)]
pub struct CurrentEstimate {
    pub azimuth: f32,
    pub elevation: f32,
}

pub struct AttitudePipeline<'a, const N: usize> {
    pub current_estimate: Option<CurrentEstimate>,

    inverse_embedding_table: InverseEmbeddingTable<'a, N>,
    array_position_mapping: SMatrix<f32, 2, N>,
}

impl<'a> AttitudePipeline<'a, 7> {
    pub fn new_filled_hexagon(
        inverse_embedding_table: InverseEmbeddingTable<'a, 7>,
        array_scale_factor: f32,
    ) -> Self {
        Self {
            inverse_embedding_table,
            array_position_mapping: array_scale_factor
                * SMatrix::from_columns(&[
                    Vector2::new(0.0, 0.0),
                    Vector2::new(0.0, 1.0),
                    Vector2::new(-1.0 / 2.0, sqrtf(3.0) / 2.0),
                    Vector2::new(-1.0 / 2.0, -sqrtf(3.0) / 2.0),
                    Vector2::new(0.0, -1.0),
                    Vector2::new(1.0 / 2.0, -sqrtf(3.0) / 2.0),
                    Vector2::new(1.0 / 2.0, sqrtf(3.0) / 2.0),
                ]),
            current_estimate: None,
        }
    }
}

impl<'a, const N: usize> AttitudePipeline<'a, N> {
    pub fn update(&mut self, samples: SMatrix<f32, N, 1>) {
        let readings = self.inverse_embedding_table.invert_embedding(samples);
        let centroid = self.compute_centroid(samples);
        self.current_estimate = Some(CurrentEstimate {
            azimuth: atan2f(centroid[(1, 0)], centroid[(0, 0)]),
            elevation: PI / 2.0 - atanf(13.5 * sqrtf(3.0) / 2.0 / centroid.norm()),
        });
    }

    fn compute_centroid(&self, readings: SMatrix<f32, N, 1>) -> SMatrix<f32, 2, 1> {
        return self.array_position_mapping * readings / readings.sum();
    }

    // Computes the expected unit illumination due to an emitter centered on `centroid`
    fn compute_expected_unit_illumination(
        &self,
        centroid: SMatrix<f32, 2, 1>,
    ) -> SMatrix<f32, 7, 1> {
        unimplemented!()
    }

    // Computes a scale invariant deviation metric per entry to determine outliers in `readings` based on `expected_readings`
    fn compute_deviation_metrics(
        readings: SMatrix<f32, 7, 1>,
        expected_readings: SMatrix<f32, 7, 1>,
    ) -> SMatrix<f32, 7, 1> {
        unimplemented!()
    }
}
