use libm::sqrtf;
use nalgebra::{SMatrix, Vector2};

pub struct AttitudePipeline<const N: usize> {
    array_position_mapping: SMatrix<f32, 2, N>,
}

impl AttitudePipeline<7> {
    pub fn new_filled_hexagon(array_scale_factor: f32) -> Self {
        Self {
            array_position_mapping: array_scale_factor
                * SMatrix::from_columns(&[
                    Vector2::new(0.0, 0.0),
                    Vector2::new(sqrtf(3.0) / 2.0, 1.0 / 2.0),
                    Vector2::new(0.0, 1.0),
                    Vector2::new(-sqrtf(3.0) / 2.0, 1.0 / 2.0),
                    Vector2::new(-sqrtf(3.0) / 2.0, -1.0 / 2.0),
                    Vector2::new(0.0, -1.0),
                    Vector2::new(sqrtf(3.0) / 2.0, -1.0 / 2.0),
                ]),
        }
    }
}

impl<const N: usize> AttitudePipeline<N> {
    pub fn update(&self, readings: SMatrix<f32, N, 1>) {
        let centroid = self.compute_centroid(readings);
        unimplemented!()
    }

    fn compute_centroid(&self, readings: SMatrix<f32, N, 1>) -> SMatrix<f32, 2, 1> {
        return self.array_position_mapping * readings / readings.shape().0 as f32;
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
