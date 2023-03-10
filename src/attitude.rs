use nalgebra::SMatrix;

struct AttitudeEstimator {
    position_mapping: SMatrix<f32, 2, 7>,
}

impl AttitudeEstimator {
    pub fn update(&self, readings: SMatrix<f32, 7, 1>) {
        let centroid = self.compute_centroid(readings);
        unimplemented!()
    }

    fn compute_centroid(&self, readings: SMatrix<f32, 7, 1>) -> SMatrix<f32, 2, 1> {
        return self.position_mapping * readings / readings.shape().0 as f32;
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
