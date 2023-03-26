use nalgebra::{UnitQuaternion, UnitVector3, Vector3};

#[derive(Clone, Copy, Debug)]
pub struct SensorReading {
    acceleration: UnitVector3<f32>,
    rate: Vector3<f32>,
}

impl SensorReading {
    pub fn new(acceleration: UnitVector3<f32>, rate: Vector3<f32>) -> Self {
        Self { acceleration, rate }
    }
}

impl Default for SensorReading {
    fn default() -> Self {
        SensorReading {
            acceleration: UnitVector3::new_unchecked(Vector3::new(0.0, 0.0, 1.0)),
            rate: Default::default(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ComplementaryFilter {
    pub estimate: UnitQuaternion<f32>,
    last_reading: SensorReading,
    alpha: f32,
}

impl ComplementaryFilter {
    pub fn new(alpha: f32) -> Self {
        Self {
            estimate: Default::default(),
            last_reading: Default::default(),
            alpha,
        }
    }

    pub fn update(&mut self, reading: SensorReading, dt: f32) {
        let average_rate = (self.last_reading.rate + reading.rate) / 2.0;
        let propagated_state = UnitQuaternion::from_scaled_axis(-average_rate * dt) * self.estimate;

        let acceleration_state = UnitQuaternion::rotation_between_axis(
            &UnitVector3::new_unchecked(Vector3::new(0.0, 0.0, 1.0)),
            &reading.acceleration,
        )
        .unwrap();

        self.estimate = propagated_state.nlerp(&acceleration_state, self.alpha);
        self.last_reading = reading;
    }
}
