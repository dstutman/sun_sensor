use crate::estimator::{TimestampedAttitude, TimestampedReading, USecTimestamp};
use nalgebra::{UnitQuaternion, UnitVector3, Vector3};

#[derive(Clone, Copy, Debug)]
pub struct ComplementaryFilter {
    pub estimate: TimestampedAttitude,
    last_reading: TimestampedReading,
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

    pub fn step(&mut self, time: USecTimestamp) {
        self.estimate = TimestampedAttitude::new(
            time,
            UnitQuaternion::from_scaled_axis(
                -self.last_reading.rate * (time - self.estimate.time).to_seconds(),
            ) * self.estimate.attitude,
        );
    }

    pub fn update(&mut self, reading: TimestampedReading) {
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

trait Estimator {
    fn predict(&mut self, dt: f32);
    fn correct(&mut self, reading: SensorReading);
    fn get_attitude(&self) -> UnitQuaternion<f32>;
}
