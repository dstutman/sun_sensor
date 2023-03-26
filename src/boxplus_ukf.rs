//! Implements attitude estimation using a UKF with box-plus algebra
//! Attitude is parameterized as a quaternion

use core::{
    f32::{consts::PI, EPSILON, MAX},
    ops::{Add, AddAssign, Sub},
};

use libm::atanf;
use nalgebra::{Matrix6, SMatrix, SVector, UnitQuaternion, Vector3, Vector6};

// Maximum number of iterations for certain methods such as `mean` of `State` sigma points
const MAX_ITERS: usize = 10;

type ManifoldDelta = Vector6<f32>;

trait ManifoldDeltaExt {
    fn from_scaled_axis(scaled_axis: Vector3<f32>) -> Self;
}

impl ManifoldDeltaExt for ManifoldDelta {
    fn from_scaled_axis(scaled_axis: Vector3<f32>) -> Self {
        ManifoldDelta::new(
            scaled_axis[0],
            scaled_axis[1],
            scaled_axis[2],
            0.0,
            0.0,
            0.0,
        )
    }
}

#[derive(Clone, Copy, Default, Debug)]
pub struct State {
    pub attitude: UnitQuaternion<f32>,
    pub angular_rate: Vector3<f32>,
}

impl State {
    pub fn new(attitude: UnitQuaternion<f32>, angular_rate: Vector3<f32>) -> Self {
        Self {
            attitude,
            angular_rate,
        }
    }
}

// Implement the box-plus operator for the state
impl Add<ManifoldDelta> for State {
    type Output = Self;

    fn add(mut self, rhs: ManifoldDelta) -> Self::Output {
        // TODO: Figure out how to use a range instead of element axis with nalgebra
        self.attitude *= UnitQuaternion::from_scaled_axis(Vector3::new(rhs[0], rhs[1], rhs[2]));
        self.angular_rate += Vector3::new(rhs[3], rhs[4], rhs[5]);
        self
    }
}

impl AddAssign<ManifoldDelta> for State {
    fn add_assign(&mut self, rhs: ManifoldDelta) {
        *self = *self + rhs;
    }
}

impl Sub for State {
    type Output = ManifoldDelta;

    fn sub(self, rhs: Self) -> Self::Output {
        fn log_bar(quaternion: UnitQuaternion<f32>) -> Vector3<f32> {
            let raw_quat = quaternion.quaternion();
            if raw_quat.w < 1E2 * EPSILON {
                return PI / 2.0 * raw_quat.vector().normalize();
            } else if raw_quat.vector().norm() < 1E2 * EPSILON {
                return Vector3::zeros();
            } else {
                return atanf(raw_quat.vector().norm_squared() / raw_quat.w)
                    * raw_quat.vector().normalize();
            }
        }

        let quaternion_delta = 2.0 * log_bar(rhs.attitude.inverse() * self.attitude);
        let omega_delta = self.angular_rate - rhs.angular_rate;

        ManifoldDelta::new(
            quaternion_delta[0],
            quaternion_delta[1],
            quaternion_delta[2],
            omega_delta[0],
            omega_delta[1],
            omega_delta[2],
        )
    }
}

/// Covariances are represented in manifold space
pub type StateCovariance = Matrix6<f32>;

/// Intuitively, the 1-sigma uncertainty
#[derive(Clone, Copy, Default, Debug)]
pub struct Uncertainty {
    pub attitude: UnitQuaternion<f32>,
    pub angular_rate: Vector3<f32>,
}

trait States {
    fn update(&mut self, mean: State, covariance: StateCovariance);
    fn update_with_delta(&mut self, mean: State, delta: ManifoldDelta, covariance: StateCovariance);
    fn mean(&self) -> State;
    fn covariance(&self) -> StateCovariance;
}

impl<const N: usize> States for [State; N] {
    fn update(&mut self, mean: State, covariance: StateCovariance) {
        // TODO: handle unwrap
        let cholesky = covariance.cholesky().unwrap().l();

        self[0] = mean;
        for i in 0..(self.len() / 2) {
            self[i] = mean + cholesky.column(i).into();
            self[i + self.len() / 2 - 1] = mean + -cholesky.column(i);
        }
    }

    fn update_with_delta(
        &mut self,
        mean: State,
        delta: ManifoldDelta,
        covariance: StateCovariance,
    ) {
        // TODO: handle unwrap
        let cholesky = covariance.cholesky().unwrap().l();

        self[0] = mean;
        for i in 0..(self.len() / 2) {
            // NOTE: These parens are NOT superfluous.
            // They define the priority of box-plus composition.
            // DO NOT REMOVE THEM.
            self[i] = mean + (delta + cholesky.column(i));
            self[i + self.len() / 2 - 1] = mean + (delta + -cholesky.column(i));
        }
    }

    fn mean(&self) -> State {
        let mut x = self[0];

        // TODO: Early termination
        for _ in 0..MAX_ITERS {
            x += 1.0 / self.len() as f32
                * self
                    .iter()
                    .fold(ManifoldDelta::zeros(), |acc, s| acc + (*s - x));
        }

        x
    }

    fn covariance(&self) -> StateCovariance {
        let mut x = StateCovariance::zeros();

        for state in self[1..].iter() {
            x += (*state - self[0]) * (*state - self[0]).transpose();
        }

        1.0 / 2.0 * x
    }
}

/// Covariances are represented in manifold space
pub type ObservationCovariance = SMatrix<f32, 9, 9>;

// Expected observations are:
// - in a coordinate system as follows
//   - X is North
//   - Z is down
//   - Y completes a right handed system
// - in g for accelerations
// - in radian per second for angular_rate
// - a normalized magnetic field
// Fields are ordered:
// - ax
// - ay
// - az
// - gx
// - gy
// - gz
// - mx
// - my
// - mz
pub type Observation = SVector<f32, 9>;

trait ObsevationExt {
    fn estimated_from(state: State) -> Self;
    fn from_measurements(
        acceleration: Vector3<f32>,
        angular_rate: Vector3<f32>,
        magnetic_field: Vector3<f32>,
    ) -> Self;
}

impl ObsevationExt for Observation {
    fn estimated_from(state: State) -> Self {
        let acceleration = state.attitude * Vector3::new(0.0, 0.0, -1.0);
        let angular_rate = state.angular_rate;
        let magnetic_field = state.attitude * Vector3::new(1.0, 0.0, 0.0);
        SVector::from_column_slice(&[
            acceleration.x,
            acceleration.y,
            acceleration.z,
            angular_rate.x,
            angular_rate.y,
            angular_rate.z,
            magnetic_field.x,
            magnetic_field.y,
            magnetic_field.z,
        ])
    }

    fn from_measurements(
        acceleration: Vector3<f32>,
        angular_rate: Vector3<f32>,
        magnetic_field: Vector3<f32>,
    ) -> Self {
        SVector::from_column_slice(&[
            acceleration.x,
            acceleration.y,
            acceleration.z,
            angular_rate.x,
            angular_rate.y,
            angular_rate.z,
            magnetic_field.x,
            magnetic_field.y,
            magnetic_field.z,
        ])
    }
}

trait Observations {
    fn mean(&self) -> Observation;
    fn covariance(&self) -> ObservationCovariance;
}

impl<const N: usize> Observations for [Observation; N] {
    fn mean(&self) -> Observation {
        let mut x = self[0];

        // TODO: Early termination
        for _ in 0..MAX_ITERS {
            x += 1.0 / self.len() as f32
                * self
                    .iter()
                    .fold(Observation::zeros(), |acc, s| acc + (*s - x));
        }

        x
    }

    fn covariance(&self) -> ObservationCovariance {
        let mut x = ObservationCovariance::zeros();

        for observation in self[1..].iter() {
            x += (*observation - self[0]) * (*observation - self[0]).transpose();
        }

        1.0 / 2.0 * x
    }
}

pub struct BpUkf {
    pub estimate: State,
    process_covariance: StateCovariance,
    observation_covariance: ObservationCovariance,
    // Storing the sigma points and covariance saves a bunch of compute later
    sigma_points: [State; 6 * 2 + 1],
    estimate_covariance: StateCovariance,
}

// TODO: Guard against `correct` without `update`
impl BpUkf {
    pub fn new(
        initial_estimate: State,
        process_covariance: StateCovariance,
        observation_covariance: ObservationCovariance,
    ) -> Self {
        Self {
            estimate: initial_estimate,
            process_covariance,
            observation_covariance,
            sigma_points: Default::default(),
            estimate_covariance: StateCovariance::identity() * MAX / 2.0,
        }
    }

    pub fn update(&mut self, dt: f32) {
        // Propagate the existing sigma points
        for state in self.sigma_points.as_mut() {
            // Update the attitude, angular rate is unchanged
            // TODO: Fix this stupid dereferencing
            *state = *state + ManifoldDelta::from_scaled_axis(state.angular_rate * dt);
        }

        // Compute the new mean and covariance estimates from the sigma points
        self.estimate = self.sigma_points.mean();
        self.estimate_covariance = self.sigma_points.covariance() + self.process_covariance * dt;

        // Generate new sigma points
        // Do it here instead of in the correction as is traditionally done
        // in case we want to call update multiple times before a sensor
        // reading arrives.
        self.sigma_points
            .update(self.estimate, self.estimate_covariance);

        for state in self.sigma_points.as_mut() {
            *state = *state + ManifoldDelta::from_scaled_axis(state.angular_rate * dt);
        }
    }

    pub fn correct(&mut self, observation: Observation) {
        let sigma_observations = self.sigma_points.map(|s| Observation::estimated_from(s));
        let estimated_observation = sigma_observations.mean();
        let observation_covariance = sigma_observations.covariance() + self.observation_covariance;

        // TODO compute cross-variance
        let cross_variance = SMatrix::<f32, 6, 9>::zeros();
        // TODO: handle unwrap
        let gain = cross_variance * observation_covariance.try_inverse().unwrap();

        // Compute new sigma points
        let innovation = gain * (observation - estimated_observation);
        let intermediate_covariance =
            self.estimate_covariance - gain * observation_covariance * gain.transpose();
        self.sigma_points
            .update_with_delta(self.estimate, innovation, intermediate_covariance);

        // Compute and apply the final updates
        self.estimate = self.sigma_points.mean();
        self.estimate_covariance = self.sigma_points.covariance();
    }
}
