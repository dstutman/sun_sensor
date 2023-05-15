use nalgebra::SVector;

use crate::ldr_array;

pub struct Faults<const N: usize> {
    open_circuit: SVector<bool, N>,
    dead_short: SVector<bool, N>,
}

impl<const N: usize> Faults<N> {
    pub fn from_reading(readings: ldr_array::Reading<N>) -> Self {
        Self {
            open_circuit: readings.map(|e| e > (2 ^ 12)),
            dead_short: readings.map(|e| e == 0),
        }
    }
}
