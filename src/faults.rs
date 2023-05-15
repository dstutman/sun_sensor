use embedded_hal::adc::Channel;
use nalgebra::SVector;

use crate::ldr_array;

#[derive(Clone, Copy, Debug, serde::Serialize)]
pub enum ChannelFault {
    OpenCircuit,
    ShortCircuit,
}

#[derive(Clone, Copy, Debug, serde::Serialize)]
pub enum ChannelHealth {
    Ok,
    Fault(ChannelFault),
}

#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct ArrayStatus {
    channels: [ChannelHealth; 6],
}

impl ArrayStatus {
    pub fn from_reading(readings: ldr_array::Reading<6>) -> Self {
        let mut iter = readings.iter().map(|&e| {
            if e == 0 {
                ChannelHealth::Fault(ChannelFault::ShortCircuit)
            } else if e == 4095 {
                ChannelHealth::Fault(ChannelFault::OpenCircuit)
            } else {
                ChannelHealth::Ok
            }
        });

        Self {
            channels: [
                iter.next().unwrap(),
                iter.next().unwrap(),
                iter.next().unwrap(),
                iter.next().unwrap(),
                iter.next().unwrap(),
                iter.next().unwrap(),
            ],
        }
    }
}
