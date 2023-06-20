use crate::ldr_array;

#[derive(Clone, Copy, Debug, serde::Serialize, PartialEq)]
pub enum ChannelFault {
    GroundFault,
    ShortCircuit,
}

#[derive(Clone, Copy, Debug, serde::Serialize, PartialEq)]
pub enum ChannelHealth {
    Ok,
    Fault(ChannelFault),
}

#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct ArrayStatus {
    pub channels: [ChannelHealth; 6],
}

impl ArrayStatus {
    pub fn from_reading(readings: ldr_array::Reading<6>) -> Self {
        let mut iter = readings.iter().map(|&e| {
            if e == 0 {
                ChannelHealth::Fault(ChannelFault::GroundFault)
            } else if e == 4095 {
                // Approx, based on divider
                ChannelHealth::Fault(ChannelFault::ShortCircuit)
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
