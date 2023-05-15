// Quick and dirty driver for the BNO055

use cortex_m::asm::delay;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use libm::powf;
use nalgebra::{Quaternion, UnitQuaternion};

#[derive(Debug, Clone, Copy)]
pub enum Bno055Error {
    BusTransactionFailed,
    DetectionFailed,
}

type Result<T> = core::result::Result<T, Bno055Error>;

pub struct Bno055<B: Read + Write + WriteRead> {
    address: u8,
    bus: B,
}

impl<B: Read + Write + WriteRead> Bno055<B> {
    pub fn new(bus: B) -> Result<Self> {
        let mut bno055: Bno055<B> = Self { address: 0x28, bus };

        if let Ok(true) = bno055.chipid_matches() {
            defmt::trace!("BNO presence check : PASS");
        } else {
            defmt::warn!("BNO presence check : FAIL");
            return Err(Bno055Error::DetectionFailed);
        }

        bno055.soft_reset()?;
        // Wait for the device to come back up
        delay(64000000);
        bno055.configure()?;

        Ok(bno055)
    }

    pub fn soft_reset(&mut self) -> Result<()> {
        // SYS_RST
        self.bus
            .write(self.address, &[0x3F, 0b1u8 << 5])
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        Ok(())
    }

    // Configure for maximum data rates and block data among some other stuff
    pub fn configure(&mut self) -> Result<()> {
        // OPR_MODE: NDOF
        self.bus
            .write(self.address, &[0x3D, 0b00001100])
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        // AXIS_MAP_CONFIG
        // Read reserved (could replace with modify)
        let mut axis_map_config = [0u8];
        self.bus
            .write_read(self.address, &[0x41], &mut axis_map_config)
            .map_err(|_| Bno055Error::BusTransactionFailed)?;
        self.bus
            .write(
                self.address,
                &[0x41, axis_map_config[0] & (0b11 << 6) | (0b01 << 2) | 0b10],
            )
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        // AXIS_MAP_SIGN
        let mut axis_map_sign = [0u8];
        self.bus
            .write_read(self.address, &[0x42], &mut axis_map_sign)
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        self.bus
            .write(self.address, &[0x42, axis_map_sign[0] & 0b111 | (0b1 << 1)])
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        Ok(())
    }

    pub fn chipid_matches(&mut self) -> Result<bool> {
        let mut chip_id = [0u8];

        // CHIP_ID
        self.bus
            .write_read(self.address, &[0x00], &mut chip_id)
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        Ok(chip_id[0] == 0b10100000)
    }

    pub fn read_quaternion(&mut self) -> Result<UnitQuaternion<f32>> {
        let mut qua_data = [0u8; 8];

        // QUA_DATA
        self.bus
            .write_read(self.address, &[0x27], &mut qua_data)
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        Ok(UnitQuaternion::from_quaternion(Quaternion::new(
            i16::from_le_bytes([qua_data[6], qua_data[7]]) as f32 / powf(2.0, 14.0),
            i16::from_le_bytes([qua_data[0], qua_data[1]]) as f32 / powf(2.0, 14.0),
            i16::from_le_bytes([qua_data[2], qua_data[3]]) as f32 / powf(2.0, 14.0),
            i16::from_le_bytes([qua_data[4], qua_data[5]]) as f32 / powf(2.0, 14.0),
        )))
    }
}
