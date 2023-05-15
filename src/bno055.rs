// Quick and dirty driver for the BNO055

use cortex_m::asm::delay;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use libm::powf;
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

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

// General registers
pub const ADDR_WHOAMI: u8 = 0x0F;
pub const ADDR_CTRLREG7: u8 = 0x21;
pub const ADDR_CTRLREG8: u8 = 0x22;
pub const ADDR_CTRLREG1G: u8 = 0x10;
pub const ADDR_CTRLREG6XL: u8 = 0x20;
pub const ADDR_WHOAMIM: u8 = 0x0F;
pub const ADDR_CTRLREG1M: u8 = 0x20;
pub const ADDR_CTRLREG2M: u8 = 0x21;
pub const ADDR_CTRLREG3M: u8 = 0x22;
pub const ADDR_CTRLREG4M: u8 = 0x23;
pub const ADDR_CTRLREG5M: u8 = 0x24;
// Gyroscope registers
pub const ADDR_OUTXLG: u8 = 0x18;
pub const ADDR_OUTXHG: u8 = 0x19;
pub const ADDR_OUTYLG: u8 = 0x1A;
pub const ADDR_OUTYHG: u8 = 0x1B;
pub const ADDR_OUTZLG: u8 = 0x1C;
pub const ADDR_OUTZHG: u8 = 0x1D;
// Accelerometer registers
pub const ADDR_OUTXLXL: u8 = 0x28;
pub const ADDR_OUTXHXL: u8 = 0x29;
pub const ADDR_OUTYLXL: u8 = 0x2A;
pub const ADDR_OUTYHXL: u8 = 0x2B;
pub const ADDR_OUTZLXL: u8 = 0x2C;
pub const ADDR_OUTZHXL: u8 = 0x2D;
// Magnetometer registers
pub const ADDR_XLM: u8 = 0x28;
pub const ADDR_XHM: u8 = 0x29;
pub const ADDR_YLM: u8 = 0x2A;
pub const ADDR_YHM: u8 = 0x2B;
pub const ADDR_ZLM: u8 = 0x2C;
pub const ADDR_ZHM: u8 = 0x2D;

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
                &[
                    0x41,
                    axis_map_config[0] & (0b11 << 6) | (0b00 << 4) | (0b01 << 2) | (0b10 << 0),
                ],
            )
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        // AXIS_MAP_SIGN
        let mut axis_map_sign = [0u8];
        self.bus
            .write_read(self.address, &[0x42], &mut axis_map_sign)
            .map_err(|_| Bno055Error::BusTransactionFailed)?;

        self.bus
            .write(
                self.address,
                &[
                    0x42,
                    axis_map_sign[0] & (0b111 << 0) | (0b0 << 2) | (0b1 << 1) | (0b0 << 0),
                ],
            )
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

        return Ok(UnitQuaternion::from_quaternion(Quaternion::new(
            i16::from_le_bytes([qua_data[6], qua_data[7]]) as f32 / powf(2.0, 14.0),
            i16::from_le_bytes([qua_data[0], qua_data[1]]) as f32 / powf(2.0, 14.0),
            i16::from_le_bytes([qua_data[2], qua_data[3]]) as f32 / powf(2.0, 14.0),
            i16::from_le_bytes([qua_data[4], qua_data[5]]) as f32 / powf(2.0, 14.0),
        )));
    }
}
