use core::f32::consts::PI;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

#[derive(Debug, Clone, Copy)]
pub enum Lsm9ds1Error {
    BusTransactionFailed,
    DetectionFailed,
}

type Result<T> = core::result::Result<T, Lsm9ds1Error>;

pub struct Lsm9ds1<B: Read + Write + WriteRead> {
    ag_address: u8,
    m_address: u8,
    bus: B,
}

// General registers
pub const ADDR_WHOAMI: u8 = 0x0F;
pub const ADDR_CTRLREG8: u8 = 0x22;
pub const ADDR_CTRLREG1G: u8 = 0x10;
pub const ADDR_CTRLREG6XL: u8 = 0x20;
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

impl<B: Read + Write + WriteRead> Lsm9ds1<B> {
    pub fn new(bus: B, sdo_ag: bool, sdo_m: bool) -> Result<Self> {
        let mut lsm = Self {
            bus,
            ag_address: 0x6A | sdo_ag as u8,
            m_address: 0x1C | (sdo_m as u8) << 1,
        };

        if !lsm.whoami_matches()? {
            return Err(Lsm9ds1Error::DetectionFailed);
        };

        lsm.soft_reset()?;
        lsm.configure()?;

        Ok(lsm)
    }

    pub fn soft_reset(&mut self) -> Result<()> {
        self.bus
            .write(self.ag_address, &[0x1u8 << 0])
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        Ok(())
    }

    // Configure for maximum data rates
    pub fn configure(&mut self) -> Result<()> {
        self.bus
            .write(
                self.ag_address,
                &[ADDR_CTRLREG1G, 0b110u8 << 5 | 0b00 << 3 | 0b11],
            )
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        self.bus
            .write(
                self.ag_address,
                &[ADDR_CTRLREG6XL, 0b110u8 << 5 | 0b00 << 3],
            )
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;
        Ok(())
    }

    pub fn whoami_matches(&mut self) -> Result<bool> {
        let mut whoami_byte = [0u8];
        self.bus
            .write_read(self.ag_address, &[ADDR_WHOAMI], &mut whoami_byte)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        Ok(whoami_byte[0] == 0b01101000)
    }

    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32)> {
        let mut data = [0u8; 6];
        self.bus
            .write_read(self.ag_address, &[ADDR_OUTXLG], &mut data)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        // Convert the high and low bytes of each axis to a single float
        // Full scale is set at 245 DPS, so 8.75 mDPS/LSB
        Ok((
            i16::from_le_bytes([data[0], data[1]]) as f32 * 8.75 / 1000.0 * PI / 180.0,
            i16::from_le_bytes([data[2], data[3]]) as f32 * 8.75 / 1000.0 * PI / 180.0,
            i16::from_le_bytes([data[4], data[5]]) as f32 * 8.75 / 1000.0 * PI / 180.0,
        ))
    }

    pub fn read_accel(&mut self) -> Result<(f32, f32, f32)> {
        let mut data = [0u8; 6];
        self.bus
            .write_read(self.ag_address, &[ADDR_OUTXLXL], &mut data)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        // Convert the high and low bytes of each axis to a single float
        // Full scale is set at 2g, so 0.061 mg/LSB
        Ok((
            i16::from_le_bytes([data[0], data[1]]) as f32 * 0.061 / 1000.0,
            i16::from_le_bytes([data[2], data[3]]) as f32 * 0.061 / 1000.0,
            i16::from_le_bytes([data[4], data[5]]) as f32 * 0.061 / 1000.0,
        ))
    }
}
