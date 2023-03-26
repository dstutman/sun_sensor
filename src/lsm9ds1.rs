use core::f32::consts::PI;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use nalgebra::{Matrix3, Vector3};

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

impl<B: Read + Write + WriteRead> Lsm9ds1<B> {
    pub fn new(bus: B, sdo_ag: bool, sdo_m: bool) -> Result<Self> {
        let mut lsm9ds1 = Self {
            bus,
            ag_address: 0x6A | sdo_ag as u8,
            m_address: 0x1C | (sdo_m as u8) << 1,
        };

        if let Ok(true) = lsm9ds1.whoami_matches() {
            defmt::trace!("LSM9DS1 presence check : PASS");
        } else {
            defmt::warn!("LSM9DS1 presence check : FAIL");
            return Err(Lsm9ds1Error::DetectionFailed);
        }

        lsm9ds1.soft_reset()?;
        lsm9ds1.configure()?;

        Ok(lsm9ds1)
    }

    pub fn soft_reset(&mut self) -> Result<()> {
        self.bus
            .write(self.ag_address, &[ADDR_CTRLREG8, 0b1u8 << 0])
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        self.bus
            .write(self.m_address, &[ADDR_CTRLREG2M, 0b1u8 << 2])
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        Ok(())
    }

    // Configure for maximum data rates and block data among some other stuff
    pub fn configure(&mut self) -> Result<()> {
        self.bus
            .write(
                self.ag_address,
                &[ADDR_CTRLREG1G, 0b110u8 << 5 | 0b00u8 << 3 | 0b11u8],
            )
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        self.bus
            .write(self.ag_address, &[ADDR_CTRLREG8, 0b1u8 << 6 | 0b1u8 << 2])
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        self.bus
            .write(
                self.ag_address,
                &[ADDR_CTRLREG6XL, 0b110u8 << 5 | 0b00u8 << 3],
            )
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        self.bus
            .write(
                self.m_address,
                &[ADDR_CTRLREG1M, 0b11u8 << 5 | 0b111u8 << 2 | 0b1u8 << 1],
            )
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        // Continuous-conversion mode
        self.bus
            .write(self.m_address, &[ADDR_CTRLREG3M, 0b00u8])
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        // Ultra-high performance on Z-axis
        self.bus
            .write(self.m_address, &[ADDR_CTRLREG4M, 0b11u8 << 2])
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        self.bus
            .write(self.m_address, &[ADDR_CTRLREG5M, 0b1u8 << 6])
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        Ok(())
    }

    pub fn whoami_matches(&mut self) -> Result<bool> {
        let mut ag_whoami_byte = [0u8];
        self.bus
            .write_read(self.ag_address, &[ADDR_WHOAMI], &mut ag_whoami_byte)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        let mut m_whoami_byte = [0u8];
        self.bus
            .write_read(self.m_address, &[ADDR_WHOAMIM], &mut m_whoami_byte)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        Ok(ag_whoami_byte[0] == 0b01101000u8 && m_whoami_byte[0] == 0b00111101u8)
    }

    pub fn read_accel(&mut self) -> Result<Vector3<f32>> {
        let mut data = [0u8; 6];
        self.bus
            .write_read(self.ag_address, &[ADDR_OUTXLXL], &mut data)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        // Convert the high and low bytes of each axis to a single float
        // Full scale is set at 2g, so 0.061 mg/LSB
        Ok(Vector3::new(
            i16::from_le_bytes([data[0], data[1]]) as f32 * 0.061 / 1000.0,
            i16::from_le_bytes([data[2], data[3]]) as f32 * 0.061 / 1000.0,
            i16::from_le_bytes([data[4], data[5]]) as f32 * 0.061 / 1000.0,
        ))
    }

    pub fn read_gyro(&mut self) -> Result<Vector3<f32>> {
        let mut data = [0u8; 6];
        self.bus
            .write_read(self.ag_address, &[ADDR_OUTXLG], &mut data)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        // Convert the high and low bytes of each axis to a single float
        // Full scale is set at 245 DPS, so 8.75 mDPS/LSB
        Ok(Vector3::new(
            i16::from_le_bytes([data[0], data[1]]) as f32 * 8.75 / 1000.0 * PI / 180.0,
            i16::from_le_bytes([data[2], data[3]]) as f32 * 8.75 / 1000.0 * PI / 180.0,
            i16::from_le_bytes([data[4], data[5]]) as f32 * 8.75 / 1000.0 * PI / 180.0,
        ))
    }

    pub fn read_mag(&mut self) -> Result<Vector3<f32>> {
        let mut data = [0u8; 6];
        self.bus
            .write_read(self.m_address, &[ADDR_XLM], &mut data)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        // Convert the high and low bytes of each axis to a single float
        // Full scale is set at 4 gauss, so 0.14 mgauss/LSB
        Ok(Vector3::new(
            i16::from_le_bytes([data[0], data[1]]) as f32 * 0.14 / 1000.0,
            i16::from_le_bytes([data[2], data[3]]) as f32 * 0.14 / 1000.0,
            i16::from_le_bytes([data[4], data[5]]) as f32 * 0.14 / 1000.0,
        ))
    }
}

pub struct CalibrationParameters {
    accel_bias: Vector3<f32>,
    gyro_bias: Vector3<f32>,
    mag_bias: Vector3<f32>,
}

impl CalibrationParameters {
    pub fn new(accel_bias: Vector3<f32>, gyro_bias: Vector3<f32>, mag_bias: Vector3<f32>) -> Self {
        Self {
            accel_bias,
            gyro_bias,
            mag_bias,
        }
    }
}

pub struct CorrectedLsm9ds1<B: Read + Write + WriteRead> {
    lsm: Lsm9ds1<B>,
    cal_params: CalibrationParameters,
}

impl<B: Read + Write + WriteRead> CorrectedLsm9ds1<B> {
    /// Applies correction parameters and remaps all measurements into a right handed
    /// coordinate system sharing the x and y axes of figure 1 in the datasheet. These
    /// axes are also marked on the module PCB.
    pub fn new(lsm: Lsm9ds1<B>, cal_params: CalibrationParameters) -> Self {
        Self { lsm, cal_params }
    }

    pub fn read_accel(&mut self) -> Result<Vector3<f32>> {
        self.lsm
            .read_accel()
            .map(|a| a - self.cal_params.accel_bias)
            .map(|a| a.component_mul(&Vector3::new(-1.0, -1.0, 1.0)))
    }

    pub fn read_gyro(&mut self) -> Result<Vector3<f32>> {
        self.lsm
            .read_gyro()
            .map(|g| g - self.cal_params.gyro_bias)
            .map(|g| g.component_mul(&Vector3::new(1.0, 1.0, -1.0)))
    }

    pub fn read_mag(&mut self) -> Result<Vector3<f32>> {
        self.lsm
            .read_mag()
            .map(|m| m - self.cal_params.mag_bias)
            // TODO: FIXME
            .map(|m| m.component_mul(&Vector3::new(-1.0, 1.0, -1.0)))
    }
}
