use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

#[derive(Debug, Clone, Copy)]
pub enum Lsm9ds1Error {
    BusTransactionFailed,
}

type Result<T> = core::result::Result<T, Lsm9ds1Error>;

pub struct Lsm9ds1<B: Read + Write + WriteRead> {
    ag_address: u8,
    m_address: u8,
    bus: B,
}

impl<B: Read + Write + WriteRead> Lsm9ds1<B> {
    pub fn new(bus: B, sdo_ag: bool, sdo_m: bool) -> Self {
        Self {
            bus,
            ag_address: 0x6A | sdo_ag as u8,
            m_address: 0x1C | (sdo_m as u8) << 1,
        }
    }

    pub fn whoami_matches(&mut self) -> Result<bool> {
        let mut whoami_byte = [0u8];
        self.bus
            .write_read(self.ag_address, &[0x0Fu8], &mut whoami_byte)
            .map_err(|_| Lsm9ds1Error::BusTransactionFailed)?;

        Ok(whoami_byte[0] == 0b01101000)
    }
}
