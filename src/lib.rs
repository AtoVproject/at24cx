#![cfg_attr(not(test), no_std)]

use core::cmp::min;
use core::fmt::Debug;
use embedded_hal_async::{
    delay::DelayNs,
    i2c::{ErrorType as I2cErrorType, I2c},
};
use embedded_storage_async::nor_flash::{
    ErrorType as StorageErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

// TODO: These are only valid for AT24CM01. Implement the others
const PAGE_SIZE: usize = 256;
const ADDRESS_BYTES: usize = 2;

/// Custom error type for the various errors that can be thrown by AT24Cx
/// Can be converted into a NorFlashError.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error<E: Debug> {
    I2cError(E),
    NotAligned,
    OutOfBounds,
    WriteEnableFail,
    ReadbackFail,
}

impl<E: Debug> NorFlashError for Error<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Error::NotAligned => NorFlashErrorKind::NotAligned,
            Error::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            _ => NorFlashErrorKind::Other,
        }
    }
}

pub struct Address(pub u8, pub u8);

impl From<Address> for u8 {
    fn from(a: Address) -> Self {
        0xa0 | (a.1 << 2) | (a.0 << 1)
    }
}

pub struct At24Cx<I2C, D> {
    address_bits: usize,
    base_address: u8,
    delay: D,
    i2c: I2C,
}

impl<I2C, E: Debug, D: DelayNs> At24Cx<I2C, D>
where
    I2C: I2c<Error = E>,
{
    pub fn new(i2c: I2C, address: Address, address_bits: usize, delay: D) -> Self {
        Self {
            address_bits,
            base_address: address.into(),
            delay,
            i2c,
        }
    }

    fn get_device_address(&self, memory_address: u32) -> Result<u8, Error<E>> {
        if memory_address >= (1 << self.address_bits) {
            return Err(Error::OutOfBounds);
        }
        let p0 = if memory_address & 1 << 16 == 0 { 0 } else { 1 };
        Ok(self.base_address & p0 << 1)
    }

    async fn page_write(&mut self, address: u32, data: &[u8]) -> Result<(), Error<E>> {
        if data.is_empty() {
            return Ok(());
        }

        // check this before to ensure that data.len() fits into u32
        // ($page_size always fits as its maximum value is 256).
        if data.len() > PAGE_SIZE {
            // This would actually be supported by the EEPROM but
            // the data in the page would be overwritten
            return Err(Error::OutOfBounds);
        }

        let page_boundary = address | (PAGE_SIZE as u32 - 1);
        if address + data.len() as u32 > page_boundary + 1 {
            // This would actually be supported by the EEPROM but
            // the data in the page would be overwritten
            return Err(Error::OutOfBounds);
        }
        //
        let device_addr = self.get_device_address(address)?;
        let mut payload: [u8; ADDRESS_BYTES + PAGE_SIZE] = [0; ADDRESS_BYTES + PAGE_SIZE];
        payload[0] = (address >> 8) as u8;
        payload[1] = address as u8;
        payload[ADDRESS_BYTES..ADDRESS_BYTES + data.len()].copy_from_slice(data);
        self.i2c
            .write(device_addr, &payload[..ADDRESS_BYTES + data.len()])
            .await
            .map_err(Error::I2cError)
    }
}

impl<I2C, E: Debug, D: DelayNs> StorageErrorType for At24Cx<I2C, D>
where
    I2C: I2cErrorType<Error = E>,
{
    type Error = Error<E>;
}

impl<I2C, E: Debug, D: DelayNs> ReadNorFlash for At24Cx<I2C, D>
where
    I2C: I2c<Error = E>,
{
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let device_address = self.get_device_address(offset)?;
        let mut memaddr = [0; 2];
        memaddr[0] = (offset >> 8) as u8;
        memaddr[1] = offset as u8;
        self.i2c
            .write_read(device_address, &memaddr[..2], bytes)
            .await
            .map_err(Error::I2cError)
    }

    fn capacity(&self) -> usize {
        1 << self.address_bits
    }
}

impl<I2C, E: Debug, D: DelayNs> NorFlash for At24Cx<I2C, D>
where
    I2C: I2c<Error = E>,
    E: Into<Error<E>>,
{
    const WRITE_SIZE: usize = 1;

    const ERASE_SIZE: usize = 1;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        todo!();
    }

    async fn write(&mut self, mut offset: u32, mut bytes: &[u8]) -> Result<(), Self::Error> {
        if offset as usize + bytes.len() > self.capacity() {
            return Err(Error::OutOfBounds);
        }
        while !bytes.is_empty() {
            let this_page_offset = offset as usize % PAGE_SIZE;
            let this_page_remaining = PAGE_SIZE - this_page_offset;
            let chunk_size = min(bytes.len(), this_page_remaining);
            self.page_write(offset, &bytes[..chunk_size]).await?;
            offset += chunk_size as u32;
            bytes = &bytes[chunk_size..];
            // TODO: ACK polling:
            // Acknowledge Polling: Once the internally timed write cycle has started and the EEPROM inputs are disabled, Acknowledge Polling can be initiated. This involves sending a Start condition followed by the device address word. The read/write bit is representative of the operation desired. Only if the internal write cycle has completed will the EEPROM respond with a zero, allowing a new read or write sequence to be initiated.
            self.delay.delay_ms(5).await;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
