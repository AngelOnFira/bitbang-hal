//! Serial communication (USART)
//!
//! This implementation consumes the following hardware resources:
//! - Periodic timer to mark clock cycles
//! - Output GPIO pin for transmission (TX)
//! - Input GPIO pin for reception (RX)
//!
//! The timer must be configured to twice the desired communication frequency.
//!

use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::serial;
use embedded_hal::timer::{CountDown, Periodic};
use nb::block;

/// Serial communication error type
#[derive(Debug)]
pub enum Error<E> {
    /// Bus error
    Bus(E),
    /// Invalid interrupt call
    InvalidInterrupt,
}

/// Bit banging serial communication (USART) device
pub struct Serial<TX, RX, Timer>
where
    TX: OutputPin,
    RX: InputPin,
    Timer: CountDown + Periodic,
{
    tx: TX,
    rx: RX,
    timer: Timer,
}

/// Reset the timer
pub trait Reset {
    /// Reset the timer
    fn reset(&mut self);
}

/// Nop
pub trait Nop {
    /// Nop
    fn nop(&mut self);
}

impl<TX, RX, Timer, E> Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic + Reset + Nop,
{
    /// Create instance
    pub fn new(tx: TX, rx: RX, timer: Timer) -> Self {
        Serial { tx, rx, timer }
    }

    #[inline]
    fn reset_timer(&mut self) {
        self.timer.reset();
    }

    #[inline]
    fn wait_for_timer(&mut self) {
        block!(self.timer.wait()).ok();
    }

    #[inline]
    fn nop(&mut self) {
        self.timer.nop();
    }
}

impl<TX, RX, Timer, E> serial::Write<u8> for Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic + Reset + Nop,
{
    type Error = crate::serial::Error<E>;

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        // return Ok(());
        let mut data_out = byte;
        self.tx.set_low().map_err(Error::Bus)?; // start bit
        self.reset_timer();

        for _ in 0..5 {
            self.nop();
        }

        self.wait_for_timer();
        for _bit in 0..8 {
            if data_out & 1 == 1 {
                self.tx.set_high().map_err(Error::Bus)?;
            } else {
                self.tx.set_low().map_err(Error::Bus)?;
            }
            data_out >>= 1;
            self.wait_for_timer();
        }
        self.tx.set_high().map_err(Error::Bus)?; // stop bit
        self.wait_for_timer();
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        Ok(())
    }
}

impl<TX, RX, Timer, E> serial::Read<u8> for Serial<TX, RX, Timer>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Timer: CountDown + Periodic + Reset + Nop,
{
    type Error = crate::serial::Error<E>;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut data_in = 0;

        // If we're currently in a high bit, then this is an invalid inturupt
        // call. Return a string of an error.
        if self.rx.is_high().map_err(Error::Bus)? {
            return Err(nb::Error::Other(Error::InvalidInterrupt));
        }

        // wait for start bit
        // while self.rx.is_high().map_err(Error::Bus)? {}
        // reset timer

        // nop 100 times to align in the middle of the bit
        // for _ in 0..80 {
        //     self.nop();
        // }

        self.reset_timer();

        for _ in 0..5 {
            self.nop();
        }

        self.wait_for_timer();
        for _bit in 0..8 {
            data_in >>= 1;
            if self.rx.is_high().map_err(Error::Bus)? {
                data_in |= 0x80;
            }
            self.wait_for_timer();
        }
        // wait for stop bit
        self.wait_for_timer();

        Ok(data_in)
    }
}
