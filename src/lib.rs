//! `no_std` driver for single-wire addressable LED strips (WS2812, SK6812, WS2811, WS2816).
//!
//! # Architecture
//!
//! ```text
//! LedStrip --> WireCodec (pixels => transport words) --> TransportBackend (transmit)
//!                |                                        |
//!                +-- SpiCodec (SPI bit-banging)            +-- SpiBackend (SPI)
//! ```
//!
//! The [`WireCodec`] / [`TransportBackend`] traits are intentionally generic:
//! `SpiCodec` / `SpiBackend` are the current production implementation,
//! but the trait design accommodates future backends (RMT, PIO, etc.) without
//! API changes.
//!
//! # Features
//!
//! | Feature | Default | Description |
//! |---------|---------|-------------|
//! | `spi`   | Yes     | SPI backend, requires `embedded-hal` |
//!
//! # Quick start
//!
//! ```
//! # #[cfg(feature = "spi")]
//! # {
//! # use core::convert::Infallible;
//! # use embedded_hal::spi::{ErrorType, SpiBus};
//! # struct MockSpi;
//! # impl ErrorType for MockSpi { type Error = Infallible; }
//! # impl SpiBus<u8> for MockSpi {
//! #     fn read(&mut self, _: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
//! #     fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
//! #         assert_eq!(words.len(), 555);
//! #         Ok(())
//! #     }
//! #     fn transfer(&mut self, _: &mut [u8], _: &[u8]) -> Result<(), Self::Error> { Ok(()) }
//! #     fn transfer_in_place(&mut self, _: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
//! #     fn flush(&mut self) -> Result<(), Self::Error> { Ok(()) }
//! # }
//! # let spi_peripheral = MockSpi;
//! use led_strip::{LedStrip, SpiCodec, SpiBackend, SpiEncodingPlan, Rgb, RgbOrder, Ws2812B};
//!
//! let codec = SpiCodec::<Rgb, Ws2812B>::for_protocol(
//!     SpiEncodingPlan::ws2812_3bit(), false
//! ).unwrap();
//! let backend = SpiBackend::new(spi_peripheral);
//! // 60 pixels need 60 × 9 payload bytes + 15 reset bytes = 555 bytes.
//! let mut strip: LedStrip<Rgb, Ws2812B, _, _, 1024> =
//!     LedStrip::new(RgbOrder::Grb, codec, backend);
//!
//! let pixels = [Rgb::new(255, 0, 0); 60];
//! strip.write(&pixels).unwrap();
//! # }
//! ```

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

mod api;
mod error;
mod pixel;
mod protocol;
#[cfg(feature = "spi")]
mod spi;

pub use api::{LedStrip, RefreshError, TransportBackend, WireCodec};
pub use error::{LedStripError, LedStripResult};
pub use pixel::{LedPixel, Rgb, Rgb16, Rgb16Order, RgbOrder, Rgbw, RgbwOrder};
pub use protocol::{PulseTiming, SingleWireProtocol, Sk6812, Ws2811, Ws2812B, Ws2816};
#[cfg(feature = "spi")]
pub use spi::{
    SpiBackend, SpiCodec, SpiCodecPlanError, SpiEncodeError, SpiEncodingPlan, TimingEdge,
};
