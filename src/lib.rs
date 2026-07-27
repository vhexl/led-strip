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
//! [`SpiCodec`] / [`SpiBackend`] are the current production implementation,
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
//! ```ignore
//! use led_strip::{LedStrip, SpiCodec, SpiBackend, SpiEncodingPlan, Rgb, RgbOrder, Ws2812B};
//!
//! let codec = SpiCodec::<Rgb, Ws2812B>::for_protocol(
//!     SpiEncodingPlan::ws2812_3bit(), false
//! )?;
//! let backend = SpiBackend::new(spi_peripheral);
//! let mut strip: LedStrip<Rgb, Ws2812B, _, _, 256> =
//!     LedStrip::new(RgbOrder::Grb, codec, backend);
//!
//! let pixels = [Rgb::new(255, 0, 0); 60];
//! strip.write(&pixels)?;
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
pub use pixel::{LedPixel, PixelKind, Rgb, Rgb16, Rgb16Order, RgbOrder, Rgbw, RgbwOrder};
pub use protocol::{BitOrder, PulseTiming, SingleWireProtocol, Sk6812, Ws2811, Ws2812B, Ws2816};
#[cfg(feature = "spi")]
pub use spi::{
    SpiBackend, SpiCodec, SpiCodecPlanError, SpiEncodeError, SpiEncodingPlan, TimingEdge,
};
