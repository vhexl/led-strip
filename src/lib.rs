//! `no_std` driver for single-wire addressable LED strips (WS2812, SK6812, WS2811, WS2816).
//!
//! # Architecture
//!
//! ```text
//! LedStrip ──► WireCodec (pixels → transport words) ──► TransportBackend (transmit)
//!                 │                                         │
//!                 └─ SpiCodec (SPI bit-banging)              └─ SpiBackend (SPI)
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
//! | `spi`   | ✅      | SPI backend, requires `embedded-hal` |
//!
//! # Quick start
//!
//! ```ignore
//! use led_strip::{LedStrip, LedStripConfig, SpiCodec, SpiBackend, SpiEncodingPlan, Rgb, Ws2812B};
//!
//! let config = LedStripConfig::ws2812b(60);
//! let codec = SpiCodec::for_protocol::<Rgb, Ws2812B>(
//!     SpiEncodingPlan::ws281x_3bit(), false
//! )?;
//! let backend = SpiBackend::new(spi_peripheral);
//! let mut strip: LedStrip<Rgb, Ws2812B, _, _, 64, 256> =
//!     LedStrip::new(config, codec, backend)?;
//!
//! strip.set(0, Rgb::new(255, 0, 0))?;
//! strip.refresh()?;
//! ```

#![cfg_attr(not(test), no_std)]

mod api;
mod config;
mod error;
mod frame;
mod pixel;
mod protocol;
#[cfg(feature = "spi")]
mod spi;

pub use api::{LedStrip, RefreshError, TransportBackend, WireCodec};
pub use config::LedStripConfig;
pub use error::{LedStripError, LedStripResult};
pub use frame::FrameBuf;
pub use frame::FrameError;
pub use pixel::{LedPixel, PixelKind, Rgb, Rgb16, Rgb16Order, RgbOrder, Rgbw, RgbwOrder};
pub use protocol::{
    BitOrder, PulseTiming, SingleWireProtocol, Sk6812, Ws2811, Ws2812B, Ws2816,
};
#[cfg(feature = "spi")]
pub use spi::{SpiBackend, SpiCodec, SpiCodecPlanError, SpiEncodingPlan, TimingEdge};
