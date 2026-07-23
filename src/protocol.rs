use crate::pixel::{LedPixel, Rgb, Rgb16, Rgb16Order, RgbOrder, Rgbw, RgbwOrder};

/// Sealed trait — prevents external crates from implementing `SingleWireProtocol`.
/// New protocol markers must be added in `protocol.rs` alongside the `<pixel, protocol>` impl matrix.
mod private {
    use super::{Sk6812, Ws2811, Ws2812B, Ws2816};

    pub trait ProtocolSealed {}

    impl ProtocolSealed for Ws2812B {}
    impl ProtocolSealed for Ws2811 {}
    impl ProtocolSealed for Sk6812 {}
    impl ProtocolSealed for Ws2816 {}
}

/// Bit transmission order within a byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitOrder {
    /// Most significant bit first (WS2812-family default).
    MsbFirst,
    /// Least significant bit first.
    LsbFirst,
}

/// High/low pulse widths for a single protocol bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PulseTiming {
    /// High-level duration in nanoseconds.
    pub high_ns: u32,
    /// Low-level duration in nanoseconds.
    pub low_ns: u32,
}

/// Describes the wire-level electrical characteristics of a single-wire LED
/// protocol (pulse timings, bit order, reset duration).
///
/// Each protocol is a zero-sized marker type (`Ws2812`, `Sk6812`, …) that
/// implements this trait for the pixel formats it can drive. An incompatible
/// `<pixel, protocol>` pair fails at compile time — no runtime check is needed.
///
/// Constants:
/// - `ZERO`/`ONE`: nominal `(high_ns, low_ns)` for a logical 0 and 1 bit.
/// - `RESET_NS`: minimum low-level time to latch the frame.
/// - `TIMING_TOLERANCE_NS`: symmetric tolerance for SPI encoding plan validation.
///   Each edge must satisfy `|actual − expected| ≤ TIMING_TOLERANCE_NS`.
///   Defaults to 150 ns (WS2812B-class datasheet typical). Override for stricter
///   or looser protocols.
pub trait SingleWireProtocol<P>: private::ProtocolSealed
where
    P: LedPixel,
{
    /// Human-readable protocol name (e.g. "WS2812B").
    const NAME: &'static str;
    /// Bit transmission order within each byte.
    const BIT_ORDER: BitOrder;
    /// Default on-wire color channel order for this protocol.
    const DEFAULT_COLOR_ORDER: P::Order;
    /// Minimum reset (latch) duration in nanoseconds.
    const RESET_NS: u32;
    /// Nominal pulse timing for a logical 0 bit.
    const ZERO: PulseTiming;
    /// Nominal pulse timing for a logical 1 bit.
    const ONE: PulseTiming;

    /// Symmetric timing tolerance for SPI encoding plan validation.
    /// Each edge must satisfy `|actual − expected| ≤ TIMING_TOLERANCE_NS`.
    /// Defaults to 150 ns (common tolerance for WS2812B-class protocols).
    /// Override in a specific impl for protocols with stricter or looser requirements.
    const TIMING_TOLERANCE_NS: u32 = 150;
}

/// Protocol marker for WS2812B (3-channel 8-bit RGB, GRB order, 800 kbps).
#[derive(Debug, Clone, Copy, Default)]
pub struct Ws2812B;

/// Protocol marker for WS2811 (3-channel 8-bit RGB, RGB order, 400 kbps slow mode).
#[derive(Debug, Clone, Copy, Default)]
pub struct Ws2811;

/// Protocol marker for SK6812 (4-channel 8-bit RGBW, GRBW order).
#[derive(Debug, Clone, Copy, Default)]
pub struct Sk6812;

/// Protocol marker for WS2816 (3-channel 16-bit RGB, GRB order).
#[derive(Debug, Clone, Copy, Default)]
pub struct Ws2816;

impl SingleWireProtocol<Rgb> for Ws2812B {
    const NAME: &'static str = "WS2812B";
    const BIT_ORDER: BitOrder = BitOrder::MsbFirst;
    const DEFAULT_COLOR_ORDER: RgbOrder = RgbOrder::Grb;
    const RESET_NS: u32 = 50_000;
    const ZERO: PulseTiming = PulseTiming {
        high_ns: 400,
        low_ns: 850,
    };
    const ONE: PulseTiming = PulseTiming {
        high_ns: 800,
        low_ns: 450,
    };
}

impl SingleWireProtocol<Rgb> for Ws2811 {
    const NAME: &'static str = "WS2811";
    const BIT_ORDER: BitOrder = BitOrder::MsbFirst;
    const DEFAULT_COLOR_ORDER: RgbOrder = RgbOrder::Rgb;
    const RESET_NS: u32 = 50_000;
    // WS2811 uses 400 kbps slow-mode timing (≈ 2.5 µs per bit vs ≈ 1.25 µs for WS2812B).
    // Datasheet: T0H>200 ns, T0L>1300 ns → nominal (500, 2000); T1H>800 ns, T1L>500 ns → nominal (1200, 1300).
    const ZERO: PulseTiming = PulseTiming {
        high_ns: 500,
        low_ns: 2_000,
    };
    const ONE: PulseTiming = PulseTiming {
        high_ns: 1_200,
        low_ns: 1_300,
    };
}

impl SingleWireProtocol<Rgbw> for Sk6812 {
    const NAME: &'static str = "SK6812";
    const BIT_ORDER: BitOrder = BitOrder::MsbFirst;
    const DEFAULT_COLOR_ORDER: RgbwOrder = RgbwOrder::Grbw;
    const RESET_NS: u32 = 80_000;
    const ZERO: PulseTiming = PulseTiming {
        high_ns: 300,
        low_ns: 900,
    };
    const ONE: PulseTiming = PulseTiming {
        high_ns: 600,
        low_ns: 600,
    };
}

impl SingleWireProtocol<Rgb16> for Ws2816 {
    const NAME: &'static str = "WS2816";
    const BIT_ORDER: BitOrder = BitOrder::MsbFirst;
    const DEFAULT_COLOR_ORDER: Rgb16Order = Rgb16Order::Grb;
    // 16-bit channels + color order shift → longer reset latch required.
    const RESET_NS: u32 = 300_000;
    const ZERO: PulseTiming = PulseTiming {
        high_ns: 200,
        low_ns: 600,
    };
    const ONE: PulseTiming = PulseTiming {
        high_ns: 400,
        low_ns: 400,
    };
}
