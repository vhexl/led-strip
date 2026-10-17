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

/// High/low pulse widths for a single protocol bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PulseTiming {
    /// High-level duration in nanoseconds.
    pub high_ns: u32,
    /// Low-level duration in nanoseconds.
    pub low_ns: u32,
}

/// Describes the wire-level electrical characteristics of a single-wire LED
/// protocol (pulse timings, reset duration).
///
/// Each protocol is a zero-sized marker type (`Ws2812B`, `Sk6812`, …) that
/// implements this trait for the pixel formats it can drive. An incompatible
/// `<pixel, protocol>` pair fails at compile time — no runtime check is needed.
///
/// Constants:
/// - `ZERO`/`ONE`: nominal `(high_ns, low_ns)` for a logical 0 and 1 bit.
/// - `RESET_NS`: minimum low-level time to latch the frame.
/// - `ZERO_MIN`/`ZERO_MAX` and `ONE_MIN`/`ONE_MAX`: inclusive timing limits.
///   By default these are derived from the nominal timings and
///   `TIMING_TOLERANCE_NS`; protocols can override each limit independently.
/// - `MIN_BIT_PERIOD_NS`: minimum high-plus-low duration for either bit value.
pub trait SingleWireProtocol<P>: private::ProtocolSealed
where
    P: LedPixel,
{
    /// Human-readable protocol name (e.g. "WS2812B").
    const NAME: &'static str;
    /// Default on-wire color channel order for this protocol.
    const DEFAULT_COLOR_ORDER: P::Order;
    /// Minimum reset (latch) duration in nanoseconds.
    const RESET_NS: u32;
    /// Nominal pulse timing for a logical 0 bit.
    const ZERO: PulseTiming;
    /// Nominal pulse timing for a logical 1 bit.
    const ONE: PulseTiming;

    /// Symmetric tolerance used to derive the default timing limits.
    /// Defaults to 150 ns (common tolerance for WS2812B-class protocols).
    /// Explicit `ZERO_MIN`/`ZERO_MAX` and `ONE_MIN`/`ONE_MAX` overrides take
    /// precedence when a protocol has different limits for each phase.
    const TIMING_TOLERANCE_NS: u32 = 150;

    /// Inclusive minimum high/low durations for a logical 0 bit.
    const ZERO_MIN: PulseTiming = PulseTiming {
        high_ns: Self::ZERO.high_ns.saturating_sub(Self::TIMING_TOLERANCE_NS),
        low_ns: Self::ZERO.low_ns.saturating_sub(Self::TIMING_TOLERANCE_NS),
    };
    /// Inclusive maximum high/low durations for a logical 0 bit.
    const ZERO_MAX: PulseTiming = PulseTiming {
        high_ns: Self::ZERO.high_ns.saturating_add(Self::TIMING_TOLERANCE_NS),
        low_ns: Self::ZERO.low_ns.saturating_add(Self::TIMING_TOLERANCE_NS),
    };
    /// Inclusive minimum high/low durations for a logical 1 bit.
    const ONE_MIN: PulseTiming = PulseTiming {
        high_ns: Self::ONE.high_ns.saturating_sub(Self::TIMING_TOLERANCE_NS),
        low_ns: Self::ONE.low_ns.saturating_sub(Self::TIMING_TOLERANCE_NS),
    };
    /// Inclusive maximum high/low durations for a logical 1 bit.
    const ONE_MAX: PulseTiming = PulseTiming {
        high_ns: Self::ONE.high_ns.saturating_add(Self::TIMING_TOLERANCE_NS),
        low_ns: Self::ONE.low_ns.saturating_add(Self::TIMING_TOLERANCE_NS),
    };
    /// Minimum high-plus-low duration for either bit value, or 0 if unrestricted.
    const MIN_BIT_PERIOD_NS: u32 = 0;
}

/// Protocol marker for WS2812B (3-channel 8-bit RGB, GRB order, 800 kbps).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Ws2812B;

/// Protocol marker for WS2811 (3-channel 8-bit RGB, RGB order, 400 kbps slow mode).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Ws2811;

/// Protocol marker for SK6812 (4-channel 8-bit RGBW, GRBW order).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Sk6812;

/// Protocol marker for the 48-bit WS2816B/WS2816C variants
/// (3-channel 16-bit RGB, GRB order).
///
/// Uses the manufacturer's per-phase limits and a minimum bit period of
/// 1250 ns. Other WS2816 variants may use a different wire format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Ws2816;

impl SingleWireProtocol<Rgb> for Ws2812B {
    const NAME: &'static str = "WS2812B";
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
    const DEFAULT_COLOR_ORDER: Rgb16Order = Rgb16Order::Grb;
    const RESET_NS: u32 = 300_000;
    // Midpoints of the manufacturer's WS2816B/C 48-bit timing ranges.
    const ZERO: PulseTiming = PulseTiming {
        high_ns: 260,
        low_ns: 1_000,
    };
    const ONE: PulseTiming = PulseTiming {
        high_ns: 660,
        low_ns: 740,
    };
    const ZERO_MIN: PulseTiming = PulseTiming {
        high_ns: 200,
        low_ns: 800,
    };
    const ZERO_MAX: PulseTiming = PulseTiming {
        high_ns: 320,
        low_ns: 1_200,
    };
    const ONE_MIN: PulseTiming = PulseTiming {
        high_ns: 520,
        low_ns: 480,
    };
    const ONE_MAX: PulseTiming = PulseTiming {
        high_ns: 800,
        low_ns: 1_000,
    };
    const MIN_BIT_PERIOD_NS: u32 = 1_250;
}
