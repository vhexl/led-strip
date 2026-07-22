use embedded_hal::spi::SpiBus;
use heapless::Vec;

use crate::{
    LedPixel, LedStripConfig, LedStripError, LedStripResult, SingleWireProtocol, TransportBackend,
    WireCodec,
};

/// SPI clock frequency, bit patterns, and symbol width for encoding a
/// single-wire LED protocol over an SPI bus.
///
/// Each protocol bit is encoded as `bits_per_symbol` SPI bits (typically 3 or 4).
/// For example, WS2812B uses 3-bit symbols: `0b100` for logical 0, `0b110` for logical 1.
///
/// Use the predefined constructors ([`ws281x_3bit`](Self::ws281x_3bit),
/// [`sk6812_4bit`](Self::sk6812_4bit)) or build a custom plan with
/// [`new`](Self::new).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiEncodingPlan {
    /// SPI clock frequency in Hz.
    spi_hz: u32,
    /// SPI bit pattern for a logical 0 (MSB-first within the symbol window).
    zero_pattern: u8,
    /// SPI bit pattern for a logical 1 (MSB-first within the symbol window).
    one_pattern: u8,
    /// Number of SPI bits used to encode one protocol bit (e.g. 3 for 0b100/0b110 WS281x).
    bits_per_symbol: u8,
    /// Additional reset extension beyond `Proto::RESET_NS`.
    /// Actual reset = Proto::RESET_NS + extra_reset_ns. Use when the peripheral
    /// (inverter, level shifter) needs extra settling time.
    extra_reset_ns: u32,
}

impl SpiEncodingPlan {
    /// Creates a new encoding plan. Use [`SpiCodec::for_protocol`] to validate
    /// that the plan fits a target protocol's timing tolerances.
    #[must_use]
    pub const fn new(spi_hz: u32, zero_pattern: u8, one_pattern: u8, bits_per_symbol: u8) -> Self {
        Self {
            spi_hz,
            zero_pattern,
            one_pattern,
            bits_per_symbol,
            extra_reset_ns: 0,
        }
    }

    /// 3-bit SPI plan at 2.4 MHz for WS2812 / WS2812B.
    ///
    /// Derived timing (spi_bit ≈ 416 ns, tolerance ±150 ns per WS2812B datasheet V1):
    /// - T0H ≈ 416 ns (expected 400 ns, Δ = 16 ns ✓)
    /// - T0L ≈ 833 ns (expected 850 ns, Δ = 17 ns ✓)
    /// - T1H ≈ 833 ns (expected 800 ns, Δ = 33 ns ✓)
    /// - T1L ≈ 416 ns (expected 450 ns, Δ = 34 ns ✓)
    ///
    /// TX bytes per pixel: 3 ch × 8 bits × 3 SPI bits / 8 = 9 bytes.
    #[must_use]
    pub const fn ws281x_3bit() -> Self {
        Self::new(2_400_000, 0b100, 0b110, 3)
    }

    /// 4-bit SPI plan at 3.2 MHz for SK6812.
    ///
    /// Derived timing (spi_bit ≈ 312 ns, tolerance ±150 ns per SK6812 datasheet):
    /// - T0H ≈ 312 ns (expected 300 ns, Δ = 12 ns ✓)
    /// - T0L ≈ 937 ns (expected 900 ns, Δ = 37 ns ✓)
    /// - T1H ≈ 625 ns (expected 600 ns, Δ = 25 ns ✓)
    /// - T1L ≈ 625 ns (expected 600 ns, Δ = 25 ns ✓)
    ///
    /// TX bytes per pixel: 4 ch × 8 bits × 4 SPI bits / 8 = 16 bytes.
    /// Note: bits_per_symbol = 4; size TX_CAPACITY accordingly (larger than ws281x_3bit).
    #[must_use]
    pub const fn sk6812_4bit() -> Self {
        Self::new(3_200_000, 0b1000, 0b1100, 4)
    }

    /// Appends extra reset time for external signal conditioning.
    #[must_use]
    pub const fn with_extra_reset_ns(mut self, extra_reset_ns: u32) -> Self {
        self.extra_reset_ns = extra_reset_ns;
        self
    }

    #[must_use]
    pub const fn spi_hz(&self) -> u32 {
        self.spi_hz
    }

    #[must_use]
    pub const fn zero_pattern(&self) -> u8 {
        self.zero_pattern
    }

    #[must_use]
    pub const fn one_pattern(&self) -> u8 {
        self.one_pattern
    }

    #[must_use]
    pub const fn bits_per_symbol(&self) -> u8 {
        self.bits_per_symbol
    }

    #[must_use]
    pub const fn extra_reset_ns(&self) -> u32 {
        self.extra_reset_ns
    }
}

/// Identifies one of the four timing edges in a single-wire protocol bit:
/// T0H (zero high), T0L (zero low), T1H (one high), T1L (one low).
///
/// Used by [`SpiCodecPlanError::TimingOutOfTolerance`] to report which
/// edge failed validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimingEdge {
    /// Logical 0, high phase.
    ZeroHigh,
    /// Logical 0, low phase.
    ZeroLow,
    /// Logical 1, high phase.
    OneHigh,
    /// Logical 1, low phase.
    OneLow,
}

impl core::fmt::Display for TimingEdge {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let s = match self {
            Self::ZeroHigh => "T0H",
            Self::ZeroLow => "T0L",
            Self::OneHigh => "T1H",
            Self::OneLow => "T1L",
        };
        f.write_str(s)
    }
}

/// Errors returned when constructing a [`SpiCodec`] or validating an
/// [`SpiEncodingPlan`] against a protocol's timing tolerances.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiCodecPlanError {
    ZeroClock,
    ZeroBitsPerSymbol,
    BitsPerSymbolTooWide {
        bits_per_symbol: u8,
    },
    PatternOutOfRange {
        pattern: u8,
        bits_per_symbol: u8,
    },
    TimingOutOfTolerance {
        edge: TimingEdge,
        actual_ns: u32,
        expected_ns: u32,
        tolerance_ns: u32,
    },
}

impl core::fmt::Display for SpiCodecPlanError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ZeroClock => f.write_str("invalid SPI plan: spi_hz must be non-zero"),
            Self::ZeroBitsPerSymbol => {
                f.write_str("invalid SPI plan: bits_per_symbol must be non-zero")
            }
            Self::BitsPerSymbolTooWide { bits_per_symbol } => write!(
                f,
                "invalid SPI plan: bits_per_symbol={bits_per_symbol} exceeds 8"
            ),
            Self::PatternOutOfRange {
                pattern,
                bits_per_symbol,
            } => write!(
                f,
                "invalid SPI plan: pattern=0x{pattern:02X} out of range for bits_per_symbol={bits_per_symbol}"
            ),
            Self::TimingOutOfTolerance {
                edge,
                actual_ns,
                expected_ns,
                tolerance_ns,
            } => write!(
                f,
                "timing out of tolerance: edge={edge} actual={actual_ns}ns expected={expected_ns}ns tolerance=\u{00B1}{tolerance_ns}ns"
            ),
        }
    }
}

impl core::error::Error for SpiCodecPlanError {}

/// SPI-based wire codec that encodes LED pixel data into a byte stream
/// using configurable bit patterns (bit-banging over SPI MOSI).
///
/// Pre-computes inverted/normal patterns and reset fill bytes at construction
/// time so the hot encoding path is branch-free.
#[derive(Debug)]
pub struct SpiCodec {
    plan: SpiEncodingPlan,
    /// Zero-bit pattern, pre-inverted if `invert_output` was set at construction.
    zero_pattern: u8,
    /// One-bit pattern, pre-inverted if `invert_output` was set at construction.
    one_pattern: u8,
    /// Reset fill byte: 0x00 for normal, 0xFF for inverted output.
    reset_fill: u8,
}

impl SpiCodec {
    /// Constructs a codec from a plan. Validates the plan's structural
    /// parameters (clock, symbol width, pattern range) but does **not**
    /// check protocol timing. Use [`for_protocol`](Self::for_protocol) if
    /// timing validation is desired.
    #[must_use = "SPI codec construction validates the plan; ignoring the result would silently drop errors"]
    pub fn new(plan: SpiEncodingPlan, invert_output: bool) -> Result<Self, SpiCodecPlanError> {
        validate_plan(plan)?;
        Ok(Self::from_plan(plan, invert_output))
    }

    /// Constructs a codec with both structural and protocol timing tolerance validation.
    ///
    /// Preferred over [`Self::new`] when the target protocol is known at construction time.
    /// Returns [`SpiCodecPlanError::TimingOutOfTolerance`] if any of the four timing edges
    /// (T0H, T0L, T1H, T1L) exceeds `Proto::TIMING_TOLERANCE_NS`.
    #[must_use = "returns Err on timing violations; use new() to skip timing validation"]
    pub fn for_protocol<P, Proto>(
        plan: SpiEncodingPlan,
        invert_output: bool,
    ) -> Result<Self, SpiCodecPlanError>
    where
        P: LedPixel,
        Proto: SingleWireProtocol<P>,
    {
        validate_plan(plan)?;
        validate_timing::<P, Proto>(&plan)?;
        Ok(Self::from_plan(plan, invert_output))
    }

    /// Pre-computes inverted/normal patterns and reset fill based on `invert_output`.
    ///
    /// When `invert_output` is true, each pattern bit is flipped within the
    /// `bits_per_symbol`-wide mask window (e.g. 0b100 → 0b011 for 3-bit SPI).
    const fn from_plan(plan: SpiEncodingPlan, invert_output: bool) -> Self {
        if invert_output {
            let mask = pattern_mask(plan.bits_per_symbol());
            Self {
                zero_pattern: (!plan.zero_pattern()) & mask,
                one_pattern: (!plan.one_pattern()) & mask,
                reset_fill: u8::MAX,
                plan,
            }
        } else {
            Self {
                zero_pattern: plan.zero_pattern(),
                one_pattern: plan.one_pattern(),
                reset_fill: 0,
                plan,
            }
        }
    }

    #[must_use]
    pub const fn plan(&self) -> SpiEncodingPlan {
        self.plan
    }

    /// Computes how many SPI bytes of `reset_fill` are needed to satisfy
    /// `Proto::RESET_NS + extra_reset_ns` at the current SPI clock rate.
    ///
    /// Uses ceiling division: `ceil(ns * spi_hz / 1e9)`.
    fn reset_bytes_for<P, Proto>(&self, config: &LedStripConfig<P, Proto>) -> usize
    where
        P: LedPixel,
        Proto: SingleWireProtocol<P>,
    {
        let total_reset_ns = u64::from(config.reset_ns()) + u64::from(self.plan.extra_reset_ns);

        // Fixed-point arithmetic: reset_ns × spi_hz gives scaled clock cycles
        // (in units of 10^9); dividing by 8 × 10^9 converts to SPI bytes in
        // one step (cycles → seconds → bytes).
        let Some(scaled_cycles) = total_reset_ns.checked_mul(u64::from(self.plan.spi_hz)) else {
            // Overflow only at physically impossible parameters
            // (multi-second reset at GHz clocks). Return a value that will
            // fail the capacity check in `new()`.
            return usize::MAX;
        };

        let total_bytes =
            scaled_cycles.saturating_add(8_000_000_000_u64 - 1) / 8_000_000_000_u64;

        usize::try_from(total_bytes).unwrap_or(usize::MAX)
    }
}

impl<P, Proto> WireCodec<P, Proto, u8> for SpiCodec
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
{
    type Error = SpiCodecPlanError;
    type EncodeError = core::convert::Infallible;

    fn encoded_len(&self, config: &LedStripConfig<P, Proto>) -> usize {
        let Some(payload_bits) = config
            .frame_len_bytes()
            .checked_mul(8)
            .and_then(|v| v.checked_mul(usize::from(self.plan.bits_per_symbol)))
        else {
            // Overflow → return a value that will fail the capacity check in `new()`.
            // In practice this path is unreachable on 32-bit+ platforms for any real
            // LED strip (would require billions of pixels).
            return usize::MAX;
        };

        let payload_bytes = payload_bits.saturating_add(7) / 8;
        let total = payload_bytes.saturating_add(self.reset_bytes_for(config));

        // Saturating arithmetic on a 32-bit (or wider) usize only triggers at
        // millions of LEDs — far beyond any real strip.  Still, if it ever
        // saturates, the capacity check above would silently pass.
        debug_assert!(
            total < usize::MAX / 2,
            "encoded_len overflowed: frame too large for usize"
        );

        total
    }

    fn encode<const TX_CAPACITY: usize>(
        &self,
        config: &LedStripConfig<P, Proto>,
        pixels: &[P],
        out: &mut Vec<u8, TX_CAPACITY>,
    ) -> LedStripResult<(), Self::EncodeError> {
        if pixels.len() != config.len() {
            return Err(LedStripError::InvalidLength {
                expected: config.len(),
                actual: pixels.len(),
            });
        }

        let required = self.encoded_len(config);
        if required > TX_CAPACITY {
            return Err(LedStripError::BufferTooSmall {
                required,
                capacity: TX_CAPACITY,
            });
        }

        out.clear();

        let mut current_byte = 0_u8;
        let mut used_bits = 0_u8;
        let mut raw = [0_u8; 6];
        for pixel in pixels.iter().copied() {
            pixel.encode(config.color_order(), &mut raw[..P::BYTES_PER_PIXEL]);

            for byte in &raw[..P::BYTES_PER_PIXEL] {
                for bit_index in 0..8 {
                    let shift = bit_shift::<P, Proto>(bit_index);
                    let bit_is_set = ((*byte >> shift) & 1) != 0;
                    let pattern = if bit_is_set {
                        self.one_pattern
                    } else {
                        self.zero_pattern
                    };

                    append_pattern::<TX_CAPACITY>(
                        out,
                        pattern,
                        self.plan.bits_per_symbol(),
                        &mut current_byte,
                        &mut used_bits,
                    )?;
                }
            }
        }

        if used_bits != 0 {
            if self.reset_fill != 0 {
                current_byte |= u8::MAX >> used_bits;
            }

            out.push(current_byte)
                .map_err(|_| LedStripError::BufferTooSmall {
                    required,
                    capacity: TX_CAPACITY,
                })?;
        }

        let reset_bytes = self.reset_bytes_for(config);
        let target_len = out.len().saturating_add(reset_bytes);
        out.resize(target_len, self.reset_fill)
            .map_err(|_| LedStripError::BufferTooSmall {
                required,
                capacity: TX_CAPACITY,
            })?;

        Ok(())
    }
}

/// SPI transport backend.
///
/// Wraps an [`embedded_hal::spi::SpiBus`] implementor. The codec handles all
/// protocol-level concerns (timing, bit patterns, reset); this backend only
/// pushes the resulting byte stream over MOSI.
#[derive(Debug)]
pub struct SpiBackend<SPI> {
    spi: SPI,
}

impl<SPI> SpiBackend<SPI> {
    /// Wraps an SPI peripheral.
    #[must_use]
    pub const fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Consumes the backend, returning the underlying SPI peripheral.
    #[must_use]
    pub fn into_inner(self) -> SPI {
        self.spi
    }

    /// Returns a shared reference to the SPI peripheral.
    #[must_use]
    pub fn inner(&self) -> &SPI {
        &self.spi
    }

    /// Returns a mutable reference to the SPI peripheral.
    #[must_use]
    pub fn inner_mut(&mut self) -> &mut SPI {
        &mut self.spi
    }
}

impl<SPI> TransportBackend for SpiBackend<SPI>
where
    SPI: SpiBus<u8>,
{
    type Word = u8;
    type Error = SPI::Error;

    fn transmit(&mut self, words: &[Self::Word]) -> Result<(), Self::Error> {
        self.spi.write(words)
    }
}

/// Returns the bitmask covering `bits_per_symbol` low bits.
///
/// For `bits_per_symbol == 8` this is `0xFF`; otherwise `(1 << n) - 1`.
const fn pattern_mask(bits_per_symbol: u8) -> u8 {
    if bits_per_symbol == 8 {
        u8::MAX
    } else {
        (1_u8 << bits_per_symbol) - 1
    }
}

fn validate_plan(plan: SpiEncodingPlan) -> Result<(), SpiCodecPlanError> {
    if plan.spi_hz == 0 {
        return Err(SpiCodecPlanError::ZeroClock);
    }

    if plan.bits_per_symbol == 0 {
        return Err(SpiCodecPlanError::ZeroBitsPerSymbol);
    }

    if plan.bits_per_symbol > 8 {
        return Err(SpiCodecPlanError::BitsPerSymbolTooWide {
            bits_per_symbol: plan.bits_per_symbol,
        });
    }

    let limit = pattern_mask(plan.bits_per_symbol);

    if plan.zero_pattern > limit {
        return Err(SpiCodecPlanError::PatternOutOfRange {
            pattern: plan.zero_pattern,
            bits_per_symbol: plan.bits_per_symbol,
        });
    }

    if plan.one_pattern > limit {
        return Err(SpiCodecPlanError::PatternOutOfRange {
            pattern: plan.one_pattern,
            bits_per_symbol: plan.bits_per_symbol,
        });
    }

    Ok(())
}

/// Returns the bit position within a pixel byte for the given logical bit index,
/// respecting the protocol's `BIT_ORDER`.
///
/// With `MsbFirst`, bit 0 → position 7, bit 1 → position 6, …
/// With `LsbFirst`, bit 0 → position 0, bit 1 → position 1, …
fn bit_shift<P, Proto>(bit_index: u8) -> u8
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
{
    match Proto::BIT_ORDER {
        crate::BitOrder::MsbFirst => 7 - bit_index,
        crate::BitOrder::LsbFirst => bit_index,
    }
}

/// Stream-oriented SPI bit-packing kernel.
///
/// Appends `bits_per_symbol` bits of `pattern` (MSB-first within the pattern)
/// into the SPI output buffer. Bits are accumulated into `current_byte` from
/// MSB to LSB; when the byte is full it is pushed and a new byte starts.
///
/// The pattern should already be inverted if the output line uses an external
/// inverter (pre-computed by `SpiCodec::from_plan`).
///
/// This is the hot path during `refresh`. The capacity check was already
/// performed at the top of `encode()`, so push failures here indicate a
/// logic error in `encoded_len` — the error path is cold but provides a
/// safe panic-free fallback instead of UB.
#[inline]
fn append_pattern<const TX_CAPACITY: usize>(
    out: &mut Vec<u8, TX_CAPACITY>,
    pattern: u8,
    bits_per_symbol: u8,
    current_byte: &mut u8,
    used_bits: &mut u8,
) -> LedStripResult<(), core::convert::Infallible> {
    for shift in (0..bits_per_symbol).rev() {
        let bit = ((pattern >> shift) & 1) != 0;

        if bit {
            *current_byte |= 1_u8 << (7 - *used_bits);
        }

        *used_bits += 1;
        if *used_bits == 8 {
            out.push(*current_byte).map_err(|_| LedStripError::BufferTooSmall {
                required: TX_CAPACITY.saturating_add(1),
                capacity: TX_CAPACITY,
            })?;

            *current_byte = 0;
            *used_bits = 0;
        }
    }
    Ok(())
}

/// Counts leading 1-bits from the MSB within the `bits_per_symbol` window.
///
/// Assumes the pattern has the form `(1≥1)(0≥0)`. Non-consecutive patterns
/// (e.g. `0b1010`) are under-counted — a limitation of the simple
/// leading-ones heuristic. Use only with well-formed patterns like `0b100` or
/// `0b1100` where 1-bits are contiguous from the MSB.
fn leading_ones_in_symbol(pattern: u8, bits_per_symbol: u8) -> u8 {
    let mut count = 0_u8;
    for i in (0..bits_per_symbol).rev() {
        if (pattern >> i) & 1 == 1 {
            count += 1;
        } else {
            break;
        }
    }
    count
}

/// Derives actual `T0H/T0L/T1H/T1L` from `plan` and compares against
/// `Proto::ZERO/ONE` using the symmetric `TIMING_TOLERANCE_NS` band.
/// Each edge must satisfy `|actual − expected| ≤ tolerance`.
///
/// Timing derivation:
///   `spi_bit_ns = 10^9 / spi_hz`
///   `T0H = leading_ones(zero_pattern) × spi_bit_ns`
///   `T0L = (bits_per_symbol - T0H_bits) × spi_bit_ns`
///   … same for T1H/T1L.
fn validate_timing<P, Proto>(plan: &SpiEncodingPlan) -> Result<(), SpiCodecPlanError>
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
{
    // Integer division is safe: validate_plan already ensures spi_hz != 0.
    let spi_bit_ns = 1_000_000_000_u32 / plan.spi_hz;
    let tolerance = Proto::TIMING_TOLERANCE_NS;

    let zero_high = leading_ones_in_symbol(plan.zero_pattern, plan.bits_per_symbol);
    let zero_low = plan.bits_per_symbol - zero_high;
    let one_high = leading_ones_in_symbol(plan.one_pattern, plan.bits_per_symbol);
    let one_low = plan.bits_per_symbol - one_high;

    check_timing(
        u32::from(zero_high) * spi_bit_ns,
        Proto::ZERO.high_ns,
        tolerance,
        TimingEdge::ZeroHigh,
    )?;
    check_timing(
        u32::from(zero_low) * spi_bit_ns,
        Proto::ZERO.low_ns,
        tolerance,
        TimingEdge::ZeroLow,
    )?;
    check_timing(
        u32::from(one_high) * spi_bit_ns,
        Proto::ONE.high_ns,
        tolerance,
        TimingEdge::OneHigh,
    )?;
    check_timing(
        u32::from(one_low) * spi_bit_ns,
        Proto::ONE.low_ns,
        tolerance,
        TimingEdge::OneLow,
    )?;

    Ok(())
}

fn check_timing(
    actual_ns: u32,
    expected_ns: u32,
    tolerance_ns: u32,
    edge: TimingEdge,
) -> Result<(), SpiCodecPlanError> {
    if actual_ns.abs_diff(expected_ns) > tolerance_ns {
        return Err(SpiCodecPlanError::TimingOutOfTolerance {
            edge,
            actual_ns,
            expected_ns,
            tolerance_ns,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::{Rgb, Rgb16, Rgbw, Sk6812, SpiBackend, TransportBackend, WireCodec, Ws2812B};

    use super::{SpiCodec, SpiCodecPlanError, SpiEncodingPlan, TimingEdge};

    #[test]
    fn ws281x_3bit_passes_ws2812b_timing() {
        SpiCodec::for_protocol::<Rgb, Ws2812B>(SpiEncodingPlan::ws281x_3bit(), false).unwrap();
    }

    #[test]
    fn sk6812_4bit_passes_sk6812_timing() {
        SpiCodec::for_protocol::<Rgbw, Sk6812>(SpiEncodingPlan::sk6812_4bit(), false).unwrap();
    }

    #[test]
    fn ws281x_3bit_rejects_sk6812_timing() {
        // T1H: actual ≈ 832 ns, expected = 600 ns, Δ = 232 ns > 150 ns tolerance.
        let err = SpiCodec::for_protocol::<Rgbw, Sk6812>(SpiEncodingPlan::ws281x_3bit(), false)
            .unwrap_err();
        assert!(matches!(
            err,
            SpiCodecPlanError::TimingOutOfTolerance {
                edge: TimingEdge::OneHigh,
                ..
            }
        ));
    }

    #[test]
    fn out_of_tolerance_plan_reports_first_failing_edge() {
        // 1 MHz 3-bit: spi_bit = 1000 ns, T0H = 1000 ns vs WS2812B T0H = 400 ns, Δ = 600 ns.
        let plan = SpiEncodingPlan::new(1_000_000, 0b100, 0b110, 3);
        let err = SpiCodec::for_protocol::<Rgb, Ws2812B>(plan, false).unwrap_err();
        assert_eq!(
            err,
            SpiCodecPlanError::TimingOutOfTolerance {
                edge: TimingEdge::ZeroHigh,
                actual_ns: 1000,
                expected_ns: 400,
                tolerance_ns: 150,
            }
        );
    }

    #[test]
    fn timing_edge_display_uses_protocol_labels() {
        assert_eq!(TimingEdge::ZeroHigh.to_string(), "T0H");
        assert_eq!(TimingEdge::ZeroLow.to_string(), "T0L");
        assert_eq!(TimingEdge::OneHigh.to_string(), "T1H");
        assert_eq!(TimingEdge::OneLow.to_string(), "T1L");
    }

    #[test]
    fn plan_error_display_zero_clock() {
        let s = SpiCodecPlanError::ZeroClock.to_string();
        assert!(s.contains("spi_hz"), "{s}");
    }

    #[test]
    fn plan_error_display_zero_bits_per_symbol() {
        let s = SpiCodecPlanError::ZeroBitsPerSymbol.to_string();
        assert!(s.contains("bits_per_symbol"), "{s}");
    }

    #[test]
    fn plan_error_display_bits_per_symbol_too_wide() {
        let s = SpiCodecPlanError::BitsPerSymbolTooWide { bits_per_symbol: 9 }.to_string();
        assert!(s.contains("bits_per_symbol=9"), "{s}");
    }

    #[test]
    fn plan_error_display_pattern_out_of_range() {
        let s = SpiCodecPlanError::PatternOutOfRange {
            pattern: 0xFF,
            bits_per_symbol: 3,
        }
        .to_string();
        assert!(s.contains("0xFF"), "{s}");
        assert!(s.contains("bits_per_symbol=3"), "{s}");
    }

    #[test]
    fn plan_error_display_timing_out_of_tolerance_includes_all_fields() {
        let s = SpiCodecPlanError::TimingOutOfTolerance {
            edge: TimingEdge::OneHigh,
            actual_ns: 832,
            expected_ns: 600,
            tolerance_ns: 150,
        }
        .to_string();
        assert!(s.contains("T1H"), "{s}");
        assert!(s.contains("832"), "{s}");
        assert!(s.contains("600"), "{s}");
        assert!(s.contains("150"), "{s}");
    }

    #[test]
    fn plan_error_is_core_error() {
        let _: &dyn core::error::Error = &SpiCodecPlanError::ZeroClock;
    }

    // ── Encode end-to-end tests ──────────────────────────────────────

    use crate::{LedStripConfig, LedStripError};
    use heapless::Vec as HVec;

    #[test]
    fn encode_produces_nonempty_buffer() {
        let codec =
            SpiCodec::for_protocol::<Rgb, Ws2812B>(SpiEncodingPlan::ws281x_3bit(), false).unwrap();
        let config = LedStripConfig::ws2812b(1);
        let pixels = [Rgb::new(0, 255, 0)];
        let mut out: HVec<u8, 64> = HVec::new();

        codec.encode(&config, &pixels, &mut out).unwrap();

        assert!(!out.is_empty(), "encoded output should not be empty");
        // ws281x_3bit: 3 ch × 8 bits × 3 spi_bits / 8 = 9 payload bytes + reset
        assert!(out.len() >= 9, "expected >=9 bytes, got {}", out.len());
    }

    #[test]
    fn encode_output_len_matches_encoded_len() {
        let codec =
            SpiCodec::for_protocol::<Rgb, Ws2812B>(SpiEncodingPlan::ws281x_3bit(), false).unwrap();
        let config = LedStripConfig::ws2812b(3);
        let pixels = [
            Rgb::new(255, 0, 0),
            Rgb::new(0, 255, 0),
            Rgb::new(0, 0, 255),
        ];
        let mut out: HVec<u8, 256> = HVec::new();

        codec.encode(&config, &pixels, &mut out).unwrap();

        assert_eq!(
            out.len(),
            codec.encoded_len(&config),
            "output len must match encoded_len prediction"
        );
    }

    #[test]
    fn encode_rejects_length_mismatch() {
        let codec =
            SpiCodec::for_protocol::<Rgb, Ws2812B>(SpiEncodingPlan::ws281x_3bit(), false).unwrap();
        let config = LedStripConfig::ws2812b(2);
        let pixels = [Rgb::new(0, 0, 0)];
        let mut out: HVec<u8, 64> = HVec::new();

        let err = codec.encode(&config, &pixels, &mut out).unwrap_err();
        assert!(matches!(err, LedStripError::InvalidLength { .. }));
    }

    #[test]
    fn encode_rejects_buffer_too_small() {
        let codec =
            SpiCodec::for_protocol::<Rgb, Ws2812B>(SpiEncodingPlan::ws281x_3bit(), false).unwrap();
        let config = LedStripConfig::ws2812b(2);
        let pixels = [Rgb::new(0, 0, 0), Rgb::new(0, 0, 0)];
        // 2 pixels * 9 bytes = 18 + reset_bytes → need > 30, give only 5
        let mut out: HVec<u8, 5> = HVec::new();

        let err = codec.encode(&config, &pixels, &mut out).unwrap_err();
        assert!(matches!(err, LedStripError::BufferTooSmall { .. }));
    }

    // ── invert_output tests ──────────────────────────────────────────

    #[test]
    fn inverted_codec_precomputes_patterns_correctly() {
        let plan = SpiEncodingPlan::ws281x_3bit(); // 0b100, 0b110, 3 bits/symbol
        let codec = SpiCodec::for_protocol::<Rgb, Ws2812B>(plan, true).unwrap();

        // In 3-bit window (mask=0b111): !0b100 = 0b011, !0b110 = 0b001
        assert_eq!(codec.zero_pattern, 0b011);
        assert_eq!(codec.one_pattern, 0b001);
        assert_eq!(codec.reset_fill, 0xFF);
    }

    #[test]
    fn invert_output_produces_different_data_and_reset() {
        let plan = SpiEncodingPlan::ws281x_3bit();
        let codec_normal = SpiCodec::for_protocol::<Rgb, Ws2812B>(plan, false).unwrap();
        let codec_inverted = SpiCodec::for_protocol::<Rgb, Ws2812B>(plan, true).unwrap();
        let config = LedStripConfig::ws2812b(1);
        let pixels = [Rgb::new(255, 0, 0)];
        let mut out_normal: HVec<u8, 128> = HVec::new();
        let mut out_inverted: HVec<u8, 128> = HVec::new();

        codec_normal
            .encode(&config, &pixels, &mut out_normal)
            .unwrap();
        codec_inverted
            .encode(&config, &pixels, &mut out_inverted)
            .unwrap();

        // Same length
        assert_eq!(out_normal.len(), out_inverted.len());
        // Different content (inverted data bits)
        assert_ne!(out_normal, out_inverted);
        // Reset fill bytes differ
        let last_normal = *out_normal.last().unwrap();
        let last_inverted = *out_inverted.last().unwrap();
        assert_eq!(last_normal, 0x00, "normal reset fill should be 0x00");
        assert_eq!(last_inverted, 0xFF, "inverted reset fill should be 0xFF");
    }

    // ── Timing validation boundary tests ─────────────────────────────

    #[test]
    fn validate_timing_reports_first_failing_edge() {
        // 4 MHz 3-bit: spi_bit=250 ns
        // T0H=250 ns (Δ=150, at tolerance ✓), T0L=500 ns (Δ=350, >150 ✗)
        let plan = SpiEncodingPlan::new(4_000_000, 0b100, 0b110, 3);
        let err = SpiCodec::for_protocol::<Rgb, Ws2812B>(plan, false).unwrap_err();
        assert!(matches!(
            err,
            SpiCodecPlanError::TimingOutOfTolerance {
                edge: TimingEdge::ZeroLow,
                ..
            }
        ));
    }

    #[test]
    fn timing_exactly_at_tolerance_boundary_passes() {
        // 2.5 MHz 3-bit: spi_bit=400 ns
        // T0H=400 ns (Δ=0 ≤ 150 ✓), T0L=800 ns (Δ=50 ≤ 150 ✓)
        // T1H=800 ns (Δ=0 ≤ 150 ✓), T1L=400 ns (Δ=50 ≤ 150 ✓)
        let plan = SpiEncodingPlan::new(2_500_000, 0b100, 0b110, 3);
        SpiCodec::for_protocol::<Rgb, Ws2812B>(plan, false).unwrap();
    }

    // ── SpiEncodingPlan getters / extra_reset ───────────────────────

    #[test]
    fn plan_getters_return_construction_values() {
        let plan = SpiEncodingPlan::new(3_000_000, 0b1010, 0b1110, 4);
        assert_eq!(plan.spi_hz(), 3_000_000);
        assert_eq!(plan.zero_pattern(), 0b1010);
        assert_eq!(plan.one_pattern(), 0b1110);
        assert_eq!(plan.bits_per_symbol(), 4);
        assert_eq!(plan.extra_reset_ns(), 0);
    }

    #[test]
    fn plan_with_extra_reset_ns_stores_value() {
        let plan = SpiEncodingPlan::ws281x_3bit().with_extra_reset_ns(10_000);
        assert_eq!(plan.extra_reset_ns(), 10_000);
    }

    // ── SpiCodec::new (no timing, structural only) ──────────────────

    #[test]
    fn spi_codec_new_with_valid_plan_succeeds() {
        SpiCodec::new(SpiEncodingPlan::ws281x_3bit(), false).unwrap();
    }

    #[test]
    fn spi_codec_new_with_inverted_output_succeeds() {
        SpiCodec::new(SpiEncodingPlan::ws281x_3bit(), true).unwrap();
    }

    #[test]
    fn spi_codec_new_rejects_zero_clock() {
        let plan = SpiEncodingPlan::new(0, 0b100, 0b110, 3);
        let err = SpiCodec::new(plan, false).unwrap_err();
        assert_eq!(err, SpiCodecPlanError::ZeroClock);
    }

    #[test]
    fn spi_codec_plan_accessor_returns_original_plan() {
        let plan = SpiEncodingPlan::ws281x_3bit();
        let codec = SpiCodec::new(plan, false).unwrap();
        assert_eq!(codec.plan(), plan);
    }

    // ── SpiBackend ───────────────────────────────────────────────────

    #[test]
    fn spi_backend_new() {
        let backend = SpiBackend::new(42_u32);
        assert_eq!(*backend.inner(), 42);
    }

    #[test]
    fn spi_backend_into_inner() {
        let backend = SpiBackend::new("hello");
        assert_eq!(backend.into_inner(), "hello");
    }

    #[test]
    fn spi_backend_inner_mut_modifies_value() {
        let mut backend = SpiBackend::new(10_u32);
        *backend.inner_mut() = 99;
        assert_eq!(*backend.inner(), 99);
    }

    // ── Mock SPI transmit ────────────────────────────────────────────

    use embedded_hal::spi::SpiBus;
    use core::cell::Cell;

    #[derive(Debug, Default)]
    struct MockSpi(Cell<usize>);

    impl embedded_hal::spi::ErrorType for MockSpi {
        type Error = core::convert::Infallible;
    }

    impl SpiBus<u8> for MockSpi {
        fn read(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.0.set(words.len());
            Ok(())
        }
        fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> { Ok(()) }
        fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
        fn flush(&mut self) -> Result<(), Self::Error> { Ok(()) }
    }

    #[test]
    fn transmit_delegates_to_spi_bus_write() {
        let mut backend = SpiBackend::new(MockSpi::default());
        let data = [1_u8, 2, 3, 4, 5];
        backend.transmit(&data).unwrap();
        assert_eq!(backend.inner().0.get(), 5);
    }

    // ── Other protocol encodings ─────────────────────────────────────

    use crate::Ws2816;

    #[test]
    fn encode_sk6812_produces_correct_length() {
        let codec = SpiCodec::for_protocol::<Rgbw, Sk6812>(SpiEncodingPlan::sk6812_4bit(), false)
            .unwrap();
        let config = LedStripConfig::sk6812(2);
        let pixels = [Rgbw::new(0, 0, 0, 0), Rgbw::new(0, 0, 0, 0)];
        let mut out: HVec<u8, 256> = HVec::new();
        codec.encode(&config, &pixels, &mut out).unwrap();
        assert_eq!(out.len(), codec.encoded_len(&config));
    }

    #[test]
    fn encode_ws2811_produces_correct_length() {
        // WS2811 wide 8-bit symbols — use SpiCodec::new without timing validation.
        let plan = SpiEncodingPlan::new(4_800_000, 0b11100000, 0b11111110, 8);
        let codec = SpiCodec::new(plan, false).unwrap();
        let config = LedStripConfig::ws2811(2);
        let pixels = [Rgb::new(0, 0, 0), Rgb::new(0, 0, 0)];
        let mut out: HVec<u8, 256> = HVec::new();
        codec.encode(&config, &pixels, &mut out).unwrap();
        assert_eq!(out.len(), codec.encoded_len(&config));
    }

    #[test]
    fn encode_ws2816_produces_correct_length() {
        // 4 MHz, 4-bit: spi_bit=250, all edges within ±150 ns of WS2816 spec.
        let plan = SpiEncodingPlan::new(4_000_000, 0b1000, 0b1100, 4);
        let codec = SpiCodec::for_protocol::<Rgb16, Ws2816>(plan, false).unwrap();
        let config = LedStripConfig::ws2816(1);
        let pixels = [Rgb16::new(0, 0, 0)];
        let mut out: HVec<u8, 256> = HVec::new();
        codec.encode(&config, &pixels, &mut out).unwrap();
        assert_eq!(out.len(), codec.encoded_len(&config));
    }

    // ── Extra reset_ns ───────────────────────────────────────────────

    #[test]
    fn extra_reset_ns_increases_encoded_len() {
        let plan_normal = SpiEncodingPlan::ws281x_3bit();
        let plan_extra = SpiEncodingPlan::ws281x_3bit().with_extra_reset_ns(100_000);
        let codec_normal =
            SpiCodec::for_protocol::<Rgb, Ws2812B>(plan_normal, false).unwrap();
        let codec_extra =
            SpiCodec::for_protocol::<Rgb, Ws2812B>(plan_extra, false).unwrap();
        let config = LedStripConfig::ws2812b(1);
        let len_normal = codec_normal.encoded_len(&config);
        let len_extra = codec_extra.encoded_len(&config);
        assert!(len_extra > len_normal, "extra reset should increase encoded length");
    }

    // ── Plan validation edge cases ───────────────────────────────────

    #[test]
    fn validate_plan_rejects_zero_bits_per_symbol() {
        let plan = SpiEncodingPlan::new(2_400_000, 0b100, 0b110, 0);
        let err = SpiCodec::new(plan, false).unwrap_err();
        assert_eq!(err, SpiCodecPlanError::ZeroBitsPerSymbol);
    }

    #[test]
    fn validate_plan_rejects_bits_per_symbol_too_wide() {
        let plan = SpiEncodingPlan::new(2_400_000, 0b1, 0b1, 9);
        let err = SpiCodec::new(plan, false).unwrap_err();
        assert_eq!(
            err,
            SpiCodecPlanError::BitsPerSymbolTooWide {
                bits_per_symbol: 9,
            }
        );
    }

    #[test]
    fn validate_plan_rejects_zero_pattern_out_of_range() {
        let plan = SpiEncodingPlan::new(2_400_000, 0b1111, 0b110, 3);
        let err = SpiCodec::new(plan, false).unwrap_err();
        assert!(matches!(
            err,
            SpiCodecPlanError::PatternOutOfRange {
                pattern: 0b1111,
                bits_per_symbol: 3,
            }
        ));
    }

    #[test]
    fn validate_plan_rejects_one_pattern_out_of_range() {
        let plan = SpiEncodingPlan::new(2_400_000, 0b100, 0b1111, 3);
        let err = SpiCodec::new(plan, false).unwrap_err();
        assert!(matches!(
            err,
            SpiCodecPlanError::PatternOutOfRange {
                pattern: 0b1111,
                bits_per_symbol: 3,
            }
        ));
    }
}
