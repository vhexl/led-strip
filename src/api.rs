use core::convert::Infallible;

use heapless::Vec;

use crate::{
    FrameBuf, FrameError, LedPixel, LedStripConfig, LedStripError, LedStripResult,
    SingleWireProtocol,
};

/// Unified error type returned by [`LedStrip::refresh`] and [`LedStrip::clear`].
///
/// Wraps either a codec-level error (encoding failure) or a backend-level
/// error (transmission failure), so callers see a single error shape
/// regardless of which layer failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefreshError<CodecError, BackendError> {
    /// The codec failed to encode the frame into transport words.
    Codec(CodecError),
    /// The transport backend failed to transmit the encoded words.
    Backend(BackendError),
}

impl<CodecError, BackendError> core::fmt::Display for RefreshError<CodecError, BackendError>
where
    CodecError: core::fmt::Display,
    BackendError: core::fmt::Display,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Codec(inner) => write!(f, "codec error: {inner}"),
            Self::Backend(inner) => write!(f, "backend error: {inner}"),
        }
    }
}

impl<CodecError, BackendError> core::error::Error for RefreshError<CodecError, BackendError>
where
    CodecError: core::error::Error + 'static,
    BackendError: core::error::Error + 'static,
{
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Codec(inner) => Some(inner),
            Self::Backend(inner) => Some(inner),
        }
    }
}

/// Low-level transport for encoded LED data.
///
/// A backend owns a hardware peripheral (SPI bus, or a future peripheral such
/// as an RMT channel or PIO state machine) and knows how to push pre-encoded
/// words onto the wire. It does **not** understand LED protocol timings —
/// that is the codec's job.
///
/// # Implementing
///
/// Backends are stateless beyond the peripheral handle. [`transmit`](Self::transmit)
/// should block until the entire buffer has been sent, then return.
/// Reset/latch timing is handled by the codec (appended as trailing fill bytes).
pub trait TransportBackend {
    /// The unit of transmission. SPI uses `u8`; future backends (RMT, PIO)
    /// may use `u32`.
    type Word: Copy;
    /// Backend-specific error (e.g. `SPI::Error`).
    type Error;

    /// Sends `words` over the wire. Blocks until transmission completes.
    fn transmit(&mut self, words: &[Self::Word]) -> Result<(), Self::Error>;
}

/// Encodes LED pixel data into transport words for a specific protocol.
///
/// A codec understands a particular encoding scheme (SPI bit-banging, or
/// future schemes like RMT pulse codes or PIO FIFO words) and translates raw
/// pixel bytes into the word stream that a [`TransportBackend`] can transmit.
///
/// The generic parameters lock a `<pixel, protocol, codec>` triple at compile
/// time — you cannot accidentally drive SK6812 pixels with a WS2812B encoding plan.
///
/// # Implementing
///
/// - [`encoded_len`](Self::encoded_len) must return the exact number of
///   `Word`s that [`encode`](Self::encode) will produce (including
///   reset/trailer). This is used for capacity checks before encoding.
/// - [`encode`](Self::encode) writes into a caller-supplied `Vec` to avoid
///   heap allocation.
pub trait WireCodec<P, Proto, Word>
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
    Word: Copy,
{
    /// Codec-level error type (construction, validation, etc.).
    type Error;
    /// Error type produced specifically by [`encode`](Self::encode).
    ///
    /// Separated from [`Error`](Self::Error) because some codecs (e.g.
    /// [`SpiCodec`](crate::SpiCodec)) never fail during encoding — the
    /// only possible errors are structural (`InvalidLength`,
    /// `BufferTooSmall`).
    type EncodeError;

    /// Returns the exact number of `Word`s that [`encode`](Self::encode) will
    /// produce for the given configuration. Includes payload + reset/trailer overhead.
    fn encoded_len(&self, config: &LedStripConfig<P, Proto>) -> usize;

    /// Encodes `pixels` into transport words, appending to `out`.
    ///
    /// The caller must ensure `out` has enough capacity (use [`encoded_len`](Self::encoded_len)).
    /// The buffer is **not** cleared before encoding — callers that reuse a
    /// buffer should call `out.clear()` first.
    fn encode<const TX_CAPACITY: usize>(
        &self,
        config: &LedStripConfig<P, Proto>,
        pixels: &[P],
        out: &mut Vec<Word, TX_CAPACITY>,
    ) -> LedStripResult<(), Self::EncodeError>;
}

/// High-level driver for a single-wire addressable LED strip.
///
/// Owns the configuration, frame buffer, codec, and transport backend.
/// This is the primary API entry point — construct one, then call
/// [`set`](Self::set) / [`write`](Self::write) / [`fill`](Self::fill) to
/// update pixels and [`refresh`](Self::refresh) to push the frame to the strip.
///
/// # Type parameters
///
/// | Parameter | Role |
/// |-----------|------|
/// | `P` | Pixel type ([`crate::Rgb`], [`crate::Rgbw`], [`crate::Rgb16`]) |
/// | `Proto` | Protocol marker ([`crate::Ws2812B`], [`crate::Sk6812`], [`crate::Ws2811`], [`crate::Ws2816`]) |
/// | `Codec` | Encoding scheme (currently [`crate::SpiCodec`]; future backends may provide their own) |
/// | `Backend` | Transport layer (currently [`crate::SpiBackend`]; future backends may provide their own) |
/// | `MAX_LEDS` | Compile-time upper bound on pixel count |
/// | `TX_CAPACITY` | Compile-time upper bound on transport buffer size (in `Word`s) |
///
/// # Capacity validation
///
/// All capacity checks happen once in [`new`](Self::new). If it returns `Ok`,
/// subsequent [`refresh`](Self::refresh) / [`set`](Self::set) /
/// [`write`](Self::write) calls will never fail with `BufferTooSmall`.
pub struct LedStrip<P, Proto, Codec, Backend, const MAX_LEDS: usize, const TX_CAPACITY: usize>
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
    Codec: WireCodec<P, Proto, Backend::Word>,
    Backend: TransportBackend,
{
    config: LedStripConfig<P, Proto>,
    frame: FrameBuf<P, MAX_LEDS>,
    codec: Codec,
    backend: Backend,
    tx_buf: Vec<Backend::Word, TX_CAPACITY>,
}

impl<P, Proto, Codec, Backend, const MAX_LEDS: usize, const TX_CAPACITY: usize>
    LedStrip<P, Proto, Codec, Backend, MAX_LEDS, TX_CAPACITY>
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
    Codec: WireCodec<P, Proto, Backend::Word>,
    Backend: TransportBackend,
{
    /// Constructs the driver and validates all capacity constraints once.
    ///
    /// Capacity errors (`BufferTooSmall`) are only returned here — once `new`
    /// succeeds, `refresh`/`set`/`write`/`clear` will never report capacity
    /// failures on the hot path.
    pub fn new(
        config: LedStripConfig<P, Proto>,
        codec: Codec,
        backend: Backend,
    ) -> LedStripResult<Self, Infallible> {
        if config.len() > MAX_LEDS {
            return Err(LedStripError::BufferTooSmall {
                required: config.len(),
                capacity: MAX_LEDS,
            });
        }

        let required = codec.encoded_len(&config);
        if required > TX_CAPACITY {
            return Err(LedStripError::BufferTooSmall {
                required,
                capacity: TX_CAPACITY,
            });
        }

        Ok(Self {
            frame: FrameBuf::from_config(&config)?,
            config,
            codec,
            backend,
            tx_buf: Vec::new(),
        })
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.frame.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.frame.is_empty()
    }

    #[must_use]
    pub const fn config(&self) -> &LedStripConfig<P, Proto> {
        &self.config
    }

    pub fn set(
        &mut self,
        index: usize,
        pixel: P,
    ) -> LedStripResult<(), RefreshError<Codec::EncodeError, Backend::Error>> {
        self.frame.set(index, pixel).map_err(lift_frame_error)
    }

    pub fn write(
        &mut self,
        pixels: &[P],
    ) -> LedStripResult<(), RefreshError<Codec::EncodeError, Backend::Error>> {
        self.frame.write(pixels).map_err(lift_frame_error)
    }

    pub fn fill(&mut self, pixel: P) {
        self.frame.fill(pixel);
    }

    pub fn clear_pixels(&mut self) {
        self.frame.clear();
    }

    pub fn refresh(&mut self) -> LedStripResult<(), RefreshError<Codec::EncodeError, Backend::Error>> {
        self.codec
            .encode(&self.config, self.frame.as_slice(), &mut self.tx_buf)
            // Lift structural errors to top-level; wrap codec-specific errors
            // into `RefreshError::Codec` so the caller sees a unified error shape.
            .map_err(|error| match error {
                LedStripError::InvalidIndex => LedStripError::InvalidIndex,
                LedStripError::InvalidLength { expected, actual } => {
                    LedStripError::InvalidLength { expected, actual }
                }
                LedStripError::BufferTooSmall { required, capacity } => {
                    LedStripError::BufferTooSmall { required, capacity }
                }
                LedStripError::Operation(error) => {
                    LedStripError::Operation(RefreshError::Codec(error))
                }
            })?;

        self.backend
            .transmit(self.tx_buf.as_slice())
            .map_err(|error| LedStripError::Operation(RefreshError::Backend(error)))
    }

    pub fn clear(&mut self) -> LedStripResult<(), RefreshError<Codec::EncodeError, Backend::Error>> {
        self.clear_pixels();
        self.refresh()
    }

    /// Destructures the driver, returning ownership of all parts.
    /// Useful for reusing the backend (e.g. SPI bus) or the codec
    /// after the LED strip is no longer needed.
    pub fn into_parts(self) -> (LedStripConfig<P, Proto>, Codec, Backend) {
        (self.config, self.codec, self.backend)
    }
}

impl<P, Proto, Codec, Backend, const MAX_LEDS: usize, const TX_CAPACITY: usize> core::fmt::Debug
    for LedStrip<P, Proto, Codec, Backend, MAX_LEDS, TX_CAPACITY>
where
    P: LedPixel + core::fmt::Debug,
    P::Order: core::fmt::Debug,
    Proto: SingleWireProtocol<P> + core::fmt::Debug,
    Codec: WireCodec<P, Proto, Backend::Word> + core::fmt::Debug,
    Backend: TransportBackend + core::fmt::Debug,
    Backend::Word: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LedStrip")
            .field("config", &self.config)
            .field("frame", &self.frame)
            .field("codec", &self.codec)
            .field("backend", &self.backend)
            .field("tx_buf", &self.tx_buf)
            .finish()
    }
}

/// Lifts a `FrameError` into the structural variants of `LedStripError`.
///
/// Delegates to `From<FrameError> for LedStripError<Infallible>` then widens
/// the `Operation` type parameter via [`convert`](LedStripError::convert).
/// This avoids duplicating the three-branch mapping and ensures consistency
/// with the `From` impl.
///
/// `FrameError` never carries operation-specific data, so `convert()` is
/// guaranteed to only map structural variants — no `Operation` variant is ever
/// constructed here.
fn lift_frame_error<CE, BE>(e: FrameError) -> LedStripError<RefreshError<CE, BE>> {
    LedStripError::from(e).convert()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug)]
    struct CodecErr;
    impl core::fmt::Display for CodecErr {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_str("encode failed")
        }
    }
    impl core::error::Error for CodecErr {}

    #[derive(Debug)]
    struct BackendErr;
    impl core::fmt::Display for BackendErr {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_str("bus busy")
        }
    }
    impl core::error::Error for BackendErr {}

    #[test]
    fn display_codec_variant_wraps_inner() {
        let e: RefreshError<CodecErr, BackendErr> = RefreshError::Codec(CodecErr);
        let s = e.to_string();
        assert!(s.starts_with("codec error:"), "{s}");
        assert!(s.contains("encode failed"), "{s}");
    }

    #[test]
    fn display_backend_variant_wraps_inner() {
        let e: RefreshError<CodecErr, BackendErr> = RefreshError::Backend(BackendErr);
        let s = e.to_string();
        assert!(s.starts_with("backend error:"), "{s}");
        assert!(s.contains("bus busy"), "{s}");
    }

    #[test]
    fn source_returns_inner_for_both_variants() {
        let c: RefreshError<CodecErr, BackendErr> = RefreshError::Codec(CodecErr);
        assert_eq!(
            core::error::Error::source(&c).unwrap().to_string(),
            "encode failed"
        );

        let b: RefreshError<CodecErr, BackendErr> = RefreshError::Backend(BackendErr);
        assert_eq!(
            core::error::Error::source(&b).unwrap().to_string(),
            "bus busy"
        );
    }

    // ── LedStrip integration tests ──────────────────────────────────

    use crate::{LedStripConfig, Rgb, SpiCodec, SpiEncodingPlan, Ws2812B};
    use core::cell::Cell;

    /// Mock backend that records the byte count of the last transmit call.
    #[derive(Debug)]
    struct MockBackend(Cell<usize>);

    impl MockBackend {
        fn new() -> Self {
            Self(Cell::new(0))
        }
        fn last_tx_len(&self) -> usize {
            self.0.get()
        }
    }

    impl TransportBackend for MockBackend {
        type Word = u8;
        type Error = core::convert::Infallible;

        fn transmit(&mut self, words: &[Self::Word]) -> Result<(), Self::Error> {
            self.0.set(words.len());
            Ok(())
        }
    }

    type TestStrip = LedStrip<Rgb, Ws2812B, SpiCodec, MockBackend, 16, 256>;

    fn make_codec() -> SpiCodec {
        SpiCodec::for_protocol::<Rgb, Ws2812B>(SpiEncodingPlan::ws281x_3bit(), false).unwrap()
    }

    fn make_strip(len: usize) -> TestStrip {
        let config = LedStripConfig::ws2812b(len);
        TestStrip::new(config, make_codec(), MockBackend::new()).unwrap()
    }

    #[test]
    fn new_accepts_valid_config() {
        let strip = make_strip(8);
        assert_eq!(strip.len(), 8);
        assert!(!strip.is_empty());
    }

    #[test]
    fn new_accepts_zero_len() {
        let config = LedStripConfig::ws2812b(0);
        let strip = TestStrip::new(config, make_codec(), MockBackend::new()).unwrap();
        assert!(strip.is_empty());
        assert_eq!(strip.len(), 0);
    }

    #[test]
    fn new_rejects_too_many_leds() {
        let config = LedStripConfig::ws2812b(20); // MAX_LEDS = 16
        let err = TestStrip::new(config, make_codec(), MockBackend::new()).unwrap_err();
        assert_eq!(
            err,
            LedStripError::BufferTooSmall {
                required: 20,
                capacity: 16,
            }
        );
    }

    #[test]
    fn new_rejects_tx_capacity_overflow() {
        // 6 pixels × 9 payload bytes + reset ≈ 69 bytes, but TX_CAPACITY = 5
        type TinyStrip = LedStrip<Rgb, Ws2812B, SpiCodec, MockBackend, 16, 5>;
        let config = LedStripConfig::ws2812b(6);
        let err = TinyStrip::new(config, make_codec(), MockBackend::new()).unwrap_err();
        assert!(matches!(err, LedStripError::BufferTooSmall { .. }));
    }

    #[test]
    fn config_accessor_returns_config() {
        let strip = make_strip(3);
        assert_eq!(strip.config().len(), 3);
    }

    #[test]
    fn set_updates_pixel() {
        let mut strip = make_strip(4);
        strip.set(1, Rgb::new(10, 20, 30)).unwrap();
        // refresh to verify the pixel was actually set
        strip.refresh().unwrap();
        assert!(strip.backend.last_tx_len() > 0);
    }

    #[test]
    fn set_rejects_invalid_index() {
        let mut strip = make_strip(4);
        let err = strip.set(4, Rgb::new(1, 2, 3)).unwrap_err();
        assert!(matches!(err, LedStripError::InvalidIndex));
    }

    #[test]
    fn write_updates_all_pixels() {
        let mut strip = make_strip(3);
        strip
            .write(&[Rgb::new(1, 2, 3), Rgb::new(4, 5, 6), Rgb::new(7, 8, 9)])
            .unwrap();
        strip.refresh().unwrap();
        assert!(strip.backend.last_tx_len() > 0);
    }

    #[test]
    fn write_rejects_length_mismatch() {
        let mut strip = make_strip(4);
        let err = strip.write(&[Rgb::new(1, 2, 3)]).unwrap_err();
        assert!(matches!(err, LedStripError::InvalidLength { .. }));
    }

    #[test]
    fn fill_sets_all_pixels() {
        let mut strip = make_strip(4);
        strip.fill(Rgb::new(255, 0, 0));
        strip.refresh().unwrap();
        assert!(strip.backend.last_tx_len() > 0);
    }

    #[test]
    fn clear_pixels_then_refresh_zeroes_output() {
        let mut strip = make_strip(4);
        strip.fill(Rgb::WHITE);
        strip.clear_pixels();
        strip.refresh().unwrap();
        assert!(strip.backend.last_tx_len() > 0);
    }

    #[test]
    fn refresh_transmits_encoded_data() {
        let mut strip = make_strip(2);
        strip.refresh().unwrap();
        let tx_len = strip.backend.last_tx_len();
        // 2 pixels × 9 bytes + reset → should be non-zero
        assert!(tx_len > 0, "expected non-zero transmit length, got {tx_len}");
    }

    #[test]
    fn clear_combines_clear_pixels_and_refresh() {
        let mut strip = make_strip(4);
        strip.fill(Rgb::WHITE);
        strip.refresh().unwrap();
        let filled_len = strip.backend.last_tx_len();

        strip.clear().unwrap();
        let cleared_len = strip.backend.last_tx_len();

        // Both should produce non-zero output (data + reset)
        assert!(filled_len > 0);
        assert!(cleared_len > 0);
    }

    #[test]
    fn into_parts_returns_owned_components() {
        let strip = make_strip(4);
        let (config, codec, _backend) = strip.into_parts();
        assert_eq!(config.len(), 4);
        // Verify the codec still works
        assert_eq!(codec.plan().spi_hz(), 2_400_000);
    }

    #[test]
    fn debug_output_contains_fields() {
        let strip = make_strip(2);
        let s = format!("{strip:?}");
        assert!(s.contains("LedStrip"), "{s}");
        assert!(s.contains("config"), "{s}");
        assert!(s.contains("frame"), "{s}");
        assert!(s.contains("codec"), "{s}");
        assert!(s.contains("backend"), "{s}");
        assert!(s.contains("tx_buf"), "{s}");
    }

    // ── lift_frame_error ────────────────────────────────────────────

    #[test]
    fn lift_frame_error_maps_invalid_index() {
        let err = lift_frame_error::<(), ()>(crate::FrameError::InvalidIndex);
        assert_eq!(err, LedStripError::InvalidIndex);
    }

    #[test]
    fn lift_frame_error_maps_invalid_length() {
        let err = lift_frame_error::<(), ()>(crate::FrameError::InvalidLength {
            expected: 10,
            actual: 5,
        });
        assert_eq!(
            err,
            LedStripError::InvalidLength {
                expected: 10,
                actual: 5,
            }
        );
    }
}
