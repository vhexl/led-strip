use core::marker::PhantomData;

use heapless::Vec;

use crate::{LedPixel, LedStripError, LedStripResult, SingleWireProtocol};

/// Unified error type returned by [`LedStrip::write`].
///
/// Wraps either a codec-level error (encoding failure) or a backend-level
/// error (transmission failure), so callers see a single error shape
/// regardless of which layer failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RefreshError<CodecError, BackendError> {
    /// The codec failed to encode the pixel data into transport words.
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
/// words onto the wire. It does **not** understand LED protocol timings —/// that is the codec's job.
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
/// time —you cannot accidentally drive SK6812 pixels with a WS2812B encoding plan.
///
/// # Implementing
///
/// - [`encoded_len`](Self::encoded_len) must return the exact number of
///   `Word`s that [`encode`](Self::encode) will produce for the given pixel
///   count (including reset/trailer). This is used for capacity checks before encoding.
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
    /// [`SpiCodec`](crate::SpiCodec)) never fail during encoding —the
    /// only possible error is `BufferTooSmall`.
    type EncodeError;

    /// Returns the exact number of `Word`s that [`encode`](Self::encode) will
    /// produce for `pixel_count` pixels. Includes payload + reset/trailer overhead.
    fn encoded_len(&self, pixel_count: usize) -> usize;

    /// Encodes `pixels` into transport words, appending to `out`.
    ///
    /// The buffer is cleared before encoding begins, so callers may safely
    /// reuse a buffer across multiple `encode` calls without manual `out.clear()`.
    fn encode<const TX_CAPACITY: usize>(
        &self,
        color_order: P::Order,
        pixels: &[P],
        out: &mut Vec<Word, TX_CAPACITY>,
    ) -> LedStripResult<(), Self::EncodeError>;
}

/// Stateless driver for a single-wire addressable LED strip.
///
/// Owns a codec, a transport backend, and a reusable transmission buffer.
/// This driver does **not** cache a frame buffer —each [`write`](Self::write) call accepts a dynamic-length pixel slice,
/// encodes it, and transmits it to the strip in a single operation.
///
/// # Type parameters
///
/// | Parameter | Role |
/// |-----------|------|
/// | `P` | Pixel type ([`crate::Rgb`], [`crate::Rgbw`], [`crate::Rgb16`]) |
/// | `Proto` | Protocol marker ([`crate::Ws2812B`], [`crate::Sk6812`], [`crate::Ws2811`], [`crate::Ws2816`]) |
/// | `Codec` | Encoding scheme (currently [`crate::SpiCodec`]; future backends may provide their own) |
/// | `Backend` | Transport layer (currently [`crate::SpiBackend`]; future backends may provide their own) |
/// | `TX_CAPACITY` | Compile-time upper bound on transport buffer size (in `Word`s) |
pub struct LedStrip<P, Proto, Codec, Backend, const TX_CAPACITY: usize>
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
    Codec: WireCodec<P, Proto, Backend::Word>,
    Backend: TransportBackend,
{
    color_order: P::Order,
    codec: Codec,
    backend: Backend,
    tx_buf: Vec<Backend::Word, TX_CAPACITY>,
    _proto: PhantomData<Proto>,
}

impl<P, Proto, Codec, Backend, const TX_CAPACITY: usize>
    LedStrip<P, Proto, Codec, Backend, TX_CAPACITY>
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
    Codec: WireCodec<P, Proto, Backend::Word>,
    Backend: TransportBackend,
{
    /// Constructs the driver.
    ///
    /// `color_order` determines the on-wire byte order (e.g. `RgbOrder::Grb`
    /// for WS2812B). Use the protocol's `DEFAULT_COLOR_ORDER` associated
    /// constant for the typical wiring.
    ///
    /// No capacity validation is performed here —[`write`](Self::write)
    /// checks `TX_CAPACITY` on each call and returns
    /// [`BufferTooSmall`](LedStripError::BufferTooSmall) if the encoded
    /// frame exceeds the buffer.
    #[must_use]
    pub fn new(color_order: P::Order, codec: Codec, backend: Backend) -> Self {
        Self {
            color_order,
            codec,
            backend,
            tx_buf: Vec::new(),
            _proto: PhantomData,
        }
    }

    /// Encodes `pixels` and transmits them to the LED strip in one call.
    ///
    /// Returns [`LedStripError::BufferTooSmall`] if the encoded payload
    /// (including reset/trailer) exceeds `TX_CAPACITY`. All other errors
    /// are wrapped in [`RefreshError`].
    ///
    /// The pixel count is dynamic —pass any slice length supported by your
    /// `TX_CAPACITY`.
    pub fn write(
        &mut self,
        pixels: &[P],
    ) -> LedStripResult<(), RefreshError<Codec::EncodeError, Backend::Error>> {
        self.codec
            .encode(self.color_order, pixels, &mut self.tx_buf)
            .map_err(|e| e.map_operation(RefreshError::Codec))?;

        self.backend
            .transmit(self.tx_buf.as_slice())
            .map_err(|error| LedStripError::Operation(RefreshError::Backend(error)))
    }

    /// Destructures the driver, returning ownership of the codec and backend.
    /// Useful for reusing the backend (e.g. SPI bus) or the codec
    /// after the LED strip is no longer needed.
    #[must_use]
    pub fn into_parts(self) -> (Codec, Backend) {
        (self.codec, self.backend)
    }
}

impl<P, Proto, Codec, Backend, const TX_CAPACITY: usize> core::fmt::Debug
    for LedStrip<P, Proto, Codec, Backend, TX_CAPACITY>
where
    P: LedPixel,
    P::Order: core::fmt::Debug,
    Proto: SingleWireProtocol<P>,
    Codec: WireCodec<P, Proto, Backend::Word> + core::fmt::Debug,
    Backend: TransportBackend + core::fmt::Debug,
    Backend::Word: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LedStrip")
            .field("color_order", &self.color_order)
            .field("codec", &self.codec)
            .field("backend", &self.backend)
            .field("tx_buf", &self.tx_buf)
            .finish()
    }
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

    use crate::{Rgb, RgbOrder, SpiCodec, SpiEncodingPlan, Ws2812B};
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

    type TestStrip = LedStrip<Rgb, Ws2812B, SpiCodec<Rgb, Ws2812B>, MockBackend, 256>;

    fn make_codec() -> SpiCodec<Rgb, Ws2812B> {
        SpiCodec::<Rgb, Ws2812B>::for_protocol(SpiEncodingPlan::ws2812_3bit(), false).unwrap()
    }

    fn make_strip() -> TestStrip {
        TestStrip::new(RgbOrder::Grb, make_codec(), MockBackend::new())
    }

    #[test]
    fn new_constructs_without_error() {
        let strip = make_strip();
        // Verify default state
        let _ = format!("{strip:?}");
    }

    #[test]
    fn write_transmits_encoded_data() {
        let mut strip = make_strip();
        strip
            .write(&[
                Rgb::new(255, 0, 0),
                Rgb::new(0, 255, 0),
                Rgb::new(0, 0, 255),
            ])
            .unwrap();
        assert!(
            strip.backend.last_tx_len() > 0,
            "expected non-zero transmit length"
        );
    }

    #[test]
    fn write_with_zero_pixels_sends_reset_only() {
        let mut strip = make_strip();
        strip.write(&[]).unwrap();
        // Zero pixels still produce reset/latch bytes
        assert!(strip.backend.last_tx_len() > 0);
    }

    #[test]
    fn write_rejects_buffer_too_small() {
        type TinyStrip = LedStrip<Rgb, Ws2812B, SpiCodec<Rgb, Ws2812B>, MockBackend, 5>;
        let mut strip = TinyStrip::new(RgbOrder::Grb, make_codec(), MockBackend::new());
        // 2 pixels × 9 payload bytes + reset ≫ 5
        let err = strip
            .write(&[Rgb::new(0, 0, 0), Rgb::new(0, 0, 0)])
            .unwrap_err();
        assert!(matches!(err, LedStripError::BufferTooSmall { .. }));
    }

    #[test]
    fn into_parts_returns_owned_codec_and_backend() {
        let strip = make_strip();
        let (codec, _backend) = strip.into_parts();
        // Verify the codec still works
        assert_eq!(codec.plan().spi_hz(), 2_400_000);
    }

    #[test]
    fn debug_output_contains_fields() {
        let strip = make_strip();
        let s = format!("{strip:?}");
        assert!(s.contains("LedStrip"), "{s}");
        assert!(s.contains("color_order"), "{s}");
        assert!(s.contains("codec"), "{s}");
        assert!(s.contains("backend"), "{s}");
        assert!(s.contains("tx_buf"), "{s}");
    }
}
