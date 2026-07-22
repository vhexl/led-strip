use heapless::Vec;

use crate::{LedPixel, LedStripConfig, SingleWireProtocol};

/// Errors specific to [`FrameBuf`] operations.
///
/// Separated from [`crate::LedStripError`] so `FrameBuf` does not carry a phantom
/// `Operation` type parameter that is never constructed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Pixel index out of range.
    InvalidIndex,
    /// Supplied pixel slice length does not match the frame size.
    InvalidLength { expected: usize, actual: usize },
    /// Heap-less `Vec` capacity (`MAX_LEDS`) too small for the requested frame.
    BufferTooSmall { required: usize, capacity: usize },
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidIndex => f.write_str("invalid pixel index"),
            Self::InvalidLength { expected, actual } => {
                write!(f, "invalid length: expected={expected}, actual={actual}")
            }
            Self::BufferTooSmall { required, capacity } => {
                write!(
                    f,
                    "buffer too small: required={required}, capacity={capacity}"
                )
            }
        }
    }
}

impl core::error::Error for FrameError {}

/// Heap-less pixel frame buffer backed by a fixed-capacity [`heapless::Vec`].
///
/// `MAX_LEDS` is the compile-time upper bound on pixel count. Construction
/// fails with [`FrameError::BufferTooSmall`] if the requested `len` exceeds it.
#[derive(Debug, Clone)]
pub struct FrameBuf<P, const MAX_LEDS: usize>
where
    P: LedPixel,
{
    pixels: Vec<P, MAX_LEDS>,
}

impl<P, const MAX_LEDS: usize> FrameBuf<P, MAX_LEDS>
where
    P: LedPixel,
{
    /// Creates a frame of `len` pixels, initialized to `P::default()` (all-off / black).
    pub fn new(len: usize) -> Result<Self, FrameError> {
        if len > MAX_LEDS {
            return Err(FrameError::BufferTooSmall {
                required: len,
                capacity: MAX_LEDS,
            });
        }

        let mut pixels = Vec::new();
        pixels
            .resize(len, P::default())
            .expect("len ≤ MAX_LEDS was checked above; resize cannot fail");

        Ok(Self { pixels })
    }

    /// Creates a frame whose size matches the given configuration.
    pub fn from_config<Proto>(config: &LedStripConfig<P, Proto>) -> Result<Self, FrameError>
    where
        Proto: SingleWireProtocol<P>,
    {
        Self::new(config.len())
    }

    /// Number of pixels in the frame.
    #[must_use]
    pub fn len(&self) -> usize {
        self.pixels.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pixels.is_empty()
    }

    /// Compile-time capacity (`MAX_LEDS`).
    #[must_use]
    pub const fn capacity(&self) -> usize {
        MAX_LEDS
    }

    /// Immutable view of the pixel buffer.
    #[must_use]
    pub fn as_slice(&self) -> &[P] {
        self.pixels.as_slice()
    }

    /// Mutable view of the pixel buffer.
    #[must_use]
    pub fn as_mut_slice(&mut self) -> &mut [P] {
        self.pixels.as_mut_slice()
    }

    /// Sets the pixel at `index`. Returns [`FrameError::InvalidIndex`] if out of bounds.
    pub fn set(&mut self, index: usize, pixel: P) -> Result<(), FrameError> {
        let Some(slot) = self.pixels.get_mut(index) else {
            return Err(FrameError::InvalidIndex);
        };

        *slot = pixel;
        Ok(())
    }

    /// Overwrites all pixels with the given slice. The slice length must match `self.len()`.
    pub fn write(&mut self, pixels: &[P]) -> Result<(), FrameError> {
        if pixels.len() != self.len() {
            return Err(FrameError::InvalidLength {
                expected: self.len(),
                actual: pixels.len(),
            });
        }

        self.as_mut_slice().copy_from_slice(pixels);
        Ok(())
    }

    /// Fills every pixel with the same value.
    pub fn fill(&mut self, pixel: P) {
        self.as_mut_slice().fill(pixel);
    }

    /// Resets all pixels to `P::default()` (all-off / black).
    pub fn clear(&mut self) {
        self.fill(P::default());
    }
}

#[cfg(test)]
mod tests {
    use super::{FrameBuf, FrameError};
    use crate::{LedStripConfig, Rgb};

    // ── Construction ─────────────────────────────────────────────────

    #[test]
    fn new_success_initializes_to_default() {
        let frame = FrameBuf::<Rgb, 4>::new(2).unwrap();
        assert_eq!(frame.len(), 2);
        assert!(!frame.is_empty());
        assert_eq!(frame.capacity(), 4);
        assert_eq!(frame.as_slice(), &[Rgb::BLACK, Rgb::BLACK]);
    }

    #[test]
    fn new_accepts_exact_capacity() {
        let frame = FrameBuf::<Rgb, 3>::new(3).unwrap();
        assert_eq!(frame.len(), 3);
    }

    #[test]
    fn new_accepts_zero_len() {
        let frame = FrameBuf::<Rgb, 4>::new(0).unwrap();
        assert!(frame.is_empty());
        assert_eq!(frame.len(), 0);
    }

    #[test]
    fn rejects_capacity_overflow() {
        let err = FrameBuf::<Rgb, 2>::new(3).unwrap_err();
        assert_eq!(
            err,
            FrameError::BufferTooSmall {
                required: 3,
                capacity: 2,
            }
        );
    }

    #[test]
    fn from_config_constructs_matching_frame() {
        let config = LedStripConfig::ws2812b(5);
        let frame = FrameBuf::<Rgb, 10>::from_config(&config).unwrap();
        assert_eq!(frame.len(), 5);
    }

    // ── set ──────────────────────────────────────────────────────────

    #[test]
    fn set_updates_pixel_at_index() {
        let mut frame = FrameBuf::<Rgb, 4>::new(3).unwrap();
        frame.set(1, Rgb::new(10, 20, 30)).unwrap();
        assert_eq!(frame.as_slice()[1], Rgb::new(10, 20, 30));
        // Other pixels unchanged
        assert_eq!(frame.as_slice()[0], Rgb::BLACK);
        assert_eq!(frame.as_slice()[2], Rgb::BLACK);
    }

    #[test]
    fn set_rejects_out_of_bounds() {
        let mut frame = FrameBuf::<Rgb, 4>::new(2).unwrap();
        let err = frame.set(2, Rgb::new(1, 2, 3)).unwrap_err();
        assert_eq!(err, FrameError::InvalidIndex);
    }

    #[test]
    fn set_accepts_last_valid_index() {
        let mut frame = FrameBuf::<Rgb, 4>::new(2).unwrap();
        frame.set(1, Rgb::new(1, 2, 3)).unwrap();
        assert_eq!(frame.as_slice()[1], Rgb::new(1, 2, 3));
    }

    // ── write ────────────────────────────────────────────────────────

    #[test]
    fn write_overwrites_all_pixels() {
        let mut frame = FrameBuf::<Rgb, 4>::new(2).unwrap();
        frame
            .write(&[Rgb::new(1, 2, 3), Rgb::new(4, 5, 6)])
            .unwrap();
        assert_eq!(frame.as_slice(), &[Rgb::new(1, 2, 3), Rgb::new(4, 5, 6)]);
    }

    #[test]
    fn rejects_mismatched_write_length() {
        let mut frame = FrameBuf::<Rgb, 4>::new(2).unwrap();
        let err = frame.write(&[Rgb::new(1, 2, 3)]).unwrap_err();
        assert_eq!(
            err,
            FrameError::InvalidLength {
                expected: 2,
                actual: 1,
            }
        );
    }

    #[test]
    fn write_too_long_rejected() {
        let mut frame = FrameBuf::<Rgb, 4>::new(2).unwrap();
        let err = frame
            .write(&[Rgb::new(1, 2, 3), Rgb::new(4, 5, 6), Rgb::new(7, 8, 9)])
            .unwrap_err();
        assert_eq!(
            err,
            FrameError::InvalidLength {
                expected: 2,
                actual: 3,
            }
        );
    }

    // ── fill / clear ─────────────────────────────────────────────────

    #[test]
    fn fill_sets_all_pixels_to_same_value() {
        let mut frame = FrameBuf::<Rgb, 4>::new(3).unwrap();
        frame.fill(Rgb::new(200, 100, 50));
        assert_eq!(
            frame.as_slice(),
            &[
                Rgb::new(200, 100, 50),
                Rgb::new(200, 100, 50),
                Rgb::new(200, 100, 50),
            ]
        );
    }

    #[test]
    fn clear_resets_all_to_black() {
        let mut frame = FrameBuf::<Rgb, 4>::new(3).unwrap();
        frame.fill(Rgb::WHITE);
        frame.clear();
        assert_eq!(frame.as_slice(), &[Rgb::BLACK, Rgb::BLACK, Rgb::BLACK]);
    }

    // ── as_slice / as_mut_slice ──────────────────────────────────────

    #[test]
    fn as_slice_returns_correct_length() {
        let frame = FrameBuf::<Rgb, 4>::new(2).unwrap();
        assert_eq!(frame.as_slice().len(), 2);
    }

    #[test]
    fn as_mut_slice_allows_in_place_mutation() {
        let mut frame = FrameBuf::<Rgb, 4>::new(2).unwrap();
        frame.as_mut_slice()[0] = Rgb::new(9, 8, 7);
        assert_eq!(frame.as_slice()[0], Rgb::new(9, 8, 7));
    }

    // ── FrameError ───────────────────────────────────────────────────

    #[test]
    fn frame_error_display_invalid_index() {
        assert_eq!(FrameError::InvalidIndex.to_string(), "invalid pixel index");
    }

    #[test]
    fn frame_error_display_invalid_length() {
        let e = FrameError::InvalidLength {
            expected: 60,
            actual: 59,
        };
        let s = e.to_string();
        assert!(s.contains("expected=60"), "{s}");
        assert!(s.contains("actual=59"), "{s}");
    }

    #[test]
    fn frame_error_display_buffer_too_small() {
        let e = FrameError::BufferTooSmall {
            required: 100,
            capacity: 50,
        };
        let s = e.to_string();
        assert!(s.contains("required=100"), "{s}");
        assert!(s.contains("capacity=50"), "{s}");
    }

    #[test]
    fn frame_error_is_core_error() {
        let _: &dyn core::error::Error = &FrameError::InvalidIndex;
    }

    #[test]
    fn clone_produces_independent_copy() {
        let mut orig = FrameBuf::<Rgb, 4>::new(2).unwrap();
        orig.set(0, Rgb::new(1, 2, 3)).unwrap();

        let cloned = orig.clone();
        // cloned has same content
        assert_eq!(cloned.as_slice(), orig.as_slice());
        // mutating original does not affect clone
        orig.set(0, Rgb::new(9, 9, 9)).unwrap();
        assert_eq!(cloned.as_slice()[0], Rgb::new(1, 2, 3));
    }
}
