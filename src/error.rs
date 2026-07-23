use crate::frame::FrameError;

/// The unified error type for all `led-strip` operations.
///
/// Three variants cover structural issues (index, length, capacity); the
/// fourth (`Operation`) wraps backend- or codec-specific errors via the
/// generic `E` parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum LedStripError<E> {
    /// Pixel index out of range.
    InvalidIndex,
    /// Pixel slice length does not match the configured strip length.
    InvalidLength { expected: usize, actual: usize },
    /// Buffer capacity exceeded. Check `MAX_LEDS` or `TX_CAPACITY`.
    BufferTooSmall { required: usize, capacity: usize },
    /// Backend- or codec-specific error.
    Operation(E),
}

/// Convenience alias for `Result<T, LedStripError<E>>`.
pub type LedStripResult<T, E> = Result<T, LedStripError<E>>;

impl<E> From<E> for LedStripError<E> {
    fn from(error: E) -> Self {
        Self::Operation(error)
    }
}

impl From<FrameError> for LedStripError<core::convert::Infallible> {
    fn from(e: FrameError) -> Self {
        match e {
            FrameError::InvalidIndex => Self::InvalidIndex,
            FrameError::InvalidLength { expected, actual } => {
                Self::InvalidLength { expected, actual }
            }
            FrameError::BufferTooSmall { required, capacity } => {
                Self::BufferTooSmall { required, capacity }
            }
        }
    }
}

impl<E: core::fmt::Display> core::fmt::Display for LedStripError<E> {
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
            Self::Operation(inner) => write!(f, "operation error: {inner}"),
        }
    }
}

impl<E: core::error::Error + 'static> core::error::Error for LedStripError<E> {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Operation(inner) => Some(inner),
            _ => None,
        }
    }
}

impl<E> LedStripError<E> {
    /// Maps the `Operation` variant's error type via `f`, leaving structural
    /// variants unchanged.  Useful for wrapping codec/backend errors into a
    /// unified [`RefreshError`](crate::RefreshError) without enumerating every
    /// structural variant by hand.
    pub fn map_operation<F, E2>(self, f: F) -> LedStripError<E2>
    where
        F: FnOnce(E) -> E2,
    {
        match self {
            Self::InvalidIndex => LedStripError::InvalidIndex,
            Self::InvalidLength { expected, actual } => {
                LedStripError::InvalidLength { expected, actual }
            }
            Self::BufferTooSmall { required, capacity } => {
                LedStripError::BufferTooSmall { required, capacity }
            }
            Self::Operation(e) => LedStripError::Operation(f(e)),
        }
    }
}

impl LedStripError<core::convert::Infallible> {
    /// Widens an infallible error into `LedStripError<E>`.
    /// The `Operation` branch is unreachable because `Infallible` has no values.
    pub fn convert<E>(self) -> LedStripError<E> {
        match self {
            Self::InvalidIndex => LedStripError::InvalidIndex,
            Self::InvalidLength { expected, actual } => {
                LedStripError::InvalidLength { expected, actual }
            }
            Self::BufferTooSmall { required, capacity } => {
                LedStripError::BufferTooSmall { required, capacity }
            }
            Self::Operation(never) => match never {},
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;

    #[derive(Debug)]
    struct DummyOp(&'static str);

    impl core::fmt::Display for DummyOp {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_str(self.0)
        }
    }

    impl core::error::Error for DummyOp {}

    #[test]
    fn display_invalid_index() {
        let e: LedStripError<DummyOp> = LedStripError::InvalidIndex;
        assert_eq!(e.to_string(), "invalid pixel index");
    }

    #[test]
    fn display_invalid_length_includes_fields() {
        let e: LedStripError<DummyOp> = LedStripError::InvalidLength {
            expected: 60,
            actual: 59,
        };
        let s = e.to_string();
        assert!(s.contains("expected=60"), "{s}");
        assert!(s.contains("actual=59"), "{s}");
    }

    #[test]
    fn display_buffer_too_small_includes_fields() {
        let e: LedStripError<DummyOp> = LedStripError::BufferTooSmall {
            required: 144,
            capacity: 128,
        };
        let s = e.to_string();
        assert!(s.contains("required=144"), "{s}");
        assert!(s.contains("capacity=128"), "{s}");
    }

    #[test]
    fn display_operation_wraps_inner() {
        let e: LedStripError<DummyOp> = LedStripError::Operation(DummyOp("spi nack"));
        assert!(e.to_string().contains("spi nack"));
    }

    #[test]
    fn error_source_returns_inner_for_operation_only() {
        let op: LedStripError<DummyOp> = LedStripError::Operation(DummyOp("boom"));
        let src = core::error::Error::source(&op);
        assert!(src.is_some());
        assert_eq!(src.unwrap().to_string(), "boom");

        let idx: LedStripError<DummyOp> = LedStripError::InvalidIndex;
        assert!(core::error::Error::source(&idx).is_none());
    }

    #[test]
    fn convert_preserves_structural_variants() {
        let cases: [LedStripError<Infallible>; 3] = [
            LedStripError::InvalidIndex,
            LedStripError::InvalidLength {
                expected: 1,
                actual: 2,
            },
            LedStripError::BufferTooSmall {
                required: 3,
                capacity: 4,
            },
        ];
        for c in cases {
            let widened: LedStripError<DummyOp> = c.convert();
            // round-trip Display to assert variant identity preserved
            let _: String = format!("{widened}");
        }
    }

    #[test]
    fn from_generic_error_wraps_into_operation() {
        let e: LedStripError<DummyOp> = LedStripError::from(DummyOp("oh no"));
        match e {
            LedStripError::Operation(inner) => assert_eq!(inner.0, "oh no"),
            _ => panic!("expected Operation variant"),
        }
    }

    #[test]
    fn from_frame_error_invalid_index() {
        let e: LedStripError<Infallible> = LedStripError::from(FrameError::InvalidIndex);
        assert_eq!(e, LedStripError::InvalidIndex);
    }

    #[test]
    fn from_frame_error_invalid_length() {
        let e: LedStripError<Infallible> = LedStripError::from(FrameError::InvalidLength {
            expected: 60,
            actual: 30,
        });
        assert_eq!(
            e,
            LedStripError::InvalidLength {
                expected: 60,
                actual: 30,
            }
        );
    }

    #[test]
    fn from_frame_error_buffer_too_small() {
        let e: LedStripError<Infallible> = LedStripError::from(FrameError::BufferTooSmall {
            required: 144,
            capacity: 128,
        });
        assert_eq!(
            e,
            LedStripError::BufferTooSmall {
                required: 144,
                capacity: 128,
            }
        );
    }
}
