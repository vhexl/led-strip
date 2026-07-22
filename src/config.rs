use core::marker::PhantomData;

use crate::{
    LedPixel, Rgb, Rgb16, Rgbw, SingleWireProtocol, Sk6812, Ws2811, Ws2812B, Ws2816,
};

/// Compile-time-verified LED strip configuration.
///
/// Encodes the pixel count, color channel order, and `<pixel, protocol>` type
/// pair. An invalid `<pixel, protocol>` combination (e.g. `Rgb` with `Sk6812`)
/// is rejected at compile time.
///
/// Convenience constructors ([`ws2812b`](Self::ws2812b),
/// [`ws2811`](Self::ws2811), etc.) default to each protocol's typical color
/// order. Use [`new`](Self::new) with an explicit `color_order` when your strip
/// uses a non-standard wiring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LedStripConfig<P, Proto>
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
{
    len: usize,
    color_order: P::Order,
    // PhantomData fields anchor P and Proto at the type level without holding a value.
    _pixel: PhantomData<P>,
    _protocol: PhantomData<Proto>,
}

impl<P, Proto> LedStripConfig<P, Proto>
where
    P: LedPixel,
    Proto: SingleWireProtocol<P>,
{
    /// Creates a configuration with an explicit color order.
    /// Prefer the convenience constructors unless your strip uses non-standard wiring.
    #[must_use]
    pub const fn new(len: usize, color_order: P::Order) -> Self {
        Self {
            len,
            color_order,
            _pixel: PhantomData,
            _protocol: PhantomData,
        }
    }

    /// Number of pixels in the strip.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[must_use]
    pub const fn color_order(&self) -> P::Order {
        self.color_order
    }

    #[must_use]
    pub const fn bytes_per_pixel(&self) -> usize {
        P::BYTES_PER_PIXEL
    }

    #[must_use]
    pub const fn frame_len_bytes(&self) -> usize {
        self.len * P::BYTES_PER_PIXEL
    }

    #[must_use]
    pub const fn reset_ns(&self) -> u32 {
        Proto::RESET_NS
    }
}

impl LedStripConfig<Rgb, Ws2812B> {
    /// Convenience constructor for WS2812B strips (GRB color order).
    #[must_use]
    pub const fn ws2812b(len: usize) -> Self {
        Self::new(len, Ws2812B::DEFAULT_COLOR_ORDER)
    }
}

impl LedStripConfig<Rgb, Ws2811> {
    /// Convenience constructor for WS2811 strips (RGB color order).
    #[must_use]
    pub const fn ws2811(len: usize) -> Self {
        Self::new(len, Ws2811::DEFAULT_COLOR_ORDER)
    }
}

impl LedStripConfig<Rgbw, Sk6812> {
    /// Convenience constructor for SK6812 RGBW strips (GRBW color order).
    #[must_use]
    pub const fn sk6812(len: usize) -> Self {
        Self::new(len, Sk6812::DEFAULT_COLOR_ORDER)
    }
}

impl LedStripConfig<Rgb16, Ws2816> {
    /// Convenience constructor for WS2816 16-bit strips (GRB color order).
    #[must_use]
    pub const fn ws2816(len: usize) -> Self {
        Self::new(len, Ws2816::DEFAULT_COLOR_ORDER)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{RgbOrder, RgbwOrder, Rgb16Order};

    #[test]
    fn ws2812b_config_defaults_to_grb() {
        let cfg = LedStripConfig::ws2812b(60);
        assert_eq!(cfg.len(), 60);
        assert!(!cfg.is_empty());
        assert_eq!(cfg.color_order(), RgbOrder::Grb);
        assert_eq!(cfg.bytes_per_pixel(), 3);
        assert_eq!(cfg.frame_len_bytes(), 180);
        assert_eq!(cfg.reset_ns(), 50_000);
    }

    #[test]
    fn ws2811_config_defaults_to_rgb() {
        let cfg = LedStripConfig::ws2811(100);
        assert_eq!(cfg.len(), 100);
        assert_eq!(cfg.color_order(), RgbOrder::Rgb);
        assert_eq!(cfg.reset_ns(), 50_000);
    }

    #[test]
    fn sk6812_config_defaults_to_grbw() {
        let cfg = LedStripConfig::sk6812(30);
        assert_eq!(cfg.len(), 30);
        assert_eq!(cfg.color_order(), RgbwOrder::Grbw);
        assert_eq!(cfg.bytes_per_pixel(), 4);
        assert_eq!(cfg.frame_len_bytes(), 120);
        assert_eq!(cfg.reset_ns(), 80_000);
    }

    #[test]
    fn ws2816_config_defaults_to_grb() {
        let cfg = LedStripConfig::ws2816(10);
        assert_eq!(cfg.len(), 10);
        assert_eq!(cfg.color_order(), Rgb16Order::Grb);
        assert_eq!(cfg.bytes_per_pixel(), 6);
        assert_eq!(cfg.frame_len_bytes(), 60);
        assert_eq!(cfg.reset_ns(), 300_000);
    }

    #[test]
    fn custom_color_order_overrides_default() {
        let cfg = LedStripConfig::<Rgb, Ws2812B>::new(5, RgbOrder::Rgb);
        assert_eq!(cfg.color_order(), RgbOrder::Rgb);
    }

    #[test]
    fn is_empty_true_for_zero_len() {
        let cfg = LedStripConfig::ws2812b(0);
        assert!(cfg.is_empty());
        assert_eq!(cfg.len(), 0);
        assert_eq!(cfg.frame_len_bytes(), 0);
    }
}
