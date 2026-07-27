/// Discriminant for the three supported pixel formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum PixelKind {
    /// 3-channel 8-bit RGB.
    Rgb,
    /// 4-channel 8-bit RGBW (dedicated white channel).
    Rgbw,
    /// 3-channel 16-bit RGB (high dynamic range).
    Rgb16,
}

/// Wire-level byte order for 8-bit RGB pixels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RgbOrder {
    /// Red, Green, Blue.
    Rgb,
    /// Green, Red, Blue (WS2812B default).
    Grb,
}

/// Wire-level byte order for 8-bit RGBW pixels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum RgbwOrder {
    /// Red, Green, Blue, White.
    Rgbw,
    /// Green, Red, Blue, White (SK6812 default).
    Grbw,
}

/// Wire-level byte order for 16-bit RGB pixels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Rgb16Order {
    /// Red, Green, Blue (big-endian per channel).
    Rgb,
    /// Green, Red, Blue (big-endian per channel, WS2816 default).
    Grb,
}

/// Logical pixel representation for a single LED.
///
/// Each pixel type owns its color-channel layout, byte count, and exclusive
/// color-order enum (`type Order`). This prevents e.g. `Grbw` from being
/// applied to an `Rgb` pixel at compile time.
///
/// Sealed so downstream crates cannot add new pixel types — new pixel formats
/// must be added here alongside their protocol impl matrix.
pub trait LedPixel: Copy + Default + private::Sealed {
    type Order: Copy + Eq;

    /// Discriminant for runtime pixel-format dispatch.
    const KIND: PixelKind;
    /// Number of bytes this pixel occupies on the wire.
    const BYTES_PER_PIXEL: usize;

    /// Encodes this pixel into wire-format bytes using the given color order.
    /// The caller must ensure `out.len() >= Self::BYTES_PER_PIXEL`.
    fn encode(self, order: Self::Order, out: &mut [u8]);
}

/// 8-bit RGB pixel (3 bytes on the wire).
///
/// Used with WS2812B and WS2811 protocols.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    /// All channels off.
    pub const BLACK: Self = Self::new(0, 0, 0);
    /// All channels at maximum.
    pub const WHITE: Self = Self::new(u8::MAX, u8::MAX, u8::MAX);

    /// Creates a new RGB pixel.
    #[must_use]
    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}

/// 8-bit RGBW pixel with dedicated white channel (4 bytes on the wire).
///
/// Used with SK6812 RGBW protocol. The white channel is independent —
/// setting (0,0,0,255) produces pure white without engaging the RGB LEDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Rgbw {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub w: u8,
}

impl Rgbw {
    /// All channels off.
    pub const BLACK: Self = Self::new(0, 0, 0, 0);
    /// All channels at maximum.
    pub const WHITE: Self = Self::new(u8::MAX, u8::MAX, u8::MAX, u8::MAX);

    /// Creates a new RGBW pixel.
    #[must_use]
    pub const fn new(r: u8, g: u8, b: u8, w: u8) -> Self {
        Self { r, g, b, w }
    }
}

/// 16-bit RGB pixel (6 bytes on the wire, big-endian per channel).
///
/// Used with WS2816 protocol for high dynamic range (65536 levels per channel).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Rgb16 {
    pub r: u16,
    pub g: u16,
    pub b: u16,
}

impl Rgb16 {
    /// All channels off.
    pub const BLACK: Self = Self::new(0, 0, 0);
    /// All channels at maximum.
    pub const WHITE: Self = Self::new(u16::MAX, u16::MAX, u16::MAX);

    /// Creates a new 16-bit RGB pixel.
    #[must_use]
    pub const fn new(r: u16, g: u16, b: u16) -> Self {
        Self { r, g, b }
    }
}

impl LedPixel for Rgb {
    type Order = RgbOrder;

    const KIND: PixelKind = PixelKind::Rgb;
    const BYTES_PER_PIXEL: usize = 3;

    fn encode(self, order: Self::Order, out: &mut [u8]) {
        debug_assert!(out.len() >= Self::BYTES_PER_PIXEL);

        match order {
            RgbOrder::Rgb => {
                out[0] = self.r;
                out[1] = self.g;
                out[2] = self.b;
            }
            RgbOrder::Grb => {
                out[0] = self.g;
                out[1] = self.r;
                out[2] = self.b;
            }
        }
    }
}

impl LedPixel for Rgbw {
    type Order = RgbwOrder;

    const KIND: PixelKind = PixelKind::Rgbw;
    const BYTES_PER_PIXEL: usize = 4;

    fn encode(self, order: Self::Order, out: &mut [u8]) {
        debug_assert!(out.len() >= Self::BYTES_PER_PIXEL);

        match order {
            RgbwOrder::Rgbw => {
                out[0] = self.r;
                out[1] = self.g;
                out[2] = self.b;
                out[3] = self.w;
            }
            RgbwOrder::Grbw => {
                out[0] = self.g;
                out[1] = self.r;
                out[2] = self.b;
                out[3] = self.w;
            }
        }
    }
}

impl LedPixel for Rgb16 {
    type Order = Rgb16Order;

    const KIND: PixelKind = PixelKind::Rgb16;
    const BYTES_PER_PIXEL: usize = 6;

    fn encode(self, order: Self::Order, out: &mut [u8]) {
        debug_assert!(out.len() >= Self::BYTES_PER_PIXEL);

        // 16-bit values are encoded as big-endian byte pairs per channel
        // (matches WS2816 wire format: R[15:8] R[7:0] G[15:8] ...)
        let [r0, r1] = self.r.to_be_bytes();
        let [g0, g1] = self.g.to_be_bytes();
        let [b0, b1] = self.b.to_be_bytes();

        match order {
            Rgb16Order::Rgb => {
                out[0] = r0;
                out[1] = r1;
                out[2] = g0;
                out[3] = g1;
                out[4] = b0;
                out[5] = b1;
            }
            Rgb16Order::Grb => {
                out[0] = g0;
                out[1] = g1;
                out[2] = r0;
                out[3] = r1;
                out[4] = b0;
                out[5] = b1;
            }
        }
    }
}

/// Sealed trait pattern — prevents external crates from implementing `LedPixel`.
/// New pixel types must be added in `pixel.rs` alongside the protocol impl matrix.
mod private {
    use super::{Rgb, Rgb16, Rgbw};

    pub trait Sealed {}

    impl Sealed for Rgb {}
    impl Sealed for Rgb16 {}
    impl Sealed for Rgbw {}
}

#[cfg(test)]
mod tests {
    use super::{LedPixel, PixelKind, Rgb, Rgb16, Rgb16Order, RgbOrder, Rgbw, RgbwOrder};

    // ── Constructor & constants ──────────────────────────────────────

    #[test]
    fn rgb_new_stores_channels() {
        let p = Rgb::new(0x10, 0x20, 0x30);
        assert_eq!(p.r, 0x10);
        assert_eq!(p.g, 0x20);
        assert_eq!(p.b, 0x30);
    }

    #[test]
    fn rgb_black_is_all_zero() {
        assert_eq!(Rgb::BLACK, Rgb::new(0, 0, 0));
    }

    #[test]
    fn rgb_white_is_all_max() {
        assert_eq!(Rgb::WHITE, Rgb::new(255, 255, 255));
    }

    #[test]
    fn rgb_default_is_black() {
        assert_eq!(Rgb::default(), Rgb::BLACK);
    }

    #[test]
    fn rgbw_new_stores_channels() {
        let p = Rgbw::new(0x10, 0x20, 0x30, 0x40);
        assert_eq!(p.r, 0x10);
        assert_eq!(p.g, 0x20);
        assert_eq!(p.b, 0x30);
        assert_eq!(p.w, 0x40);
    }

    #[test]
    fn rgbw_black_is_all_zero() {
        assert_eq!(Rgbw::BLACK, Rgbw::new(0, 0, 0, 0));
    }

    #[test]
    fn rgbw_white_is_all_max() {
        assert_eq!(Rgbw::WHITE, Rgbw::new(255, 255, 255, 255));
    }

    #[test]
    fn rgbw_default_is_black() {
        assert_eq!(Rgbw::default(), Rgbw::BLACK);
    }

    #[test]
    fn rgb16_new_stores_channels() {
        let p = Rgb16::new(0x1122, 0x3344, 0x5566);
        assert_eq!(p.r, 0x1122);
        assert_eq!(p.g, 0x3344);
        assert_eq!(p.b, 0x5566);
    }

    #[test]
    fn rgb16_black_is_all_zero() {
        assert_eq!(Rgb16::BLACK, Rgb16::new(0, 0, 0));
    }

    #[test]
    fn rgb16_white_is_all_max() {
        assert_eq!(Rgb16::WHITE, Rgb16::new(65535, 65535, 65535));
    }

    #[test]
    fn rgb16_default_is_black() {
        assert_eq!(Rgb16::default(), Rgb16::BLACK);
    }

    // ── Encode: all color orders ────────────────────────────────────

    #[test]
    fn rgb_encode_respects_rgb_order() {
        let mut out = [0_u8; 3];
        Rgb::new(0x11, 0x22, 0x33).encode(RgbOrder::Rgb, &mut out);
        assert_eq!(out, [0x11, 0x22, 0x33]);
    }

    #[test]
    fn rgb_encode_respects_grb_order() {
        let mut out = [0_u8; 3];
        Rgb::new(0x11, 0x22, 0x33).encode(RgbOrder::Grb, &mut out);
        assert_eq!(out, [0x22, 0x11, 0x33]);
    }

    #[test]
    fn rgbw_encode_respects_rgbw_order() {
        let mut out = [0_u8; 4];
        Rgbw::new(0x11, 0x22, 0x33, 0x44).encode(RgbwOrder::Rgbw, &mut out);
        assert_eq!(out, [0x11, 0x22, 0x33, 0x44]);
    }

    #[test]
    fn rgbw_encode_respects_grbw_order() {
        let mut out = [0_u8; 4];
        Rgbw::new(0x11, 0x22, 0x33, 0x44).encode(RgbwOrder::Grbw, &mut out);
        assert_eq!(out, [0x22, 0x11, 0x33, 0x44]);
    }

    #[test]
    fn rgb16_encode_respects_rgb_order() {
        let mut out = [0_u8; 6];
        Rgb16::new(0x1122, 0x3344, 0x5566).encode(Rgb16Order::Rgb, &mut out);
        // big-endian: 0x1122 → [0x11, 0x22]
        assert_eq!(out, [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
    }

    #[test]
    fn rgb16_encode_respects_grb_order() {
        let mut out = [0_u8; 6];
        Rgb16::new(0x1122, 0x3344, 0x5566).encode(Rgb16Order::Grb, &mut out);
        assert_eq!(out, [0x33, 0x44, 0x11, 0x22, 0x55, 0x66]);
    }

    // ── KIND & BYTES_PER_PIXEL ──────────────────────────────────────

    #[test]
    fn pixel_kind_discriminants() {
        assert_eq!(Rgb::KIND, PixelKind::Rgb);
        assert_eq!(Rgbw::KIND, PixelKind::Rgbw);
        assert_eq!(Rgb16::KIND, PixelKind::Rgb16);
    }

    #[test]
    fn bytes_per_pixel_values() {
        assert_eq!(Rgb::BYTES_PER_PIXEL, 3);
        assert_eq!(Rgbw::BYTES_PER_PIXEL, 4);
        assert_eq!(Rgb16::BYTES_PER_PIXEL, 6);
    }
}
