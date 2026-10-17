# led-strip

English | [简体中文](README.zh-CN.md)

[![CI](https://github.com/vhexl/led-strip/actions/workflows/ci.yml/badge.svg)](https://github.com/vhexl/led-strip/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/vhexl/led-strip/branch/main/graph/badge.svg)](https://codecov.io/gh/vhexl/led-strip)

A `no_std` driver for single-wire addressable LED strips
(WS2812B, SK6812, WS2811, WS2816) with statically sized buffers
and zero heap allocations.

## Features

- **SPI backend** via `embedded-hal` — the only fully implemented backend today
- `WireCodec` / `TransportBackend` extension traits for custom backends; RMT and PIO implementations remain future work
- Typed pixel formats: `Rgb` (8-bit), `Rgbw` (8-bit + white), `Rgb16` (16-bit)
- Typed protocol presets: `Ws2812B`, `Ws2811`, `Sk6812`, `Ws2816`
- Runtime capacity checks on each `LedStrip::write` against the statically sized buffer; encode + transmit in one call
- SPI encoding plan validation at construction time (symbol shape, distinct 0/1 symbols, exact timing bounds and minimum bit period)
- Explicit `SpiEncodeError::InternalConsistency` for impossible post-check encode overflows
- `invert_output` support for boards with external inverter circuits
- Destructure with `into_parts()` to reuse backends across multiple strips

## Usage

```rust
use led_strip::{LedStrip, SpiCodec, SpiBackend, SpiEncodingPlan, Rgb, RgbOrder, Ws2812B};

// Type-alias to reduce turbofish noise
type MyStrip = LedStrip<Rgb, Ws2812B, SpiCodec<Rgb, Ws2812B>, SpiBackend<MySpi>, 1024>;

let codec = SpiCodec::<Rgb, Ws2812B>::for_protocol(SpiEncodingPlan::ws2812_3bit(), false).unwrap();
let backend = SpiBackend::new(spi); // spi: impl embedded_hal::spi::SpiBus<u8>

let mut strip = MyStrip::new(RgbOrder::Grb, codec, backend);

let pixels = [Rgb::new(255, 0, 0); 60];
strip.write(&pixels).unwrap();
```

For WS2816B/C strips using 48-bit GRB data, use `Rgb16`, `Rgb16Order::Grb`,
and `SpiEncodingPlan::ws2816_5bit()` with SPI configured to 4 MHz. This plan
uses `10000`/`11100` symbols and needs `30 * pixel_count + 150` transmit bytes.
The `Ws2816` marker validates each phase against the
[WS2816B/C datasheet ranges](https://www.tme.eu/Document/2bd0f26702fa63d599bec05f7e359338/WS2816B-2121.pdf)
and requires a bit period of at least 1250 ns. Use the actual peripheral clock
when constructing a custom plan. WS2816A gain-control frames are not implemented.

## Releasing

This project uses [release-plz](https://release-plz.dev/) to automate releases:

1. Every push to `main` triggers release-plz to open/update a **Release PR**
   with the correct version bumps and a changelog derived from
   [Conventional Commits](https://www.conventionalcommits.org/).
2. Merge the Release PR when ready — release-plz will then:
   - Create the git tag (`led-strip-v<version>`)
   - Publish to [crates.io](https://crates.io/crates/led-strip)
   - Create a GitHub release with release notes

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
