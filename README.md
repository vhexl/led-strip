# led-strip

[![CI](https://github.com/visua/led-strip/workflows/CI/badge.svg)](https://github.com/visua/led-strip/actions)
[![codecov](https://codecov.io/gh/visua/led-strip/branch/main/graph/badge.svg)](https://codecov.io/gh/visua/led-strip)

A `no_std` driver for single-wire addressable LED strips
(WS2812B, SK6812, WS2811, WS2816) with compile-time buffer sizing
and zero heap allocations.

## Features

- **SPI backend** via `embedded-hal` — the only fully implemented backend today
- **RMT & PIO backends** — compile-time skeletons; full transmit/encode paths pending
- Typed pixel formats: `Rgb` (8-bit), `Rgbw` (8-bit + white), `Rgb16` (16-bit)
- Typed protocol presets: `Ws2812`, `Ws2812B`, `Ws2811`, `Sk6812`, `Ws2816`
- Compile-time capacity checks in `LedStrip::new`; hot-path `refresh` never fails on sizing
- SPI encoding plan validation at construction time (timing tolerance vs protocol spec)
- `invert_output` support for boards with external inverter circuits
- Destructure with `into_parts()` to reuse backends across multiple strips

## Usage

```rust
use led_strip::{LedStrip, LedStripConfig, SpiCodec, SpiBackend, SpiEncodingPlan, Rgb, Ws2812B};

// Type-alias to reduce turbofish noise
type MyStrip = LedStrip<Rgb, Ws2812B, SpiCodec, SpiBackend<MySpi>, 60, 1024>;

let config = LedStripConfig::ws2812b(60);
let codec = SpiCodec::for_protocol::<Rgb, Ws2812B>(SpiEncodingPlan::ws281x_3bit(), false).unwrap();
let backend = SpiBackend::new(spi); // spi: impl embedded_hal::spi::SpiBus<u8>

let mut strip = MyStrip::new(config, codec, backend).unwrap();

strip.fill(Rgb::new(255, 0, 0));
strip.refresh().unwrap();
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
