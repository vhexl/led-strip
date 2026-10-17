# led-strip

[![CI](https://github.com/vhexl/led-strip/actions/workflows/ci.yml/badge.svg)](https://github.com/vhexl/led-strip/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/vhexl/led-strip/branch/main/graph/badge.svg)](https://codecov.io/gh/vhexl/led-strip)
[![crates.io](https://img.shields.io/crates/v/led-strip.svg)](https://crates.io/crates/led-strip)
[![docs.rs](https://img.shields.io/docsrs/led-strip)](https://docs.rs/led-strip)
[![MSRV](https://img.shields.io/crates/msrv/led-strip)](https://github.com/vhexl/led-strip/blob/main/Cargo.toml)
[![License](https://img.shields.io/crates/l/led-strip.svg)](#许可证)

[English](README.md) | 简体中文

一个 `no_std` 的单线可寻址 LED 灯带驱动，支持
WS2812B、SK6812、WS2811、WS2816。缓冲区容量在编译期确定，
写入时检查容量是否足够，且不使用堆内存分配。

## 功能特性

- 通过 `embedded-hal` 提供 **SPI 后端**，这是当前唯一完整实现的后端
- 通过 `WireCodec` / `TransportBackend` trait 扩展自定义后端；RMT 与 PIO 后端尚未提供
- 类型化像素格式：`Rgb`（8-bit）、`Rgbw`（8-bit + W）、`Rgb16`（16-bit）
- 类型化协议预设：`Ws2812B`、`Ws2811`、`Sk6812`、`Ws2816`
- 每次调用 `LedStrip::write` 时，在运行期检查静态缓冲区容量，编码与传输一步完成
- 在构造期检查 SPI 符号形状、0/1 符号可区分性、精确时序上下限及最小位周期
- `SpiEncodeError::InternalConsistency`：防止编码后溢出（正常逻辑下不会触发）
- 支持 `invert_output`，适配带外部反相器的硬件板卡
- 通过 `into_parts()` 解构组件，便于复用后端实例

## 用法示例

```rust
use led_strip::{LedStrip, SpiCodec, SpiBackend, SpiEncodingPlan, Rgb, RgbOrder, Ws2812B};

// 使用 type alias 减少泛型书写噪音
type MyStrip = LedStrip<Rgb, Ws2812B, SpiCodec<Rgb, Ws2812B>, SpiBackend<MySpi>, 1024>;

let codec = SpiCodec::<Rgb, Ws2812B>::for_protocol(SpiEncodingPlan::ws2812_3bit(), false).unwrap();
let backend = SpiBackend::new(spi); // spi: impl embedded_hal::spi::SpiBus<u8>

let mut strip = MyStrip::new(RgbOrder::Grb, codec, backend);

let pixels = [Rgb::new(255, 0, 0); 60];
strip.write(&pixels).unwrap();
```

对于使用 48-bit GRB 数据的 WS2816B/C 灯带，使用 `Rgb16`、`Rgb16Order::Grb`
和 `SpiEncodingPlan::ws2816_5bit()`，并将 SPI 配置为 4 MHz。
该方案的 0/1 符号为 `10000`/`11100`，发送缓冲区需要
`30 * 像素数 + 150` 字节。`Ws2816` 按
[原厂手册](https://www.tme.eu/Document/2bd0f26702fa63d599bec05f7e359338/WS2816B-2121.pdf)
检查各相位上下限及至少 1250 ns 的位周期。自定义方案应填写外设实际时钟频率。
当前未实现 WS2816A 的增益控制帧。

## 发布流程

本项目使用 [release-plz](https://release-plz.dev/) 自动化发布：

1. 每次推送到 `main` 分支，release-plz 会自动创建/更新一个 **Release PR**，
   包含版本号与基于 [约定式提交](https://www.conventionalcommits.org/) 生成的更新日志。
2. 合并 Release PR 后，release-plz 将自动：
   - 创建 git 标签（`v<version>`）
   - 发布到 [crates.io](https://crates.io/crates/led-strip)
   - 创建 GitHub Release 及发布说明

## 许可证

你可以任选以下任一许可证：

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))
