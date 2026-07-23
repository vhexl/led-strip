# led-strip

[English](README.md) | 简体中文

[![CI](https://github.com/vhexl/led-strip/actions/workflows/ci.yml/badge.svg)](https://github.com/vhexl/led-strip/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/vhexl/led-strip/branch/main/graph/badge.svg)](https://codecov.io/gh/vhexl/led-strip)

一个 `no_std` 的单线可寻址 LED 灯带驱动，支持
WS2812B、SK6812、WS2811、WS2816，具备编译期缓冲区容量约束，
且不使用堆内存分配。

## 功能特性

- 通过 `embedded-hal` 提供 **SPI 后端**，这是当前唯一完整实现的后端
- **RMT 与 PIO 后端**：当前为可编译骨架，完整发送/编码路径仍在推进中
- 类型化像素格式：`Rgb`（8-bit）、`Rgbw`（8-bit + W）、`Rgb16`（16-bit）
- 类型化协议预设：`Ws2812`、`Ws2812B`、`Ws2811`、`Sk6812`、`Ws2816`
- 在 `LedStrip::new` 中进行编译期容量边界相关校验；热路径 `refresh` 不再因容量不足失败
- 在构造期执行 SPI 编码方案校验（时序容差对照协议规范）
- `SpiEncodeError::InternalConsistency`：防止编码后溢出（正常逻辑下不会触发）
- 支持 `invert_output`，适配带外部反相器的硬件板卡
- 通过 `into_parts()` 解构组件，便于复用后端实例

## 用法示例

```rust
use led_strip::{LedStrip, LedStripConfig, SpiCodec, SpiBackend, SpiEncodingPlan, Rgb, Ws2812B};

// 使用 type alias 减少泛型书写噪音
type MyStrip = LedStrip<Rgb, Ws2812B, SpiCodec, SpiBackend<MySpi>, 60, 1024>;

let config = LedStripConfig::ws2812b(60);
let codec = SpiCodec::for_protocol::<Rgb, Ws2812B>(SpiEncodingPlan::ws281x_3bit(), false).unwrap();
let backend = SpiBackend::new(spi); // spi: impl embedded_hal::spi::SpiBus<u8>

let mut strip = MyStrip::new(config, codec, backend).unwrap();

strip.fill(Rgb::new(255, 0, 0));
strip.refresh().unwrap();
```

## 发布流程

本项目使用 [release-plz](https://release-plz.dev/) 自动化发布：

1. 每次推送到 `main` 分支，release-plz 会自动创建/更新一个 **Release PR**，
   包含版本号与基于 [约定式提交](https://www.conventionalcommits.org/) 生成的更新日志。
2. 合并 Release PR 后，release-plz 将自动：
   - 创建 git 标签（`led-strip-v<version>`）
   - 发布到 [crates.io](https://crates.io/crates/led-strip)
   - 创建 GitHub Release 及发布说明

## 许可证

你可以任选以下任一许可证：

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))
