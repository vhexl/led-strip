# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0](https://github.com/vhexl/led-strip/releases/tag/v0.1.0) - 2026-07-23

### Added

- add release-plz workflow and document release process
- parameterize SpiCodec with pixel/protocol types, add ws2811 plan, refine error handling
- add get method, seal protocol trait, and refine set/write error handling
- enhance SPI encoding with new error handling and validation

### Other

- add CHANGELOG.md
- *(ci)* enhance coverage report verification and path normalization
- update actions/checkout to v5 and codecov-action to v7 with improved settings
- update repository links and add license files
- update README.md and README.zh-CN.md4
- code format
- initial commit
