# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2021-02-02

### Changed

- Replace ``measure_result_t::distance`` by ``measure_result_t::delay`` field as external code can take into account
  extra data to calculate distance with better precision.
- Replace method `SimpleHCSR04Driver::measure_distance_async` by `SimpleHCSR04Driver::measure_delay_async`.

### Added

- Add ``SimpleHCSR04Driver::delay_to_distance_default`` method with base formula to calculate distance from an echo
  delay.
- Add ``SimpleHCSR04Driver::measure_delay`` method to measure delay synchronously.

### Fixed

- Add missed ``mbed_lib.json`` file.

## [0.1.0] - 2020-12-07

### Added

- Add base HC-SR04 driver
