# ESP-32 Utilities

A collection of lightweight, "header-only" C++ utilities for IoT/embedded applications on ESP-32.

Originally developed for the Arduino framework :
- Some components may depend on Arduino (String, Serial, digitalWrite...)
- Some may use FreeRTOS or ESP-IDF primitives (peripherals, timers, tasks...)
- No external dependencies to the ESP-32 Arduino core

- `AsyncSerial` : a thread-safe UART library, offering a drop-in replacement to the `Serial` API with efficient management of concurrent access.
- `AsyncWire` : a thread-safe I2C library, offering a drop-in replacement to the `Wire` API with intuitive Wire sequences declaration.
- `LedBlinker`: a simple and flexible Arduino library for controlling LED blinking patterns with FreeRTOS support.
- `MultiLogger`: a thread-safe logging system allowing simple management of various log ouputs (UART, SD card, JSON...)
- `PulseCounter`: a high-performance pulse counting library using ESP32's hardware `PCNT` module & timer-based polling
- `SerialConverter`: a simple utility class for converting between raw bytes and common serial formats (Hex, Base64)
- `SimpleTimer`: a lightweight and versatile timer class for ESP32

Most utilities could be easily transposed to other hardware platforms or bare-metal C++ if required.

See individual components for specific requirements and documentation.

## License

MIT License