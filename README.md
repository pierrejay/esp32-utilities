# Embedded Utilities

A collection of lightweight, header-only C++ utilities for embedded systems & IoT.

Originally developed for ESP32 MCU on Arduino framework :
- Some components may depend on Arduino (String, Serial, digitalWrite...)
- Those with the "-esp32" prefix have dependencies on either FreeRTOS or ESP-IDF primitives

However, most utilities have minimal dependencies and could be easily transposed to other hardware platforms or bare-metal C++.

See individual components for specific requirements and documentation.

## License

MIT License