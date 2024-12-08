# SerialConverter

A lightweight, header-only utility class for converting between raw bytes and common serial formats (Hex, Base64). Designed with a focus on efficiency and ease of use.

## Features

- **Header-only implementation** - just include and use
- **Standard C++ vectors** for safe and easy memory handling
- **Two-way conversions:**
  - Bytes ↔ ASCII Hex
  - Bytes ↔ Base64
- Clean handling of common edge cases (spaces, 0x prefix, padding)

## Usage

```cpp
#include "SerialConverter.h"

void example() {
    // Define some raw bytes
    Bytes rawData = {0x12, 0xAB, 0xFF};
    
    // Convert to Hex (result: "12 AB FF ")
    Bytes hexData = SerialConverter::bytesToHex(rawData);
    
    // Convert back to bytes
    Bytes originalData = SerialConverter::hexToBytes(hexData);
    
    // Base64 conversion
    Bytes base64Data = SerialConverter::bytesToBase64(rawData);
    Bytes backToOriginal = SerialConverter::base64ToBytes(base64Data);
}
```

## API Reference

### Hex Conversion
- `static Bytes bytesToHex(const Bytes& input)`
  - Converts raw bytes to ASCII hex representation
  - Each byte becomes two hex characters + space
  - Example: `{0xFF, 0x00}` → `"FF 00 "`

- `static Bytes hexToBytes(const Bytes& input)`
  - Converts hex string back to raw bytes
  - Automatically handles:
    - Optional "0x" prefix
    - Spaces between bytes
    - Case-insensitive input
    - Odd number of characters (adds leading zero)

### Base64 Conversion
- `static Bytes bytesToBase64(const Bytes& input)`
  - Converts raw bytes to Base64 representation
  - Handles proper padding with '=' characters

- `static Bytes base64ToBytes(const Bytes& input)`
  - Converts Base64 back to raw bytes
  - Automatically handles padding