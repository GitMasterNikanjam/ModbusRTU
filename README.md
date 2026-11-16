# ModbusRS485 (Modbus RTU Master for Linux and STM32)

A lightweight **Modbus RTU (RS-485) master** for Linux serial ports and STM32Cube HAL UARTs.
It supports common Modbus function codes for reading/writing coils and registers, performs CRC16 checks internally, and returns clear error messages via `Modbus::errorMessage`.

---

## Features

- ✅ Modbus RTU master (binary RTU frames + CRC16)
- ✅ Linux serial devices: `/dev/ttyUSB0`, `/dev/ttyS0`, `/dev/ttyAMA0`, etc.
- ✅ Read:
  - Coils (0x01)
  - Discrete Inputs (0x02)
  - Holding Registers (0x03)
  - Input Registers (0x04)
- ✅ Write:
  - Single Coil (0x05)
  - Single Register (0x06)
  - Multiple Coils (0x0F)
  - Multiple Registers (0x10)
- ✅ Timeouts + CRC verification + Modbus exception detection
- ✅ Move-only (`Modbus` cannot be copied, but can be moved)

---

## Platform selection

The header defines `_PLATFORM_PC_` automatically using the project's compiler check. Linux uses POSIX/termios I/O. STM32 builds use the project's `Serial` class; no platform selection define is required from the application.

### STM32 RS-485 wiring and setup

Connect the MCU UART TX/RX to the RS-485 transceiver TXD/RXD. Connect one MCU GPIO to the transceiver direction input (DE and `/RE` tied together if the transceiver requires both). The library drives that GPIO to `transmitLevel` during TX and to the opposite level for RX.

The UART must already be initialized by the application. Configure the project `Serial` object for the selected UART and call:

```cpp
Serial rs485Serial;
Modbus modbus;

// Set the application-owned HAL UART; begin() configures Serial in blocking mode.
rs485Serial.setUart(&huart2);
modbus.parameters.BAUDRATE = 115200; // Serial_STM32 supports 9600, 57600, 115200
modbus.begin(&rs485Serial, RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
```

Set parity, word length, and stop bits in the Cube/HAL UART configuration to match the slave. Use blocking TX/RX modes; `Modbus::begin()` selects those modes on the supplied `Serial` object. On STM32, `openPort()` is retained for API compatibility and returns the state established by `begin()`.

---

## Files

Typical layout:
```

ModbusRS485.h
ModbusRS485.cpp
examples/
ex1.cpp

```

---

## Addressing

Every address argument in the public API is the exact **zero-based PDU address** sent in the Modbus request. The library does not normalize or convert manual/reference numbers. The function code selects the data table (for example, `0x03` selects Holding Registers); a notation prefix such as `4` in `40001` is not part of the address transmitted on the bus.

Many device manuals use one-based reference numbers such as `40001` for the first Holding Register. In that convention, subtract the table's first reference number to obtain the PDU address: `40001` → `0`, `40002` → `1`, and `40011` → `10`. For example, if a manual labels a Holding Register `40002`, call `readHoldingRegisters(slaveID, 1, quantity, ...)`.

Do not assume every vendor uses the same notation or starting offset. Check the device's Modbus examples/register map, then pass the actual PDU address. Some manuals already show zero-based addresses, in which case do not subtract one. Coil and discrete-input references must be interpreted in their own tables in the same way.
## Build

### Simple compile (example)
```bash
mkdir -p bin
g++ -std=c++17 -O2 -Wall -Wextra examples/ex1.cpp ModbusRS485.cpp -o bin/ex1
sudo ./bin/ex1
````

> Note: You may not need `sudo` if your user has permission to access the serial device.
> Consider adding your user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
# log out and back in
```

---

## Quick Start

### 1) Configure and open the port

```cpp
#include "ModbusRS485.h"
#include <iostream>

int main() {
    Modbus modbus;

    modbus.parameters.PORT = "/dev/ttyUSB0";
    modbus.parameters.BAUDRATE = 115200;
    modbus.parameters.PARITY = Modbus::Parity::None;   // or Even/Odd
    modbus.parameters.STOP_BITS = 1;
    modbus.parameters.DATA_BITS = 8;

    modbus.parameters.TRANSMIT_DELAY_US = 3000;        // optional bus turnaround delay
    modbus.parameters.RESPONSE_TIMEOUT_MS = 1000;      // total response timeout

    if (!modbus.openPort()) {
        std::cerr << "Failed to open port: " << modbus.errorMessage << "\n";
        return 1;
    }

    // use modbus...
    return 0;
}
```

---

## Reading Registers

### Read holding registers (0x03)

`readHoldingRegisters()` returns **data bytes only** (`quantity * 2` bytes), big-endian per register.

```cpp
uint8_t slave = 1;

// Example: read 2 registers starting at address 0 (i.e., 40001 in docs)
auto data = modbus.readHoldingRegisters(slave, 0, 2);

if (data.empty()) {
    std::cerr << "Read failed: " << modbus.errorMessage << "\n";
} else {
    // data = [R0_hi, R0_lo, R1_hi, R1_lo]
    uint16_t r0 = (uint16_t(data[0]) << 8) | data[1];
    uint16_t r1 = (uint16_t(data[2]) << 8) | data[3];
    std::cout << "R0=" << r0 << " R1=" << r1 << "\n";
}
```

### Read into a `uint16_t` buffer

```cpp
uint16_t regs[2];

if (!modbus.readHoldingRegisters(slave, 0, 2, regs)) {
    std::cerr << "Read failed: " << modbus.errorMessage << "\n";
} else {
    std::cout << "R0=" << regs[0] << " R1=" << regs[1] << "\n";
}
```

---

## Writing Registers

### Write single register (0x06)

```cpp
uint8_t slave = 1;

// Write value 0x0067 into holding register address 10 (i.e., 40011 in docs)
if (!modbus.writeSingleRegister(slave, 10, 0x0067)) {
    std::cerr << "Write failed: " << modbus.errorMessage << "\n";
}
```

### Write multiple registers (0x10)

```cpp
uint8_t slave = 1;

// Write two registers starting at address 20
std::vector<uint16_t> values = { 0x1234, 0xABCD };

if (!modbus.writeMultipleRegisters(slave, 20, values)) {
    std::cerr << "Write failed: " << modbus.errorMessage << "\n";
}
```

---

## Coils / Discrete Inputs

### Read coils (0x01) / read discrete inputs (0x02)

Returned data is **packed bits (LSB-first)**.

```cpp
uint8_t slave = 1;

// Read 10 coils starting at coil address 0
auto coilBytes = modbus.readCoils(slave, 0, 10);

if (coilBytes.empty()) {
    std::cerr << "Read coils failed: " << modbus.errorMessage << "\n";
} else {
    // Check coil #0 (bit0 of first byte)
    bool coil0 = (coilBytes[0] & 0x01) != 0;
    std::cout << "coil0=" << coil0 << "\n";
}
```

### Write a single coil (0x05)

```cpp
uint8_t slave = 1;

// Turn ON coil at address 0
if (!modbus.writeSingleCoil(slave, 0, true)) {
    std::cerr << "Write coil failed: " << modbus.errorMessage << "\n";
}
```

### Write multiple coils (0x0F)

```cpp
uint8_t slave = 1;

// Write 5 coils starting at address 0
std::vector<bool> coils = { true, false, true, true, false };

if (!modbus.writeMultipleCoils(slave, 0, coils)) {
    std::cerr << "Write multiple coils failed: " << modbus.errorMessage << "\n";
}
```

---

## Example: Fixing a Common User Example

If you have older sample code, be careful with parameter names.
This library uses:

* `TRANSMIT_DELAY_US` (microseconds)
* NOT `TRANSMIT_DELAY`

Correct example snippet:

```cpp
modbus.parameters.TRANSMIT_DELAY_US = 3000;
```

Also remember: returned vectors from reads typically contain **data bytes only** (not full RTU frame). So printing them is fine for debugging, but they will not include slave/function/CRC bytes.

---

## Error Handling

All operations set a human-readable error in:

* `modbus.errorMessage`

Typical errors include:

* Port open/config failure (`/dev/tty*` permission, bad baud rate, termios errors)
* Timeout while reading
* CRC mismatch
* Function code mismatch
* Slave ID mismatch
* Modbus exception response (function | 0x80)

Always check return values:

```cpp
auto data = modbus.readInputRegisters(1, 0, 4);
if (data.empty()) {
    std::cerr << modbus.errorMessage << "\n";
}
```

---

## Notes / Best Practices

* **RS-485 direction control:** Some adapters handle DE/RE automatically. Others may require kernel RS-485 mode or GPIO toggling (not handled here).
* **Timeout sizing:** Increase `RESPONSE_TIMEOUT_MS` for slow devices or long cable runs.
* **Bus turnaround delay:** `TRANSMIT_DELAY_US` can help with flaky adapters or strict slaves.
* **One master at a time:** Don’t open the same `/dev/tty*` from multiple processes.
* **Thread safety:** `errorMessage` is not thread-safe.

---

## License

Add your preferred license here (MIT/BSD/Apache-2.0/etc.).
Example:

* `MIT License` — permissive, commonly used for small libraries.

---

## Contributing

PRs and improvements welcome:

* Optional RS-485 ioctl support (`TIOCSRS485`) for UARTs that support it
* More examples (reading floats, int32, scaling, bit unpacking helpers)
* Unit tests for CRC and framing

---
