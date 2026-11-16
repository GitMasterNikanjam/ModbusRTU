#pragma once

/**
 * @file ModbusLinuxRS485.h
 * @brief Modbus RTU (RS-485) master implementation for Linux serial ports.
 *
 * This library provides a lightweight Modbus RTU master suitable for communicating
 * with Modbus slave devices over an RS-485 serial interface on Linux.
 *
 * @par Addressing convention
 * All public API addresses (coils, discrete inputs, input registers, holding registers)
 * are **0-based Modbus protocol addresses**:
 * - Holding register "40001" in documentation corresponds to address `0`.
 * - Input register  "30001" in documentation corresponds to address `0`.
 * - Coil            "00001" in documentation corresponds to address `0`.
 * - Discrete input  "10001" in documentation corresponds to address `0`.
 *
 * Some implementations optionally normalize "4xxxx/3xxxx/0xxxx/1xxxx" style addresses;
 * however, the contract of this header is strictly 0-based addresses.
 *
 * @par Protocol
 * This is **Modbus RTU** (binary, CRC-based) over serial. It is not Modbus TCP.
 *
 * @par Error reporting
 * On failure, functions typically return `false` or an empty vector and set
 * @ref Modbus::errorMessage with a human-readable error explanation.
 */

// ######################################################################################
// Includes
// ######################################################################################

/* Match the platform convention used by Serial_STM32. */
#if defined(__linux__) || defined(_WIN32) && !defined(__CC_ARM) && !defined(__ARMCOMPILER_VERSION)
    #define _PLATFORM_PC_
#endif

#include <cstdint>
#include <cstddef>
#include <cstring>

#if defined(_PLATFORM_PC_)
    #include <vector>
    #include <string>
    template <typename T> using ModbusBuffer = std::vector<T>;
#else
    template <typename T> struct ModbusBufferCapacity { enum { value = 256 }; };
    template <> struct ModbusBufferCapacity<uint8_t> { enum { value = 256 }; };
    template <> struct ModbusBufferCapacity<uint16_t> { enum { value = 123 }; };
    template <> struct ModbusBufferCapacity<bool> { enum { value = 1968 }; };

    /** Fixed-capacity, allocation-free vector-like buffer used by the MCU backend. */
    template <typename T> class ModbusBuffer
    {
    public:
        typedef T* iterator;
        typedef const T* const_iterator;
        ModbusBuffer() : size_(0) {}
        template <typename It> ModbusBuffer(It first, It last) : size_(0) { insert(end(), first, last); }
        size_t size() const { return size_; }
        bool empty() const { return size_ == 0; }
        T* data() { return data_; }
        const T* data() const { return data_; }
        iterator begin() { return data_; }
        const_iterator begin() const { return data_; }
        iterator end() { return data_ + size_; }
        const_iterator end() const { return data_ + size_; }
        T& operator[](size_t i) { return data_[i]; }
        const T& operator[](size_t i) const { return data_[i]; }
        void reserve(size_t) {}
        void clear() { size_ = 0; }
        void push_back(const T& v) { if (size_ < capacity()) data_[size_++] = v; }
        void resize(size_t n) { resize(n, T()); }
        void resize(size_t n, const T& value) { if (n > capacity()) n = capacity(); while (size_ < n) data_[size_++] = value; size_ = n; }
        iterator insert(iterator pos, const T& value) { size_t i = static_cast<size_t>(pos - begin()); if (size_ >= capacity()) return end(); for (size_t j=size_; j>i; --j) data_[j]=data_[j-1]; data_[i]=value; ++size_; return begin()+i; }
        template <typename It> iterator insert(iterator pos, It first, It last) { size_t i=static_cast<size_t>(pos-begin()); while(first!=last && size_<capacity()) { insert(begin()+i, *first++); ++i; } return begin()+i; }
        static size_t capacity() { return ModbusBufferCapacity<T>::value; }
    private:
        T data_[ModbusBufferCapacity<T>::value];
        size_t size_;
    };
#endif

#if !defined(_PLATFORM_PC_)
    #include "Serial.h"
#endif

// ######################################################################################
// Modbus class
// ######################################################################################

/**
 * @class Modbus
 * @brief Modbus RTU master utility for RS-485 serial devices.
 *
 * This class provides a minimal abstraction for Modbus RTU master operations
 * over a Linux serial port (e.g. `/dev/ttyUSB0`, `/dev/ttyS0`).
 *
 * Supported operations include reading and writing coils and registers using
 * common Modbus function codes.
 *
 * @note
 * - Modbus RTU requires correct serial framing (baud, parity, stop bits).
 * - CRC handling is performed internally by the implementation.
 * - Public API addresses are 0-based Modbus protocol addresses (see file header).
 */
class Modbus 
{
    public:

        /**
         * @brief Human-readable message describing the last error.
         *
         * This member is updated whenever an operation fails. It may contain:
         * - system errors (e.g. failing to open the serial device, I/O errors),
         * - protocol errors (CRC mismatch, unexpected function code, Modbus exception response),
         * - validation errors (invalid quantity, invalid address range, etc.).
         *
         * @warning Not thread-safe: if multiple threads call methods concurrently,
         *          errorMessage may be overwritten.
         */
        static constexpr uint16_t ERROR_MESSAGE_CAPACITY = 96;
        char errorMessage[ERROR_MESSAGE_CAPACITY];

        /**
         * @brief Serial parity configuration.
         *
         * Typical Modbus RTU defaults are 8E1 (Even parity) or 8N1 (No parity),
         * depending on the slave device.
         */
        enum class Parity : uint8_t
        {
            None,  /**< No parity bit (8N1/8N2). */
            Even,  /**< Even parity (8E1/8E2). */
            Odd    /**< Odd parity (8O1/8O2). */
        };

        /**
         * @struct ParametersStructure
         * @brief Serial communication configuration parameters.
         *
         * Configure these parameters before calling @ref openPort().
         *
         * @note The implementation must translate BAUDRATE / parity / stop bits / data bits
         *       into Linux termios configuration.
         */
        struct ParametersStructure
        {
            /**
             * @brief Serial port device path (e.g., "/dev/ttyUSB0", "/dev/ttyS0").
             *
             * @note Example: "/dev/ttyUSB0"
             */
            #if defined(_PLATFORM_PC_)
                std::string PORT;
            #endif

            /**
             * @brief Baud rate for serial communication.
             *
             * Common Modbus RTU baud rates include: 9600, 19200, 38400, 57600, 115200.
             * The implementation should reject unsupported baud rates.
             */
            uint32_t BAUDRATE;

            /**
             * @brief Optional delay after transmitting a request, in microseconds (µs).
             *
             * Some slave devices require a short delay after receiving a request before
             * they begin transmitting the response (or some USB/RS-485 adapters benefit
             * from a short settling time).
             *
             * Set to 0 to disable.
             */
            uint32_t TRANSMIT_DELAY_US;

            /**
             * @brief Total timeout for receiving a complete Modbus RTU response, in milliseconds.
             *
             * This timeout should apply to the entire response (not per read() call),
             * including variable-length responses.
             */
            uint32_t RESPONSE_TIMEOUT_MS;

            /**
             * @brief Serial parity mode (None/Even/Odd).
             */
            Parity PARITY;

            /**
             * @brief Number of stop bits (1 or 2).
             */
            uint8_t STOP_BITS;  

            /**
             * @brief Number of data bits (typically 8).
             */
            uint8_t DATA_BITS;   
        }parameters;

        /**
         * @brief Construct a Modbus communication object.
         *
         * The serial port is not opened automatically. Configure @ref parameters
         * and then call @ref openPort().
         */
        Modbus();

        #if !defined(_PLATFORM_PC_)
            /** Attach project Serial object and RS-485 direction GPIO. TX level means transmit. */
            bool begin(Serial* serial, GPIO_TypeDef* directionPort, uint16_t directionPin,
                        GPIO_PinState transmitLevel = GPIO_PIN_SET);
        #endif
        
        /**
         * @brief Destructor.
         *
         * Automatically closes the serial port if it is open.
         */
        ~Modbus();

        // Resource-owning type: prevent accidental copying.
        Modbus(const Modbus&) = delete;
        Modbus& operator=(const Modbus&) = delete;

        // Allow moving if you want to transfer ownership safely.
        Modbus(Modbus&&) noexcept;
        Modbus& operator=(Modbus&&) noexcept;

        /**
         * @brief Open and configure the RS-485 serial port.
         *
         * This method opens the device specified by @ref parameters.PORT and configures
         * the port according to the remaining serial parameters.
         *
         * @return true if the port was successfully opened and configured.
         * @return false on failure; see @ref errorMessage.
         */
        bool openPort();

        /**
         * @brief Close the serial port if it is open.
         *
         * After calling this, @ref isPortOpen() will return false.
         */
        void closePort() noexcept;

        /**
         * @brief Check whether the serial port is open.
         *
         * @return true if the serial port file descriptor is valid.
         * @return false if the port is not opened.
         */
        bool isPortOpen() const noexcept;

        /** Copy error text into the fixed-size, always null-terminated error buffer. */
        void setError(const char* message) noexcept;

        // -------------------------------------------------------------------------
        // Modbus Read Operations
        // -------------------------------------------------------------------------

        /**
         * @brief Read coil status (Function Code 0x01).
         *
         * Coils are single-bit read/write discrete outputs.
         *
         * @param slaveID Modbus slave address (1..247 typically; 0 is broadcast for writes only).
         * @param starting_address First coil address (0-based).
         * @param quantity Number of coils to read (1..2000 per Modbus spec).
         *
         * @return A vector containing the **data bytes** from the response.
         *         Bits are packed LSB-first per Modbus specification.
         *         Returns an empty vector on failure and sets @ref errorMessage.
         */
        ModbusBuffer<uint8_t> readCoils(uint8_t slaveID, uint16_t starting_address, uint16_t quantity);

        /**
         * @brief Read discrete inputs (Function Code 0x02).
         *
         * Discrete inputs are single-bit read-only inputs.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First discrete input address (0-based).
         * @param quantity Number of discrete inputs to read (1..2000 per Modbus spec).
         *
         * @return A vector containing the **data bytes** from the response.
         *         Bits are packed LSB-first per Modbus specification.
         *         Returns an empty vector on failure and sets @ref errorMessage.
         */
        ModbusBuffer<uint8_t> readDiscreteInputs(uint8_t slaveID, uint16_t starting_address, uint16_t quantity);

        /**
         * @brief Read holding registers (Function Code 0x03).
         *
         * Holding registers are 16-bit read/write registers commonly used for configuration
         * and writable outputs.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First holding register address (0-based).
         * @param quantity Number of registers to read (1..125 per Modbus spec).
         *
         * @return A vector containing ONLY the register data bytes:
         *         length = `quantity * 2`, big-endian per register (Hi byte then Lo byte).
         *         Returns an empty vector on failure and sets @ref errorMessage.
         */
        ModbusBuffer<uint8_t> readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity);

        /**
         * @brief Read holding registers (Function Code 0x03) into a caller-provided buffer.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First holding register address (0-based).
         * @param quantity Number of registers to read (1..125 per Modbus spec).
         * @param buffer Output buffer of length at least `quantity` (uint16_t words).
         *
         * @return true on success; false on failure (see @ref errorMessage).
         *
         * @note The implementation should decode Modbus big-endian register bytes into
         *       host-order uint16_t values.
         */
        bool readHoldingRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity, uint16_t* buffer);

        /**
         * @brief Read input registers (Function Code 0x04).
         *
         * Input registers are 16-bit read-only registers commonly used for sensor/measurement values.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First input register address (0-based).
         * @param quantity Number of registers to read (1..125 per Modbus spec).
         *
         * @return A vector containing ONLY the register data bytes:
         *         length = `quantity * 2`, big-endian per register (Hi byte then Lo byte).
         *         Returns an empty vector on failure and sets @ref errorMessage.
         */
        ModbusBuffer<uint8_t> readInputRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity);

        /**
         * @brief Read input registers (Function Code 0x04) into a caller-provided buffer.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First input register address (0-based).
         * @param quantity Number of registers to read (1..125 per Modbus spec).
         * @param buffer Output buffer of length at least `quantity` (uint16_t words).
         *
         * @return true on success; false on failure (see @ref errorMessage).
         */
        bool readInputRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity, uint16_t* buffer);

        // -------------------------------------------------------------------------
        // Modbus Write Operations
        // -------------------------------------------------------------------------

        /**
         * @brief Write a single coil (Function Code 0x05).
         *
         * Writes the ON/OFF state of a single coil.
         *
         * @param slaveID Modbus slave address.
         * @param address Coil address (0-based).
         * @param state Desired coil state (`true` = ON, `false` = OFF).
         *
         * @return true on success; false on failure (see @ref errorMessage).
         *
         * @note Modbus encoding uses 0xFF00 for ON and 0x0000 for OFF in the protocol.
         */
        bool writeSingleCoil(uint8_t slaveID, uint16_t address, bool state);

        /**
         * @brief Write a single holding register (Function Code 0x06).
         *
         * Writes one 16-bit value to a holding register.
         *
         * @param slaveID Modbus slave address.
         * @param address Holding register address (0-based).
         * @param value 16-bit value to write.
         *
         * @return true on success; false on failure (see @ref errorMessage).
         *
         * @note The normal response echoes the address and value. Implementations should
         *       validate this echo and CRC.
         */
        bool writeSingleRegister(uint8_t slaveID, uint16_t address, uint16_t value);

        /**
         * @brief Write multiple coils (Function Code 0x0F).
         *
         * Writes multiple consecutive coils. Coil values are provided as booleans in @p values.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First coil address (0-based).
         * @param values Vector of coil states to write.
         *
         * @return true on success; false on failure (see @ref errorMessage).
         *
         * @note Modbus packs coil bits LSB-first into bytes. The implementation must pack
         *       @p values accordingly.
         */
        bool writeMultipleCoils(uint8_t slaveID, uint16_t starting_address, const ModbusBuffer<bool>& values);

        /**
         * @brief Write multiple holding registers (Function Code 0x10).
         *
         * Writes multiple consecutive 16-bit values to holding registers.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First holding register address (0-based).
         * @param values Vector of register values to write.
         *
         * @return true on success; false on failure (see @ref errorMessage).
         *
         * @note Modbus specifies a maximum of 123 registers per 0x10 request.
         */
        bool writeMultipleRegisters(uint8_t slaveID, uint16_t starting_address, const ModbusBuffer<uint16_t>& values);

        /**
         * @brief Write multiple holding registers (Function Code 0x10) using a raw array.
         *
         * This overload avoids allocations and is suitable for high-frequency writes.
         *
         * @param slaveID Modbus slave address.
         * @param starting_address First holding register address (0-based).
         * @param quantity Number of registers to write (must match the size of @p values).
         * @param values Pointer to an array of length at least @p quantity.
         *
         * @return true on success; false on failure (see @ref errorMessage).
         *
         * @note Modbus specifies a maximum of 123 registers per 0x10 request.
         */
        bool writeMultipleRegisters(uint8_t slaveID, uint16_t starting_address, uint16_t quantity, const uint16_t* values);

        // -------------------------------------------------------------------------
        // Utility
        // -------------------------------------------------------------------------

        /**
         * @brief Print a byte vector in hexadecimal format.
         *
         * Useful for debugging Modbus frames and responses.
         *
         * @param data Vector of bytes to print.
         *
         * @note Consider making this function static if it does not access object state.
         */
        static void printHex(const ModbusBuffer<uint8_t>& data); 

    private:

        /**
         * @brief File descriptor for the RS-485 serial port.
         *
         * A value < 0 indicates that the port is not open.
         */
        int _serialPort{-1};
        #if !defined(_PLATFORM_PC_)
            Serial* _serial{nullptr};
            GPIO_TypeDef* _directionPort{nullptr};
            uint16_t _directionPin{0};
            GPIO_PinState _transmitLevel{GPIO_PIN_SET};
        #endif

        /**
         * @brief Compute Modbus RTU CRC16 over a byte vector.
         * @param data Input frame bytes (excluding CRC bytes).
         * @return CRC16 value.
         */
        static uint16_t _crc16_modbus(const ModbusBuffer<uint8_t>& data);

        /**
         * @brief Compute Modbus RTU CRC16 over a byte buffer.
         * @param data Pointer to bytes.
         * @param len Number of bytes.
         * @return CRC16 value.
         */
        static uint16_t _crc16_modbus(const uint8_t* data, size_t len);

        /**
         * @brief Verify Modbus RTU CRC of a received frame.
         *
         * @param frame Complete frame including CRC bytes.
         * @return true if CRC is valid; false otherwise.
         */
        static bool _verifyCrc(const ModbusBuffer<uint8_t>& frame);

        /**
         * @brief Append Modbus RTU CRC16 to the end of a frame.
         *
         * CRC is appended in little-endian wire order: CRC Low byte, then CRC High byte.
         *
         * @param frame Frame to append CRC onto.
         */
        static void _appendCrc(ModbusBuffer<uint8_t>& frame);

        // -------------------------------------------------------------------------
        // Address normalization helpers (optional)
        // -------------------------------------------------------------------------
        
        /**
         * @brief Normalize a holding register address.
         *
         * Intended to optionally convert documented 4xxxx-style addresses into 0-based
         * protocol addresses. Exact behavior depends on implementation.
         *
         * @param addr Address value provided by caller.
         * @return Normalized 0-based address.
         */
        static uint16_t _normalizeHoldingRegisterAddress(uint16_t addr);

        /**
         * @brief Normalize an input register address (optional 3xxxx-style to 0-based).
         * @param addr Address value provided by caller.
         * @return Normalized 0-based address.
         */
        static uint16_t _normalizeInputRegisterAddress(uint16_t addr);

        /**
         * @brief Normalize a coil address (optional 0xxxx-style to 0-based).
         * @param addr Address value provided by caller.
         * @return Normalized 0-based address.
         */
        static uint16_t _normalizeCoilAddress(uint16_t addr);

        /**
         * @brief Normalize a discrete input address (optional 1xxxx-style to 0-based).
         * @param addr Address value provided by caller.
         * @return Normalized 0-based address.
         */
        static uint16_t _normalizeDiscreteInputAddress(uint16_t addr);

        // -------------------------------------------------------------------------
        // Serial I/O helpers
        // -------------------------------------------------------------------------

        /**
         * @brief Send a complete Modbus RTU frame over the serial port.
         *
         * @param data Vector containing the full Modbus RTU frame (including CRC).
         * @return true if transmission succeeded; false otherwise (see @ref errorMessage).
         */
        bool _serialSend(const ModbusBuffer<uint8_t>& data);

        /**
         * @brief Send a complete Modbus RTU frame over the serial port.
         *
         * @param data Pointer to the full Modbus RTU frame (including CRC).
         * @param len Length in bytes.
         * @return true if transmission succeeded; false otherwise (see @ref errorMessage).
         */
        bool _serialSend(const uint8_t* data, size_t len);

        /**
         * @brief Read exactly @p n bytes from the serial port within a timeout.
         *
         * This helper is typically used by RTU framing code that knows the expected
         * number of bytes to read next (e.g., fixed-length write responses or reading
         * header bytes to determine a variable payload length).
         *
         * @param n Number of bytes to read.
         * @param timeout_ms Total timeout in milliseconds.
         *
         * @return Vector containing exactly @p n bytes on success.
         *         Returns an empty vector on failure (timeout or I/O error) and sets
         *         @ref errorMessage.
         */
        ModbusBuffer<uint8_t> _serialReceive(size_t n, uint32_t timeout_ms = 1000);

        /**
         * @brief Read up to @p expected_length bytes into a caller-provided buffer.
         *
         * @param expected_length Number of bytes requested.
         * @param buffer Output buffer (must be at least expected_length bytes).
         *
         * @return Number of bytes actually read (0 indicates timeout / no data, depending on implementation).
         *
         * @warning This overload is lower-level and may be less safe than the vector-based
         *          exact-length receive. Prefer using the vector-based API for RTU framing.
         */
        size_t _serialReceive(size_t expected_length, uint8_t* buffer);

        // -------------------------------------------------------------------------
        // RTU response handling
        // -------------------------------------------------------------------------

        /**
         * @brief Receive a Modbus RTU response and return the full response frame.
         *
         * This function supports:
         * - Fixed-length responses (typical for write single/multiple operations).
         * - Variable-length responses where the third byte is a byte-count (typical for reads).
         *
         * It should:
         * - enforce a total timeout for the entire response,
         * - validate slave ID and function code,
         * - detect Modbus exception responses (function|0x80),
         * - validate CRC before returning.
         *
         * @param expected_slave Expected slave address in the response.
         * @param expected_function Expected function code in the response.
         * @param expected_fixed_len If non-zero, treat response as fixed length and read exactly this many bytes.
         *                           If zero, treat response as variable length and use byte-count to determine length.
         * @param timeout_ms Total timeout for reading the full response.
         *
         * @return Full Modbus RTU response frame including CRC on success.
         *         Returns empty vector on failure and sets @ref errorMessage.
         */
        ModbusBuffer<uint8_t> _receiveRtuResponse(uint8_t expected_slave, uint8_t expected_function, size_t expected_fixed_len, uint32_t timeout_ms);
        
        /**
         * @brief Validate slave ID, function code, and exception responses (basic checks).
         *
         * This helper typically checks:
         * - response length sanity,
         * - slave ID match,
         * - function match OR exception (function|0x80),
         * - if exception, populates @ref errorMessage with the exception code.
         *
         * @param resp Full RTU response frame (including CRC bytes).
         * @param expected_slave Expected slave address.
         * @param expected_function Expected function code.
         *
         * @return true if basic validation succeeds; false otherwise.
         */
        bool _validateBasicResponse(const ModbusBuffer<uint8_t>& resp, uint8_t expected_slave, uint8_t expected_function);
};

// ##############################################################################
