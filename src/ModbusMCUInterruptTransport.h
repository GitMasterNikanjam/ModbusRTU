/**
 * @file ModbusMCUInterruptTransport.h
 * @brief Interrupt-driven UART transport for an MCU Modbus RTU bus.
 * @details Provides transmit, response reception, RS-485 direction control, RTU
 *          inter-frame timing, timeout handling, and optional owner-task hooks.
 *          The object owns its response buffer and is dedicated to one Modbus
 *          bus. It must remain alive while registered with a @ref Modbus object
 *          and while that UART can generate callbacks.
 * @note This transport is available only on MCU builds. It does not create
 *       Modbus frames or validate their CRC. Those operations remain in @ref Modbus.
 */
#pragma once

#include "ModbusRTU.h"

#if !defined(_PLATFORM_PC_)
    class ModbusMCUInterruptTransport
    {
        public:
            /**
             * @brief Scheduler callbacks used to suspend and wake the bus owner.
             * @details Register these hooks when the calling task may sleep while
             *          an interrupt-driven transaction runs. Without hooks, the
             *          transport waits with WFI. Call @ref setWaitHooks while the
             *          bus is idle. The hook table must outlive its registration.
             */
            struct WaitHooks
            {
                /** @brief Return whether the current task owns this Modbus bus.
                 *  @return True when the caller is the registered bus owner.
                 */
                bool (*isOwner)();

                /** @brief Suspend the owner for at most the requested interval.
                 *  @param timeoutMs Maximum wait in milliseconds. The callback
                 *                   must return when notified or when this timeout
                 *                   expires.
                 */
                void (*wait)(uint32_t timeoutMs);

                /** @brief Wake the bus owner from a UART interrupt callback.
                 *  @details Must be safe to call from interrupt context and must
                 *           use the scheduler's ISR-safe notification mechanism.
                 */
                void (*notifyFromIsr)();

                /** @brief Discard any stale owner notification before a new frame.
                 *  @details Runs in the transaction owner's context before TX starts.
                 */
                void (*clearNotification)();
            };

            /**
             * @brief Construct an unattached interrupt transport.
             * @details The Modbus instance attaches its UART and direction GPIO
             *          when interrupt transport is enabled.
             */
            ModbusMCUInterruptTransport() = default;

            /** @brief Copy construction is disabled because one transport owns one bus state. */
            ModbusMCUInterruptTransport(const ModbusMCUInterruptTransport&) = delete;

            /** @brief Copy assignment is disabled because one transport owns one bus state. */
            ModbusMCUInterruptTransport& operator=(const ModbusMCUInterruptTransport&) = delete;

            /**
             * @brief Register or remove scheduler hooks for the bus owner.
             * @param hooks Address of a complete, persistent hook table. Pass
             *              nullptr to use WFI while waiting for UART or HAL tick
             *              interrupts.
             * @pre Call while no Modbus transaction is active.
             * @note When non-null, every function pointer in @p hooks must be valid.
             */
            void setWaitHooks(const WaitHooks* hooks) noexcept { _waitHooks = hooks; }

            /**
             * @brief Process HAL transmit completion for the attached UART.
             * @param uart HAL handle whose transmission completed.
             * @details Called from the HAL TX-complete callback. Arms response
             *          reception for unicast requests, marks broadcast TX complete,
             *          switches the transceiver to receive mode, then wakes the owner.
             */
            void TxCpltCallback(UART_HandleTypeDef* uart);

            /**
             * @brief Commit a received byte and arm reception for the next byte.
             * @param uart HAL handle that received the byte.
             * @details Called from the HAL RX-complete callback. Once the byte
             *          count requested by the Modbus reader is reached, wakes the
             *          owner task. UART errors are left for @ref ErrorCallback.
             */
            void RxCpltCallback(UART_HandleTypeDef* uart);

            /**
             * @brief Mark the active response as failed after a UART error.
             * @param uart HAL handle that reported the error.
             * @details Called from the HAL UART error callback. Wakes the owner
             *          so the transaction can finish and release receive resources.
             */
            void ErrorCallback(UART_HandleTypeDef* uart);

        private:
            /** @brief Allows Modbus to attach the configured UART and call transport I/O. */
            friend class Modbus;

            /**
             * @brief State of the current UART transaction.
             */
            enum class Phase : uint8_t
            {
                Idle,      /**< No frame is active and a new request may start. */
                Sending,   /**< HAL is transmitting the request frame. */
                Receiving, /**< Request TX is complete and a response is being collected. */
                Sent,      /**< A broadcast frame was sent and requires no response. */
                Failed     /**< An asynchronous UART operation failed. */
            };

            UART_HandleTypeDef* _uart{nullptr}; /**< HAL UART assigned to this bus. */
            GPIO_TypeDef* _directionPort{nullptr}; /**< GPIO port controlling RS-485 DE/RE. */
            uint16_t _directionPin{0}; /**< GPIO pin mask controlling RS-485 direction. */
            GPIO_PinState _transmitLevel{GPIO_PIN_SET}; /**< GPIO level that selects transmit mode. */
            const WaitHooks* _waitHooks{nullptr}; /**< Optional scheduler hooks for the bus owner. */

            volatile Phase _phase{Phase::Idle}; /**< Shared transaction state read by caller and UART ISR. */
            uint8_t _rx[ModbusBufferCapacity<uint8_t>::value]; /**< Fixed-capacity response buffer filled by RX ISR. */
            uint8_t _rxByte{0}; /**< HAL's one-byte receive destination, rearmed after each byte. */
            volatile uint16_t _rxCount{0}; /**< Number of response bytes published in @ref _rx. */
            volatile uint16_t _rxTarget{0}; /**< Count at which the active read should be awakened. */
            uint16_t _rxRead{0}; /**< Offset of the next unread response byte. */
            bool _expectsResponse{false}; /**< False for broadcasts that have no response. */
            uint32_t _idleSince{0}; /**< Tick when the previous exchange ended, used for RTU silence. */

            /**
             * @brief Attach the UART and direction GPIO supplied to Modbus::begin().
             * @param uart Initialized HAL UART handle.
             * @param directionPort GPIO port connected to the transceiver direction input.
             * @param directionPin GPIO pin mask connected to that input.
             * @param transmitLevel GPIO level that selects transmit mode.
             * @details Called by @ref Modbus when interrupt transport is enabled.
             */
            void attach(UART_HandleTypeDef* uart, GPIO_TypeDef* directionPort,
                        uint16_t directionPin, GPIO_PinState transmitLevel) noexcept;

            /**
             * @brief Set the RS-485 transceiver to transmit or receive mode.
             * @param transmit True selects TX. False selects the inverse level for RX.
             */
            void selectTransmit(bool transmit) const;

            /** @brief Notify the owner through the registered ISR-safe hook, if present. */
            void notifyOwnerFromIsr() const;

            /**
             * @brief Wait while a transaction condition remains true.
             * @tparam Pending Predicate type that returns true while waiting should continue.
             * @param pending Condition to recheck after a wakeup.
             * @param start HAL tick at which the timeout interval began.
             * @param timeoutMs Maximum wait duration in milliseconds.
             * @details Uses scheduler hooks when registered. Otherwise sleeps with WFI
             *          and wakes for UART or HAL tick interrupts.
             */
            template<typename Pending>
            void waitWhilePending(Pending pending, uint32_t start, uint32_t timeoutMs) const;

            /**
             * @brief Send one RTU request using UART transmit interrupts.
             * @param data Complete request frame, including its CRC.
             * @param length Number of bytes in @p data.
             * @param baud Configured bus baud rate, used to calculate frame silence.
             * @param timeoutMs Maximum time allowed for request transmission.
             * @return True when TX completes and response reception is armed, or
             *         when a broadcast frame is sent successfully.
             * @return False for an invalid context, UART failure, or timeout.
             */
            bool send(const uint8_t* data, uint16_t length, uint32_t baud, uint32_t timeoutMs);

            /**
             * @brief Copy additional response bytes from the internal RX buffer.
             * @param data Destination buffer owned by the Modbus protocol layer.
             * @param length Number of bytes requested.
             * @param timeoutMs Maximum wait for the requested bytes.
             * @return True when all requested bytes were copied. False on timeout,
             *         UART failure, or insufficient internal buffer capacity.
             */
            bool read(uint8_t* data, uint16_t length, uint32_t timeoutMs);

            /**
             * @brief Abort any pending RX operation and release the receive state.
             * @details Called after response processing, including error and timeout
             *          paths. Also records the tick used to enforce the next frame's
             *          RTU silent interval.
             */
            void finishReceive();
    };
#endif
