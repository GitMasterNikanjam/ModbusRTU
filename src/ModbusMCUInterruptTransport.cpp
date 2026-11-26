#include "ModbusMCUInterruptTransport.h"

#if !defined(_PLATFORM_PC_)
#include <cstring>

void ModbusMCUInterruptTransport::attach(UART_HandleTypeDef* uart, GPIO_TypeDef* directionPort,
                                         uint16_t directionPin, GPIO_PinState transmitLevel) noexcept
{
    _uart = uart;
    _directionPort = directionPort;
    _directionPin = directionPin;
    _transmitLevel = transmitLevel;
    _idleSince = HAL_GetTick();
}

void ModbusMCUInterruptTransport::selectTransmit(bool transmit) const
{
    const GPIO_PinState receiveLevel = _transmitLevel == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(_directionPort, _directionPin, transmit ? _transmitLevel : receiveLevel);
}

void ModbusMCUInterruptTransport::notifyOwnerFromIsr() const
{
    if (_waitHooks != nullptr)
        _waitHooks->notifyFromIsr();
}

template<typename Pending>
void ModbusMCUInterruptTransport::waitWhilePending(Pending pending, uint32_t start,
                                                    uint32_t timeoutMs) const
{
    if (_waitHooks != nullptr)
    {
        while (pending())
        {
            const uint32_t elapsed = HAL_GetTick() - start;
            if (elapsed >= timeoutMs)
                break;
            _waitHooks->wait(timeoutMs - elapsed);
        }
        return;
    }

    // Keep the condition check and WFI together to avoid missing a UART event.
    // A pending IRQ wakes WFI and its handler runs when PRIMASK is restored.
    for (;;)
    {
        const uint32_t irqState = __get_PRIMASK();
        __disable_irq();
        if (!pending() || HAL_GetTick() - start >= timeoutMs)
        {
            __set_PRIMASK(irqState);
            return;
        }
        __DSB();
        __WFI();
        __set_PRIMASK(irqState);
        __ISB();
    }
}

bool ModbusMCUInterruptTransport::send(const uint8_t* data, uint16_t length, uint32_t baud,
                                       uint32_t timeoutMs)
{
    // The UART IRQ and HAL tick must both be able to run while the caller waits.
    if (length == 0 || baud == 0 || _phase != Phase::Idle ||
        __get_IPSR() || __get_PRIMASK() || __get_BASEPRI())
        return false;
    if (_waitHooks != nullptr && !_waitHooks->isOwner())
        return false;

    // RTU t3.5 at up to 19200 baud uses 11 bits per character. Above that
    // rate, use the fixed interval. Allow for the resolution of HAL_GetTick().
    const uint32_t quietMs = baud > 19200u ? 3u : (38500u + baud - 1u) / baud + 1u;
    waitWhilePending([] { return true; }, _idleSince, quietMs);

    if (_waitHooks != nullptr)
        _waitHooks->clearNotification();
    _rxCount = 0;
    _rxRead = 0;
    _expectsResponse = data[0] != 0;
    __HAL_UART_CLEAR_OREFLAG(_uart);

    _phase = Phase::Sending;
    selectTransmit(true);
    const uint32_t start = HAL_GetTick();
    if (HAL_UART_Transmit_IT(_uart, const_cast<uint8_t*>(data), length) == HAL_OK)
    {
        waitWhilePending([this] { return _phase == Phase::Sending; }, start, timeoutMs);
        if (_phase == Phase::Receiving)
            return true;
        if (_phase == Phase::Sent)
        {
            _phase = Phase::Idle;
            _idleSince = HAL_GetTick();
            return true;
        }
    }

    // Stop callbacks from advancing the exchange while cancelling TX.
    const uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    _phase = Phase::Idle;
    HAL_UART_AbortTransmit(_uart);
    selectTransmit(false);
    __set_PRIMASK(irqState);
    finishReceive();
    return false;
}

bool ModbusMCUInterruptTransport::read(uint8_t* data, uint16_t length, uint32_t timeoutMs)
{
    if (length > sizeof(_rx) - _rxRead)
        return false;

    const uint16_t target = _rxRead + length;
    const uint32_t start = HAL_GetTick();
    _rxTarget = target;
    waitWhilePending([this, target] {
        return _phase == Phase::Receiving && _rxCount < target;
    }, start, timeoutMs);
    _rxTarget = 0;
    if (_phase != Phase::Receiving || _rxCount < target)
        return false;

    // After observing the published count, copy only bytes the ISR has committed.
    __DMB();
    std::memcpy(data, _rx + _rxRead, length);
    _rxRead = target;
    return true;
}

void ModbusMCUInterruptTransport::finishReceive()
{
    // Prevent the RX callback from rearming reception while it is being aborted.
    const uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    _phase = Phase::Idle;
    _rxTarget = 0;
    HAL_UART_AbortReceive(_uart);
    __set_PRIMASK(irqState);
    _idleSince = HAL_GetTick();
}

void ModbusMCUInterruptTransport::TxCpltCallback(UART_HandleTypeDef* uart)
{
    if (uart != _uart || _phase != Phase::Sending)
        return;

    // Arm reception before switching DE/RE after the final stop bit so an
    // immediate response can be captured before the owner task resumes.
    if (_expectsResponse)
    {
        _phase = HAL_UART_Receive_IT(uart, &_rxByte, 1) == HAL_OK ?
            Phase::Receiving : Phase::Failed;
    }
    else
    {
        _phase = Phase::Sent;
    }
    selectTransmit(false);
    notifyOwnerFromIsr();
}

void ModbusMCUInterruptTransport::RxCpltCallback(UART_HandleTypeDef* uart)
{
    // HAL can complete RX on an errored byte before invoking its error callback.
    // Rearming in that case would clear the error before the callback runs.
    if (uart != _uart || _phase != Phase::Receiving || uart->ErrorCode != HAL_UART_ERROR_NONE)
        return;

    const uint16_t count = _rxCount;
    _rx[count] = _rxByte;
    __DMB();
    _rxCount = count + 1u;
    if (_rxCount < sizeof(_rx) && HAL_UART_Receive_IT(uart, &_rxByte, 1) != HAL_OK)
        _phase = Phase::Failed;
    if (_phase == Phase::Failed || (_rxTarget != 0 && _rxCount >= _rxTarget))
    {
        _rxTarget = 0;
        notifyOwnerFromIsr();
    }
}

void ModbusMCUInterruptTransport::ErrorCallback(UART_HandleTypeDef* uart)
{
    if (uart == _uart && _phase == Phase::Receiving)
    {
        _phase = Phase::Failed;
        notifyOwnerFromIsr();
    }
}
#endif
