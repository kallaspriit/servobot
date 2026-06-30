#pragma once

#include <Arduino.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "IServoBus.hpp"

/**
 * Single-wire, half-duplex 8N1 UART transport built on the RP2040 PIO.
 *
 * One GPIO is shared for both directions. While transmitting, the pin is driven
 * push-pull by a TX state machine; when idle or receiving it is switched to a
 * high-impedance input so the servo bus can drive it. The RX state machine is
 * disabled during transmission, so our own bytes are never echoed back, and the
 * bus is released the instant the last stop bit is on the wire (TX-stall flag),
 * so a fast servo reply always lands on a freed, already-listening line.
 */
class SwUartPioBus : public IServoBus {
  public:
    /**
     * @param pin  GPIO shared for TX and RX.
     * @param baud Bus baud rate (1000000 for stock STS3215 servos).
     * @param pio  PIO instance to use (defaults to pio0).
     */
    SwUartPioBus(uint pin, uint baud, PIO pio = pio0)
        : pin_(pin), baud_(baud), pio_(pio) {
    }

    bool begin() override;
    void writePacket(const uint8_t* data, size_t length) override;
    size_t readBytes(uint8_t* buffer, size_t length, uint32_t timeoutUs) override;
    void flushRx() override;

  private:
    uint pin_;
    uint baud_;
    PIO pio_;
    uint smTx_ = 0;
    uint smRx_ = 0;
    uint offsetTx_ = 0;
    uint offsetRx_ = 0;
    float frameUs_ = 0.0f;

    /** Restarts the RX state machine cleanly at its program start. */
    void restartRx();
};
