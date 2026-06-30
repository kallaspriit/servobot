#pragma once

#include <Arduino.h>

#include "sw_uart_half_duplex.hpp"

/**
 * Adapts SwUartHalfDuplex to the Arduino Stream interface, so the Feetech
 * SCServo library (SMS_STS) can drive servos over the single-wire PIO bus.
 *
 * The SCServo protocol writes a full instruction packet with several write()
 * calls and then calls flush() (via the library's wFlushSCS) before reading the
 * reply. This adapter buffers the writes and performs the actual half-duplex
 * transmission on flush(), which sends the whole packet and arms the receiver
 * in a single turnaround. Reads come straight from the PIO receive FIFO.
 */
class HalfDuplexStream : public Stream {
   public:
    /**
     * @param bus A started SwUartHalfDuplex instance.
     */
    explicit HalfDuplexStream(SwUartHalfDuplex &bus) : bus_(bus) {}

    using Print::write;

    size_t write(uint8_t byte) override {
        if (txLength_ < sizeof(txBuffer_)) {
            txBuffer_[txLength_++] = byte;
        }

        return 1;
    }

    size_t write(const uint8_t *data, size_t length) override {
        for (size_t i = 0; i < length && txLength_ < sizeof(txBuffer_); i++) {
            txBuffer_[txLength_++] = data[i];
        }

        return length;
    }

    /** Transmits the buffered packet as one half-duplex turnaround. */
    void flush() override {
        if (txLength_ == 0) {
            return;
        }

        bus_.writeBytes(txBuffer_, txLength_);
        txLength_ = 0;
    }

    int available() override {
        return (peeked_ >= 0 ? 1 : 0) + bus_.rxAvailable();
    }

    int read() override {
        if (peeked_ >= 0) {
            const int value = peeked_;
            peeked_ = -1;

            return value;
        }

        return bus_.readByteNonBlocking();
    }

    int peek() override {
        if (peeked_ < 0) {
            peeked_ = bus_.readByteNonBlocking();
        }

        return peeked_;
    }

   private:
    SwUartHalfDuplex &bus_;
    uint8_t txBuffer_[256];
    size_t txLength_ = 0;
    int peeked_ = -1;
};
