#pragma once

#include <Arduino.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"

/**
 * Single-wire, half-duplex 8N1 UART implemented with the RP2040 PIO.
 *
 * One GPIO is shared for both directions. While transmitting, the pin is
 * driven push-pull by a TX state machine; when idle or receiving, the pin is
 * switched to a high-impedance input so the servo bus can drive it. The RX
 * state machine is disabled during transmission, so our own bytes are never
 * echoed back into the receive FIFO.
 *
 * This matches the electrical model of a Feetech/Waveshare STS bus, where only
 * one device drives the line at a time and the line idles high.
 */
class SwUartHalfDuplex {
   public:
    /**
     * Sets up the PIO programs, claims two state machines, and starts
     * listening on the given pin.
     *
     * @param pin  GPIO number shared for TX and RX.
     * @param baud Bus baud rate (1000000 for stock STS3215 servos).
     * @param pio  PIO instance to use (defaults to pio0).
     * @returns true on success, false if a state machine could not be claimed.
     */
    bool begin(uint pin, uint baud, PIO pio = pio0);

    /**
     * Drives the bus and transmits a block of bytes, then releases the bus and
     * resumes listening. Blocks until the last stop bit has been shifted out.
     *
     * @param data Pointer to the bytes to send.
     * @param len  Number of bytes to send.
     */
    void writeBytes(const uint8_t *data, size_t len);

    /**
     * Reads received bytes with a per-byte (inter-byte) timeout. Returns as
     * soon as len bytes arrive or the gap since the last byte exceeds the
     * timeout.
     *
     * @param buf       Destination buffer.
     * @param len       Maximum number of bytes to read.
     * @param timeoutUs Per-byte timeout in microseconds.
     * @returns Number of bytes actually read.
     */
    size_t readBytes(uint8_t *buf, size_t len, uint32_t timeoutUs);

    /** Discards any bytes currently sitting in the receive FIFO. */
    void flushRx();

    /**
     * Returns the number of bytes currently waiting in the receive FIFO.
     *
     * @returns Count of available bytes (0..8).
     */
    int rxAvailable();

    /**
     * Reads one received byte without blocking.
     *
     * @returns The byte 0..255, or -1 if the receive FIFO is empty.
     */
    int readByteNonBlocking();

   private:
    PIO pio_;
    uint smTx_;
    uint smRx_;
    uint pin_;
    uint offsetTx_;
    uint offsetRx_;
    float frameUs_;

    /** Restarts the RX state machine cleanly at its program start. */
    void restartRx();
};
