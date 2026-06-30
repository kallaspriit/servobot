#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * Transport interface for the serial bus servo driver.
 *
 * Implementations carry the byte-level half-duplex transport (e.g. a single-wire
 * PIO UART). The driver builds protocol packets and hands them to writePacket(),
 * then reads replies with readBytes(). This mirrors the adapter pattern used for
 * the CAN bus elsewhere in the codebase, keeping the driver independent of the
 * physical layer.
 */
class IServoBus {
  public:
    virtual ~IServoBus() = default;

    /**
     * Initializes the transport.
     *
     * @returns true on success.
     */
    virtual bool begin() = 0;

    /**
     * Transmits a complete packet as one half-duplex turnaround: drive the bus,
     * send every byte, release the bus, and resume listening. Blocks until the
     * last bit is on the wire.
     *
     * @param data   Bytes to send.
     * @param length Number of bytes to send.
     */
    virtual void writePacket(const uint8_t* data, size_t length) = 0;

    /**
     * Reads received bytes with a per-byte (inter-byte) timeout.
     *
     * @param buffer    Destination buffer.
     * @param length    Maximum number of bytes to read.
     * @param timeoutUs Per-byte timeout in microseconds.
     * @returns Number of bytes actually read.
     */
    virtual size_t readBytes(uint8_t* buffer, size_t length, uint32_t timeoutUs) = 0;

    /** Discards any bytes currently waiting in the receive buffer. */
    virtual void flushRx() = 0;
};
