#pragma once

#include <Arduino.h>

#include "sw_uart_half_duplex.hpp"

/**
 * Minimal driver for Feetech/Waveshare STS-series serial bus servos
 * (ST3215 / STS3215) over a single-wire half-duplex bus.
 *
 * Implements just the subset needed to bring a servo up: ping, torque
 * enable, position move, and position read. The wire protocol is the Feetech
 * SCS/STS instruction packet:
 *
 *   0xFF 0xFF  ID  LENGTH  INSTRUCTION  PARAM...  CHECKSUM
 *
 * where LENGTH = paramCount + 2 and CHECKSUM = ~(ID + LENGTH + INSTRUCTION +
 * sum(PARAM)) & 0xFF.
 */
class St3215 {
   public:
    /** Broadcast ID; servos do not reply to packets sent to this address. */
    static constexpr uint8_t kBroadcastId = 0xFE;

    /**
     * Binds the driver to an initialised half-duplex bus.
     *
     * @param bus Pointer to a started SwUartHalfDuplex instance.
     */
    void begin(SwUartHalfDuplex *bus);

    /**
     * Pings a servo to check it is present and responding.
     *
     * @param id Servo ID to ping.
     * @returns The servo ID on success, or -1 if there was no valid reply.
     */
    int ping(uint8_t id);

    /**
     * Sends a ping and returns the raw bytes received, without parsing.
     * Intended for bring-up debugging: shows exactly what (if anything) the
     * servo puts on the bus for a given attempt.
     *
     * @param id        Servo ID to ping.
     * @param buf       Destination for raw received bytes.
     * @param cap       Capacity of buf.
     * @param perByteUs Per-byte (inter-byte) read timeout in microseconds.
     * @returns Number of raw bytes received.
     */
    size_t pingRaw(uint8_t id, uint8_t *buf, size_t cap, uint32_t perByteUs);

    /**
     * Enables or disables the servo's torque (holding force).
     *
     * @param id Servo ID.
     * @param on true to hold position, false to free-wheel.
     * @returns true if the servo acknowledged.
     */
    bool setTorque(uint8_t id, bool on);

    /**
     * Changes a servo's ID, persisting it to EEPROM. Only one servo may be on
     * the bus when calling this, since every factory servo answers to ID 1.
     *
     * @param fromId Current servo ID.
     * @param toId   New servo ID (1..253).
     * @returns true if the change was acknowledged.
     */
    bool setId(uint8_t fromId, uint8_t toId);

    /**
     * Writes a single-byte register in the servo's memory table.
     *
     * @param id    Servo ID.
     * @param addr  Register address.
     * @param value Byte value to write.
     * @returns true if the servo acknowledged.
     */
    bool writeByteReg(uint8_t id, uint8_t addr, uint8_t value);

    /**
     * Moves a servo to an absolute position (servo mode).
     *
     * @param id    Servo ID.
     * @param pos   Target position, 0..4095 (2047 is centre).
     * @param speed Movement speed in steps/second (0..3073, 0 = max).
     * @param acc   Start/stop acceleration (0..150, smaller is gentler).
     * @returns true if the servo acknowledged.
     */
    bool writePos(uint8_t id, uint16_t pos, uint16_t speed, uint8_t acc);

    /**
     * Reads the servo's present position.
     *
     * @param id Servo ID.
     * @returns Position 0..4095, or -1 on failure.
     */
    int readPos(uint8_t id);

    /**
     * Reads the servo's present supply voltage.
     *
     * @param id Servo ID.
     * @returns Voltage in tenths of a volt (e.g. 115 = 11.5 V), or -1 on
     *          failure.
     */
    int readVoltage(uint8_t id);

    /**
     * Reads the servo's present temperature.
     *
     * @param id Servo ID.
     * @returns Temperature in degrees Celsius, or -1 on failure.
     */
    int readTemperature(uint8_t id);

    /**
     * Reads a single-byte register from the servo's memory table.
     *
     * @param id   Servo ID.
     * @param addr Register address.
     * @returns The byte value 0..255, or -1 on failure.
     */
    int readByteReg(uint8_t id, uint8_t addr);

    /**
     * Reads a little-endian two-byte register from the servo's memory table.
     *
     * @param id   Servo ID.
     * @param addr Address of the low byte.
     * @returns The 16-bit value, or -1 on failure.
     */
    int readWordReg(uint8_t id, uint8_t addr);

    /**
     * Returns the status/error byte from the most recent successful reply.
     * Bit flags: 0x01 voltage, 0x02 sensor, 0x04 temperature, 0x08 current,
     * 0x20 overload.
     *
     * @returns The last error byte (0 = no faults).
     */
    uint8_t lastError() const { return lastError_; }

   private:
    SwUartHalfDuplex *bus_ = nullptr;
    uint8_t lastError_ = 0;

    /**
     * Builds and transmits an instruction packet.
     *
     * @param id        Target servo ID.
     * @param instr     Instruction byte (ping/read/write).
     * @param params    Parameter bytes (may be nullptr if nparams is 0).
     * @param nparams   Number of parameter bytes.
     */
    void sendPacket(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t nparams);

    /**
     * Reads and validates a status packet from the bus.
     *
     * @param params    Destination for returned parameter bytes.
     * @param maxParams Capacity of the params buffer.
     * @param outParams Receives the number of parameter bytes returned.
     * @param timeoutUs Per-byte read timeout in microseconds.
     * @returns The servo error byte (0 = ok), or -1 on timeout/format/checksum
     *          failure.
     */
    int readStatus(uint8_t *params, uint8_t maxParams, uint8_t *outParams, uint32_t timeoutUs);
};
