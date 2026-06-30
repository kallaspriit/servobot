#pragma once

#include <stdint.h>

#include "IServoBus.hpp"

/**
 * Driver for Feetech/Waveshare STS-series serial bus servos (ST3215 / STS3215).
 *
 * Speaks the Feetech SCS/STS instruction protocol over an IServoBus transport.
 * The wire format is:
 *
 *   0xFF 0xFF  ID  LENGTH  INSTRUCTION  PARAM...  CHECKSUM
 *
 * where LENGTH = paramCount + 2 and CHECKSUM = ~(ID + LENGTH + INSTRUCTION +
 * sum(PARAM)) & 0xFF. Multi-byte values are little-endian.
 */
class St3215 {
  public:
    /** Broadcast ID; servos do not reply to packets sent to this address. */
    static constexpr uint8_t kBroadcastId = 0xFE;

    /** Position range endpoints (12-bit magnetic encoder). */
    static constexpr int16_t kPositionMin = 0;
    static constexpr int16_t kPositionMax = 4095;
    static constexpr int16_t kPositionMid = 2047;

    /** STS/SMS memory table register addresses. */
    enum class Register : uint8_t {
        // EEPROM (read-only)
        FirmwareMajor = 0,
        FirmwareMinor = 1,
        ModelMajor = 3,
        ModelMinor = 4,

        // EEPROM (read/write)
        Id = 5,
        BaudRate = 6,
        ReturnDelay = 7,
        ResponseStatusLevel = 8,
        MinAngleLimitL = 9,
        MaxAngleLimitL = 11,
        MaxTemperatureLimit = 13,
        MaxVoltageLimit = 14,
        MinVoltageLimit = 15,
        MaxTorqueLimitL = 16,
        UnloadingCondition = 19,
        LedAlarmCondition = 20,
        PositionKp = 21,
        PositionKd = 22,
        PositionKi = 23,
        CwDeadband = 26,
        CcwDeadband = 27,
        PositionOffsetL = 31,
        OperatingMode = 33,
        ProtectiveTorque = 34,
        ProtectionTime = 35,
        OverloadTorque = 36,

        // SRAM (read/write)
        TorqueEnable = 40,
        Acceleration = 41,
        GoalPositionL = 42,
        GoalTimeL = 44,
        GoalSpeedL = 46,
        TorqueLimitL = 48,
        Lock = 55,

        // SRAM (read-only)
        PresentPositionL = 56,
        PresentSpeedL = 58,
        PresentLoadL = 60,
        PresentVoltage = 62,
        PresentTemperature = 63,
        Status = 65,
        Moving = 66,
        PresentCurrentL = 69,
    };

    /** Servo operating mode (Register::OperatingMode). */
    enum class Mode : uint8_t {
        Position = 0, // Absolute position / servo mode (0..4095)
        Speed = 1,    // Constant-speed / wheel mode (continuous)
        Pwm = 2,      // Open-loop PWM mode
        Step = 3,     // Step / multi-turn mode
    };

    /** Decoded servo status/error flags (Register::Status and reply error byte). */
    struct ServoStatus {
        uint8_t raw = 0;                 // Raw status byte
        bool hasVoltageFault = false;    // Bit 0: input voltage out of range
        bool hasSensorFault = false;     // Bit 1: angle/encoder sensor fault
        bool hasTemperatureFault = false; // Bit 2: temperature over limit
        bool hasCurrentFault = false;    // Bit 3: current over limit
        bool hasOverloadFault = false;   // Bit 5: sustained overload
        bool isValid = false;            // Was the status read successfully
    };

    /** Decoded real-time servo feedback (one bulk read from PresentPosition). */
    struct ServoFeedback {
        int position = 0;       // 0..4095
        int speed = 0;          // Signed, steps/second
        int load = 0;           // Signed, -1000..1000 (0.1% of max torque)
        int voltageDeciV = 0;   // Tenths of a volt (115 = 11.5 V)
        int temperatureC = 0;   // Degrees Celsius
        int current = 0;        // Signed, raw units (~6.5 mA each)
        bool isMoving = false;  // Servo is currently moving
        ServoStatus status;     // Decoded fault flags
        bool isValid = false;   // Was the feedback read successfully
    };

    /** Static identification and configured limits read from EEPROM. */
    struct ServoInfo {
        int firmwareMajor = 0;
        int firmwareMinor = 0;
        int modelMajor = 0;
        int modelMinor = 0;
        int id = 0;
        int baudIndex = 0;
        int minVoltageDeciV = 0;
        int maxVoltageDeciV = 0;
        int maxTemperatureC = 0;
        bool isValid = false;
    };

    /**
     * @param bus A serial bus transport (not yet started; call begin()).
     */
    explicit St3215(IServoBus& bus)
        : bus_(bus) {
    }

    /**
     * Initializes the underlying transport.
     *
     * @returns true on success.
     */
    bool begin();

    /**
     * Pings a servo to check it is present.
     *
     * @param id Servo ID.
     * @returns The servo ID on success, or -1 if there was no valid reply.
     */
    int ping(uint8_t id);

    /**
     * Changes a servo's ID, persisting it to EEPROM. Only one servo may be on
     * the bus, since every factory servo answers to ID 1.
     *
     * @param fromId Current servo ID.
     * @param toId   New servo ID (1..253).
     * @returns true if the new ID responds afterwards.
     */
    bool setId(uint8_t fromId, uint8_t toId);

    /**
     * Enables or disables the servo's torque (holding force).
     *
     * @param id Servo ID.
     * @param on true to hold position, false to free-wheel.
     * @returns true if the servo acknowledged.
     */
    bool setTorque(uint8_t id, bool on);

    /**
     * Sets the servo's operating mode, persisting it to EEPROM.
     *
     * @param id   Servo ID.
     * @param mode Desired mode.
     * @returns true if acknowledged.
     */
    bool setMode(uint8_t id, Mode mode);

    /**
     * Moves a servo to an absolute position (position mode).
     *
     * @param id       Servo ID.
     * @param position Target position 0..4095 (negative wraps for multi-turn).
     * @param speed    Steps/second (0 = unlimited).
     * @param acc      Start/stop acceleration (0..255, 0 = unlimited).
     * @returns true if acknowledged.
     */
    bool writePos(uint8_t id, int16_t position, uint16_t speed = 0, uint8_t acc = 0);

    /**
     * Queues a position for a servo without moving it yet; all queued servos
     * move together on the next action() call.
     *
     * @param id       Servo ID.
     * @param position Target position.
     * @param speed    Steps/second.
     * @param acc      Acceleration.
     * @returns true if acknowledged.
     */
    bool regWritePos(uint8_t id, int16_t position, uint16_t speed = 0, uint8_t acc = 0);

    /**
     * Triggers all positions queued with regWritePos().
     *
     * @param id Target servo ID (defaults to broadcast = all).
     */
    void action(uint8_t id = kBroadcastId);

    /**
     * Moves several servos to their target positions in a single broadcast
     * packet, so they start together.
     *
     * @param ids       Array of servo IDs.
     * @param count     Number of servos.
     * @param positions Target positions, one per servo.
     * @param speeds    Speeds, one per servo.
     * @param accs      Accelerations, one per servo.
     */
    void syncWritePos(const uint8_t* ids, uint8_t count, const int16_t* positions, const uint16_t* speeds, const uint8_t* accs);

    /**
     * Sets a continuous rotation speed (speed/wheel mode).
     *
     * @param id    Servo ID.
     * @param speed Signed steps/second.
     * @param acc   Acceleration.
     * @returns true if acknowledged.
     */
    bool writeSpeed(uint8_t id, int16_t speed, uint8_t acc = 0);

    /**
     * Calibrates the servo's current physical position as the mid-point (2047).
     *
     * @param id Servo ID.
     * @returns true if acknowledged.
     */
    bool calibrateMid(uint8_t id);

    /**
     * Reads the servo's present position.
     *
     * @param id Servo ID.
     * @returns Position 0..4095, or -1 on failure.
     */
    int readPosition(uint8_t id);

    /**
     * Reads all real-time feedback in a single transaction.
     *
     * @param id  Servo ID.
     * @param out Destination feedback struct.
     * @returns true if the read succeeded (also sets out.isValid).
     */
    bool readFeedback(uint8_t id, ServoFeedback& out);

    /**
     * Reads and decodes the servo's status/fault byte.
     *
     * @param id  Servo ID.
     * @param out Destination status struct.
     * @returns true if the read succeeded.
     */
    bool readStatus(uint8_t id, ServoStatus& out);

    /**
     * Reads feedback from several servos with a single request (instruction
     * 0x82). The servos reply in the order listed, which is far fewer bus
     * turnarounds than polling each one individually.
     *
     * @param ids   Array of servo IDs.
     * @param count Number of servos.
     * @param out   Array of feedback structs (one per servo); each entry's
     *              isValid reflects whether that servo answered.
     * @returns true if every servo answered.
     */
    bool syncReadFeedback(const uint8_t* ids, uint8_t count, ServoFeedback* out);

    /**
     * Reads static identification and configured limits from EEPROM.
     *
     * @param id  Servo ID.
     * @param out Destination info struct.
     * @returns true if the read succeeded.
     */
    bool readInfo(uint8_t id, ServoInfo& out);

    /**
     * Sets the position-mode angle limits (persisted to EEPROM). Setting both to
     * 0 disables the limits, allowing full continuous travel.
     *
     * @param id          Servo ID.
     * @param minPosition Minimum allowed position 0..4095.
     * @param maxPosition Maximum allowed position 0..4095.
     * @returns true on success.
     */
    bool setAngleLimits(uint8_t id, uint16_t minPosition, uint16_t maxPosition);

    /**
     * Sets the runtime output torque (force) limit.
     *
     * @param id    Servo ID.
     * @param limit Torque limit 0..1000 (0.1% of maximum torque).
     * @returns true if acknowledged.
     */
    bool setTorqueLimit(uint8_t id, uint16_t limit);

    /**
     * Sets the position loop PID coefficients (persisted to EEPROM).
     *
     * @param id Servo ID.
     * @param kp Proportional coefficient.
     * @param kd Derivative coefficient.
     * @param ki Integral coefficient.
     * @returns true on success.
     */
    bool setPid(uint8_t id, uint8_t kp, uint8_t kd, uint8_t ki);

    /**
     * Sets which fault flags cause the servo to release torque (persisted to
     * EEPROM). The bit mask uses the same layout as ServoStatus: 0x01 voltage,
     * 0x02 sensor, 0x04 temperature, 0x08 current, 0x20 overload.
     *
     * @param id        Servo ID.
     * @param faultMask Faults that should unload torque.
     * @returns true on success.
     */
    bool setUnloadingCondition(uint8_t id, uint8_t faultMask);

    /**
     * Reads a single-byte register.
     *
     * @param id  Servo ID.
     * @param reg Register.
     * @returns The byte value 0..255, or -1 on failure.
     */
    int readByte(uint8_t id, Register reg);

    /**
     * Reads a little-endian two-byte register.
     *
     * @param id  Servo ID.
     * @param reg Address of the low byte.
     * @returns The 16-bit value, or -1 on failure.
     */
    int readWord(uint8_t id, Register reg);

    /**
     * Writes a single-byte register.
     *
     * @param id    Servo ID.
     * @param reg   Register.
     * @param value Byte value.
     * @returns true if acknowledged.
     */
    bool writeByte(uint8_t id, Register reg, uint8_t value);

    /**
     * Writes a little-endian two-byte register.
     *
     * @param id    Servo ID.
     * @param reg   Address of the low byte.
     * @param value 16-bit value.
     * @returns true if acknowledged.
     */
    bool writeWord(uint8_t id, Register reg, uint16_t value);

    /**
     * Returns the status/error byte from the most recent successful reply.
     *
     * @returns The last error byte (0 = no faults).
     */
    uint8_t lastError() const {
        return lastError_;
    }

  private:
    IServoBus& bus_;
    uint8_t lastError_ = 0;

    /** Feetech instruction set. */
    enum class Instruction : uint8_t {
        Ping = 0x01,
        Read = 0x02,
        Write = 0x03,
        RegWrite = 0x04,
        Action = 0x05,
        SyncRead = 0x82,
        SyncWrite = 0x83,
    };

    /** Builds and transmits an instruction packet. */
    void sendPacket(uint8_t id, Instruction instr, const uint8_t* params, uint8_t nparams);

    /**
     * Reads and validates a status reply, returning the error byte or -1.
     * If outId is non-null it receives the replying servo's ID.
     */
    int readReply(uint8_t* params, uint8_t maxParams, uint8_t& outCount, uint32_t timeoutUs, uint8_t* outId = nullptr);

    /** Locks or unlocks the EEPROM write protection, with a commit delay. */
    void lockEeprom(uint8_t id, bool locked);

    /** Decodes a 15-byte feedback block into a ServoFeedback. */
    static void decodeFeedback(const uint8_t* body, ServoFeedback& out);

    /** Decodes a raw status byte into a ServoStatus. */
    static ServoStatus decodeStatus(uint8_t raw);
};
