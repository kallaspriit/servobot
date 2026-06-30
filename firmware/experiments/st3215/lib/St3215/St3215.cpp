#include "St3215.hpp"

#include <Arduino.h>

#include "St3215Log.hpp"

// Per-byte timeout while waiting for a reply.
static constexpr uint32_t kReplyTimeoutUs = 4000;

// Calibration trigger value written to the torque-enable register.
static constexpr uint8_t kCalibrationTrigger = 128;

// Decodes a Feetech sign-magnitude 16-bit value (bit 15 is the sign).
static int decodeSigned(uint16_t value) {
    if (value & 0x8000) {
        return -(int)(value & 0x7FFF);
    }

    return (int)value;
}

bool St3215::begin() {
    return bus_.begin();
}

int St3215::ping(uint8_t id) {
    sendPacket(id, Instruction::Ping, nullptr, 0);

    uint8_t reply[4];
    uint8_t count = 0;

    if (readReply(reply, sizeof(reply), count, kReplyTimeoutUs) < 0) {
        return -1;
    }

    return id;
}

bool St3215::setId(uint8_t fromId, uint8_t toId) {
    // EEPROM is write-protected by default; unlock, write the ID, then re-lock.
    // After the ID write the servo answers to the new ID, so the lock targets
    // toId. Each EEPROM write needs a few ms to commit.
    writeByte(fromId, Register::Lock, 0);
    delay(10);
    writeByte(fromId, Register::Id, toId);
    delay(10);
    writeByte(toId, Register::Lock, 1);
    delay(20);

    return ping(toId) != -1;
}

bool St3215::setTorque(uint8_t id, bool on) {
    return writeByte(id, Register::TorqueEnable, on ? 1 : 0);
}

bool St3215::setMode(uint8_t id, Mode mode) {
    return writeByte(id, Register::OperatingMode, (uint8_t)mode);
}

bool St3215::writePos(uint8_t id, int16_t position, uint16_t speed, uint8_t acc) {
    uint16_t encodedPosition = (uint16_t)position;

    if (position < 0) {
        encodedPosition = (uint16_t)(-position) | 0x8000;
    }

    // Registers 41..47 are contiguous: acceleration, goal position, goal time
    // (unused), goal speed.
    const uint8_t params[] = {
        (uint8_t)Register::Acceleration,
        acc,
        (uint8_t)(encodedPosition & 0xFF),
        (uint8_t)((encodedPosition >> 8) & 0xFF),
        0,
        0,
        (uint8_t)(speed & 0xFF),
        (uint8_t)((speed >> 8) & 0xFF),
    };
    sendPacket(id, Instruction::Write, params, sizeof(params));

    uint8_t reply[4];
    uint8_t count = 0;

    return readReply(reply, sizeof(reply), count, kReplyTimeoutUs) >= 0;
}

bool St3215::regWritePos(uint8_t id, int16_t position, uint16_t speed, uint8_t acc) {
    uint16_t encodedPosition = (uint16_t)position;

    if (position < 0) {
        encodedPosition = (uint16_t)(-position) | 0x8000;
    }

    const uint8_t params[] = {
        (uint8_t)Register::Acceleration,
        acc,
        (uint8_t)(encodedPosition & 0xFF),
        (uint8_t)((encodedPosition >> 8) & 0xFF),
        0,
        0,
        (uint8_t)(speed & 0xFF),
        (uint8_t)((speed >> 8) & 0xFF),
    };
    sendPacket(id, Instruction::RegWrite, params, sizeof(params));

    uint8_t reply[4];
    uint8_t count = 0;

    return readReply(reply, sizeof(reply), count, kReplyTimeoutUs) >= 0;
}

void St3215::action(uint8_t id) {
    sendPacket(id, Instruction::Action, nullptr, 0);

    if (id != kBroadcastId) {
        uint8_t reply[4];
        uint8_t count = 0;
        readReply(reply, sizeof(reply), count, kReplyTimeoutUs);
    }
}

void St3215::syncWritePos(const uint8_t* ids, uint8_t count, const int16_t* positions, const uint16_t* speeds, const uint8_t* accs) {
    static constexpr uint8_t kDataLen = 7; // acc, posL, posH, timeL, timeH, spdL, spdH

    uint8_t packet[256];
    const uint8_t length = (kDataLen + 1) * count + 4;
    uint16_t checksum = kBroadcastId + length + (uint8_t)Instruction::SyncWrite + (uint8_t)Register::Acceleration + kDataLen;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = kBroadcastId;
    packet[3] = length;
    packet[4] = (uint8_t)Instruction::SyncWrite;
    packet[5] = (uint8_t)Register::Acceleration;
    packet[6] = kDataLen;

    size_t index = 7;

    for (uint8_t i = 0; i < count; i++) {
        uint16_t encodedPosition = (uint16_t)positions[i];

        if (positions[i] < 0) {
            encodedPosition = (uint16_t)(-positions[i]) | 0x8000;
        }

        const uint8_t block[kDataLen] = {
            accs[i],
            (uint8_t)(encodedPosition & 0xFF),
            (uint8_t)((encodedPosition >> 8) & 0xFF),
            0,
            0,
            (uint8_t)(speeds[i] & 0xFF),
            (uint8_t)((speeds[i] >> 8) & 0xFF),
        };

        packet[index++] = ids[i];
        checksum += ids[i];

        for (uint8_t j = 0; j < kDataLen; j++) {
            packet[index++] = block[j];
            checksum += block[j];
        }
    }

    packet[index++] = (uint8_t)(~checksum);

    bus_.writePacket(packet, index);
}

bool St3215::writeSpeed(uint8_t id, int16_t speed, uint8_t acc) {
    uint16_t encodedSpeed = (uint16_t)speed;

    if (speed < 0) {
        encodedSpeed = (uint16_t)(-speed) | 0x8000;
    }

    if (!writeByte(id, Register::Acceleration, acc)) {
        return false;
    }

    return writeWord(id, Register::GoalSpeedL, encodedSpeed);
}

bool St3215::calibrateMid(uint8_t id) {
    return writeByte(id, Register::TorqueEnable, kCalibrationTrigger);
}

int St3215::readPosition(uint8_t id) {
    const int value = readWord(id, Register::PresentPositionL);

    if (value < 0) {
        return -1;
    }

    return value & 0x7FFF;
}

bool St3215::readFeedback(uint8_t id, ServoFeedback& out) {
    const uint8_t params[] = {(uint8_t)Register::PresentPositionL, 15};
    sendPacket(id, Instruction::Read, params, sizeof(params));

    uint8_t body[16];
    uint8_t count = 0;

    if (readReply(body, sizeof(body), count, kReplyTimeoutUs) < 0 || count < 15) {
        out.isValid = false;

        return false;
    }

    // Position is unsigned 0..4095 in position mode, but sign-magnitude and
    // multi-turn in step mode, so decode the sign.
    out.position = decodeSigned(body[0] | (body[1] << 8));
    out.speed = decodeSigned(body[2] | (body[3] << 8));
    out.load = decodeSigned(body[4] | (body[5] << 8));
    out.voltageDeciV = body[6];
    out.temperatureC = body[7];
    out.status = decodeStatus(body[9]);
    out.isMoving = body[10] != 0;
    out.current = decodeSigned(body[13] | (body[14] << 8));
    out.isValid = true;

    if (out.status.raw != 0) {
        ST_LOG_W("servo %u fault flags 0x%02X", id, out.status.raw);
    }

    return true;
}

bool St3215::readStatus(uint8_t id, ServoStatus& out) {
    const int value = readByte(id, Register::Status);

    if (value < 0) {
        out.isValid = false;

        return false;
    }

    out = decodeStatus((uint8_t)value);

    return true;
}

bool St3215::readInfo(uint8_t id, ServoInfo& out) {
    const int firmwareMajor = readByte(id, Register::FirmwareMajor);

    if (firmwareMajor < 0) {
        out.isValid = false;

        return false;
    }

    out.firmwareMajor = firmwareMajor;
    out.firmwareMinor = readByte(id, Register::FirmwareMinor);
    out.modelMajor = readByte(id, Register::ModelMajor);
    out.modelMinor = readByte(id, Register::ModelMinor);
    out.id = readByte(id, Register::Id);
    out.baudIndex = readByte(id, Register::BaudRate);
    out.minVoltageDeciV = readByte(id, Register::MinVoltageLimit);
    out.maxVoltageDeciV = readByte(id, Register::MaxVoltageLimit);
    out.maxTemperatureC = readByte(id, Register::MaxTemperatureLimit);
    out.isValid = true;

    return true;
}

int St3215::readByte(uint8_t id, Register reg) {
    const uint8_t params[] = {(uint8_t)reg, 1};
    sendPacket(id, Instruction::Read, params, sizeof(params));

    uint8_t reply[8];
    uint8_t count = 0;

    if (readReply(reply, sizeof(reply), count, kReplyTimeoutUs) < 0 || count < 1) {
        return -1;
    }

    return reply[0];
}

int St3215::readWord(uint8_t id, Register reg) {
    const uint8_t params[] = {(uint8_t)reg, 2};
    sendPacket(id, Instruction::Read, params, sizeof(params));

    uint8_t reply[8];
    uint8_t count = 0;

    if (readReply(reply, sizeof(reply), count, kReplyTimeoutUs) < 0 || count < 2) {
        return -1;
    }

    return reply[0] | (reply[1] << 8);
}

bool St3215::writeByte(uint8_t id, Register reg, uint8_t value) {
    const uint8_t params[] = {(uint8_t)reg, value};
    sendPacket(id, Instruction::Write, params, sizeof(params));

    uint8_t reply[4];
    uint8_t count = 0;

    return readReply(reply, sizeof(reply), count, kReplyTimeoutUs) >= 0;
}

bool St3215::writeWord(uint8_t id, Register reg, uint16_t value) {
    const uint8_t params[] = {(uint8_t)reg, (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF)};
    sendPacket(id, Instruction::Write, params, sizeof(params));

    uint8_t reply[4];
    uint8_t count = 0;

    return readReply(reply, sizeof(reply), count, kReplyTimeoutUs) >= 0;
}

void St3215::sendPacket(uint8_t id, Instruction instr, const uint8_t* params, uint8_t nparams) {
    uint8_t packet[32];
    const uint8_t length = nparams + 2;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = length;
    packet[4] = (uint8_t)instr;

    uint16_t checksum = id + length + (uint8_t)instr;

    for (uint8_t i = 0; i < nparams; i++) {
        packet[5 + i] = params[i];
        checksum += params[i];
    }

    packet[5 + nparams] = (uint8_t)(~checksum);

    bus_.writePacket(packet, 6 + nparams);
}

int St3215::readReply(uint8_t* params, uint8_t maxParams, uint8_t& outCount, uint32_t timeoutUs) {
    outCount = 0;

    // Find the 0xFF 0xFF header.
    uint8_t prev = 0x00;
    bool synced = false;

    for (int i = 0; i < 32 && !synced; i++) {
        uint8_t byte = 0;

        if (bus_.readBytes(&byte, 1, timeoutUs) != 1) {
            // A clean timeout with no bytes is normal (absent servo / no reply),
            // so it is verbose, not a warning.
            ST_LOG_V("reply timeout (no header)");

            return -1;
        }

        if (prev == 0xFF && byte == 0xFF) {
            synced = true;
        }

        prev = byte;
    }

    if (!synced) {
        ST_LOG_V("reply header not found");

        return -1;
    }

    // Read ID and LENGTH.
    uint8_t header[2];

    if (bus_.readBytes(header, 2, timeoutUs) != 2) {
        return -1;
    }

    const uint8_t id = header[0];
    const uint8_t length = header[1];

    if (length < 2) {
        return -1;
    }

    // LENGTH bytes follow: error + params + checksum. Sized for the largest
    // reply we issue (a 15-byte bulk feedback read => length 17).
    uint8_t body[32];

    if (length > sizeof(body)) {
        return -1;
    }

    if (bus_.readBytes(body, length, timeoutUs) != length) {
        return -1;
    }

    const uint8_t error = body[0];
    const uint8_t nparams = length - 2;
    const uint8_t checksum = body[length - 1];

    // Verify checksum over ID + LENGTH + error + params.
    uint16_t sum = id + length + error;

    for (uint8_t i = 0; i < nparams; i++) {
        sum += body[1 + i];
    }

    if ((uint8_t)(~sum) != checksum) {
        ST_LOG_W("reply checksum mismatch");

        return -1;
    }

    const uint8_t copyCount = (nparams < maxParams) ? nparams : maxParams;

    for (uint8_t i = 0; i < copyCount; i++) {
        params[i] = body[1 + i];
    }

    outCount = copyCount;
    lastError_ = error;

    return error;
}

St3215::ServoStatus St3215::decodeStatus(uint8_t raw) {
    ServoStatus status;
    status.raw = raw;
    status.hasVoltageFault = raw & 0x01;
    status.hasSensorFault = raw & 0x02;
    status.hasTemperatureFault = raw & 0x04;
    status.hasCurrentFault = raw & 0x08;
    status.hasOverloadFault = raw & 0x20;
    status.isValid = true;

    return status;
}
