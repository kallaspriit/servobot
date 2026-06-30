#include "st3215.hpp"

// STS/SMS memory table addresses (same map the Feetech SMS_STS class uses).
static constexpr uint8_t kRegId = 5;
static constexpr uint8_t kRegLock = 55;
static constexpr uint8_t kRegTorqueEnable = 40;
static constexpr uint8_t kRegAcc = 41;
static constexpr uint8_t kRegGoalPositionL = 42;
static constexpr uint8_t kRegPresentPositionL = 56;
static constexpr uint8_t kRegPresentVoltage = 62;
static constexpr uint8_t kRegPresentTemperature = 63;

// Instruction set.
static constexpr uint8_t kInstPing = 0x01;
static constexpr uint8_t kInstRead = 0x02;
static constexpr uint8_t kInstWrite = 0x03;

// Default per-byte timeout while waiting for a reply.
static constexpr uint32_t kReplyTimeoutUs = 4000;

void St3215::begin(SwUartHalfDuplex *bus) {
    bus_ = bus;
}

int St3215::ping(uint8_t id) {
    sendPacket(id, kInstPing, nullptr, 0);

    uint8_t params[4];
    uint8_t nparams = 0;
    const int error = readStatus(params, sizeof(params), &nparams, kReplyTimeoutUs);

    if (error < 0) {
        return -1;
    }

    return id;
}

size_t St3215::pingRaw(uint8_t id, uint8_t *buf, size_t cap, uint32_t perByteUs) {
    sendPacket(id, kInstPing, nullptr, 0);

    return bus_->readBytes(buf, cap, perByteUs);
}

bool St3215::setTorque(uint8_t id, bool on) {
    return writeByteReg(id, kRegTorqueEnable, on ? 1 : 0);
}

bool St3215::writeByteReg(uint8_t id, uint8_t addr, uint8_t value) {
    const uint8_t params[] = {addr, value};
    sendPacket(id, kInstWrite, params, sizeof(params));

    uint8_t reply[4];
    uint8_t nparams = 0;

    return readStatus(reply, sizeof(reply), &nparams, kReplyTimeoutUs) >= 0;
}

bool St3215::setId(uint8_t fromId, uint8_t toId) {
    // EEPROM is write-protected by default; unlock, write the ID, then re-lock.
    // After the ID write the servo answers to the new ID, so the lock is
    // addressed to toId. Each EEPROM write needs a few ms to commit before the
    // servo will reliably answer the next packet.
    if (!writeByteReg(fromId, kRegLock, 0)) {
        return false;
    }

    delay(10);

    if (!writeByteReg(fromId, kRegId, toId)) {
        return false;
    }

    delay(10);

    return writeByteReg(toId, kRegLock, 1);
}

bool St3215::writePos(uint8_t id, uint16_t pos, uint16_t speed, uint8_t acc) {
    // Registers 41..47 are contiguous, so a single write sets acceleration,
    // goal position, goal time (unused, left zero), and goal speed at once.
    const uint8_t params[] = {
        kRegAcc,
        acc,
        (uint8_t)(pos & 0xFF),
        (uint8_t)((pos >> 8) & 0xFF),
        0,
        0,
        (uint8_t)(speed & 0xFF),
        (uint8_t)((speed >> 8) & 0xFF),
    };
    sendPacket(id, kInstWrite, params, sizeof(params));

    uint8_t reply[4];
    uint8_t nparams = 0;

    return readStatus(reply, sizeof(reply), &nparams, kReplyTimeoutUs) >= 0;
}

int St3215::readPos(uint8_t id) {
    const int value = readWordReg(id, kRegPresentPositionL);

    if (value < 0) {
        return -1;
    }

    return value & 0x7FFF;
}

int St3215::readVoltage(uint8_t id) {
    return readByteReg(id, kRegPresentVoltage);
}

int St3215::readTemperature(uint8_t id) {
    return readByteReg(id, kRegPresentTemperature);
}

int St3215::readByteReg(uint8_t id, uint8_t addr) {
    const uint8_t params[] = {addr, 1};
    sendPacket(id, kInstRead, params, sizeof(params));

    uint8_t reply[4];
    uint8_t nparams = 0;
    const int error = readStatus(reply, sizeof(reply), &nparams, kReplyTimeoutUs);

    if (error < 0 || nparams < 1) {
        return -1;
    }

    return reply[0];
}

int St3215::readWordReg(uint8_t id, uint8_t addr) {
    const uint8_t params[] = {addr, 2};
    sendPacket(id, kInstRead, params, sizeof(params));

    uint8_t reply[4];
    uint8_t nparams = 0;
    const int error = readStatus(reply, sizeof(reply), &nparams, kReplyTimeoutUs);

    if (error < 0 || nparams < 2) {
        return -1;
    }

    return reply[0] | (reply[1] << 8);
}

void St3215::sendPacket(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t nparams) {
    uint8_t packet[16];
    const uint8_t length = nparams + 2;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = length;
    packet[4] = instr;

    uint16_t checksum = id + length + instr;

    for (uint8_t i = 0; i < nparams; i++) {
        packet[5 + i] = params[i];
        checksum += params[i];
    }

    packet[5 + nparams] = (uint8_t)(~checksum);

    bus_->writeBytes(packet, 6 + nparams);
}

int St3215::readStatus(uint8_t *params, uint8_t maxParams, uint8_t *outParams, uint32_t timeoutUs) {
    *outParams = 0;

    // Find the 0xFF 0xFF header.
    uint8_t prev = 0x00;
    bool synced = false;

    for (int i = 0; i < 32 && !synced; i++) {
        uint8_t byte = 0;

        if (bus_->readBytes(&byte, 1, timeoutUs) != 1) {
            return -1;
        }

        if (prev == 0xFF && byte == 0xFF) {
            synced = true;
        }

        prev = byte;
    }

    if (!synced) {
        return -1;
    }

    // Read ID and LENGTH.
    uint8_t header[2];

    if (bus_->readBytes(header, 2, timeoutUs) != 2) {
        return -1;
    }

    const uint8_t id = header[0];
    const uint8_t length = header[1];

    if (length < 2) {
        return -1;
    }

    // LENGTH bytes follow: error + params + checksum.
    uint8_t body[16];

    if (length > sizeof(body)) {
        return -1;
    }

    if (bus_->readBytes(body, length, timeoutUs) != length) {
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
        return -1;
    }

    const uint8_t copyCount = (nparams < maxParams) ? nparams : maxParams;

    for (uint8_t i = 0; i < copyCount; i++) {
        params[i] = body[1 + i];
    }

    *outParams = copyCount;
    lastError_ = error;

    return error;
}
