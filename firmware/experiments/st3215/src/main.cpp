#include <Arduino.h>

#include <cstdlib>
#include <cstring>

#include "st3215.hpp"
#include "stream_operators.hpp"
#include "sw_uart_half_duplex.hpp"

// Single GPIO shared with the servo bus, through a ~1k series resistor.
static constexpr uint kServoPin = 23;

// STS3215 servos ship at 1 Mbps.
static constexpr uint kServoBaud = 1000000;

// Highest ID probed by the "scan" command.
static constexpr uint8_t kMaxScanId = 20;

// Identification registers in the servo's EEPROM.
static constexpr uint8_t kRegFirmwareMajor = 0;
static constexpr uint8_t kRegFirmwareMinor = 1;
static constexpr uint8_t kRegServoVersionMajor = 3;
static constexpr uint8_t kRegServoVersionMinor = 4;
static constexpr uint8_t kRegId = 5;
static constexpr uint8_t kRegBaud = 6;

// Voltage limit registers in the servo's EEPROM (tenths of a volt).
static constexpr uint8_t kRegMaxVoltageLimit = 14;
static constexpr uint8_t kRegMinVoltageLimit = 15;

static SwUartHalfDuplex bus;
static St3215 servo;

// Line buffer for the serial console.
static char lineBuffer[64];
static size_t lineLength = 0;

/** Reads and prints the servo's identification registers. */
static void printServoInfo(uint8_t id) {
    const int firmwareMajor = servo.readByteReg(id, kRegFirmwareMajor);
    const int firmwareMinor = servo.readByteReg(id, kRegFirmwareMinor);
    const int versionMajor = servo.readByteReg(id, kRegServoVersionMajor);
    const int versionMinor = servo.readByteReg(id, kRegServoVersionMinor);
    const int storedId = servo.readByteReg(id, kRegId);
    const int baudIndex = servo.readByteReg(id, kRegBaud);

    Serial << "Firmware version: " << firmwareMajor << "." << firmwareMinor << endl;
    Serial << "Servo series version: " << versionMajor << "." << versionMinor << endl;
    Serial << "Stored ID: " << storedId << ", baud index: " << baudIndex << endl;
}

/** Reads and prints the servo's voltage, temperature, limits, and error flags. */
static void printServoStatus(uint8_t id) {
    const int voltage = servo.readVoltage(id);
    const int temperature = servo.readTemperature(id);
    const int maxVoltage = servo.readByteReg(id, kRegMaxVoltageLimit);
    const int minVoltage = servo.readByteReg(id, kRegMinVoltageLimit);
    const uint8_t error = servo.lastError();

    if (voltage >= 0) {
        Serial << "Voltage: " << (voltage / 10) << "." << (voltage % 10) << " V" << endl;
    }

    if (minVoltage >= 0 && maxVoltage >= 0) {
        Serial << "Voltage limits: " << (minVoltage / 10) << "." << (minVoltage % 10) << " - "
               << (maxVoltage / 10) << "." << (maxVoltage % 10) << " V" << endl;
    }

    if (temperature >= 0) {
        Serial << "Temperature: " << temperature << " C" << endl;
    }

    Serial << "Status flags: ";

    if (error == 0) {
        Serial << "none";
    } else {
        if (error & 0x01) {
            Serial << "voltage ";
        }

        if (error & 0x02) {
            Serial << "sensor ";
        }

        if (error & 0x04) {
            Serial << "temperature ";
        }

        if (error & 0x08) {
            Serial << "current ";
        }

        if (error & 0x20) {
            Serial << "overload ";
        }
    }

    Serial << endl;
}

/** Pings every ID up to kMaxScanId and lists the ones that respond. */
static void scanBus() {
    Serial << "Scanning IDs 1.." << kMaxScanId << " ..." << endl;

    int found = 0;

    for (uint8_t id = 1; id <= kMaxScanId; id++) {
        if (servo.ping(id) >= 0) {
            Serial << "  found servo " << id << endl;
            found++;
        }
    }

    Serial << found << " servo(s) found" << endl;
}

/** Prints the available console commands. */
static void printHelp() {
    Serial << "Commands:" << endl;
    Serial << "  scan                       - find servos (IDs 1.." << kMaxScanId << ")" << endl;
    Serial << "  ping <id>                  - ping a servo" << endl;
    Serial << "  info <id>                  - version, voltage, temp, flags" << endl;
    Serial << "  id <from> <to>             - change ID (ONLY ONE servo on bus!)" << endl;
    Serial << "  move <id> <pos> [spd] [acc] - move to position 0..4095" << endl;
    Serial << "  torque <id> <0|1>          - torque off / on" << endl;
    Serial << "  help                       - this list" << endl;
}

/** Returns the next whitespace-separated token as an int, or def if absent. */
static int nextInt(int def) {
    const char *token = strtok(nullptr, " ");

    return token ? atoi(token) : def;
}

/** Parses and executes one console command line (modified in place by strtok). */
static void handleCommand(char *line) {
    const char *command = strtok(line, " ");

    if (command == nullptr) {
        return;
    }

    if (strcmp(command, "help") == 0) {
        printHelp();

        return;
    }

    if (strcmp(command, "scan") == 0) {
        scanBus();

        return;
    }

    if (strcmp(command, "ping") == 0) {
        const int id = nextInt(-1);

        if (servo.ping(id) >= 0) {
            Serial << "servo " << id << " OK" << endl;
        } else {
            Serial << "servo " << id << " no response" << endl;
        }

        return;
    }

    if (strcmp(command, "info") == 0) {
        const int id = nextInt(-1);

        if (servo.ping(id) < 0) {
            Serial << "servo " << id << " no response" << endl;

            return;
        }

        printServoInfo(id);
        printServoStatus(id);

        return;
    }

    if (strcmp(command, "id") == 0) {
        const int from = nextInt(-1);
        const int to = nextInt(-1);

        if (from < 1 || from > 253 || to < 1 || to > 253) {
            Serial << "usage: id <from 1..253> <to 1..253>" << endl;

            return;
        }

        if (!servo.setId((uint8_t)from, (uint8_t)to)) {
            Serial << "ID change not acknowledged (is exactly one servo connected?)" << endl;

            return;
        }

        // Verify, allowing for the EEPROM commit settling time.
        int verifiedId = -1;

        for (int attempt = 0; attempt < 5 && verifiedId < 0; attempt++) {
            delay(20);
            verifiedId = servo.ping((uint8_t)to);
        }

        if (verifiedId >= 0) {
            Serial << "ID changed " << from << " -> " << to << endl;
        } else {
            Serial << "ID write sent but no reply at " << to << " - run scan to confirm" << endl;
        }

        return;
    }

    if (strcmp(command, "move") == 0) {
        const int id = nextInt(-1);
        const int pos = nextInt(-1);
        const int speed = nextInt(0);
        const int acc = nextInt(0);

        if (id < 1 || pos < 0 || pos > 4095) {
            Serial << "usage: move <id> <pos 0..4095> [speed] [acc]" << endl;

            return;
        }

        servo.writePos((uint8_t)id, (uint16_t)pos, (uint16_t)speed, (uint8_t)acc);
        Serial << "move " << id << " -> " << pos << endl;

        return;
    }

    if (strcmp(command, "torque") == 0) {
        const int id = nextInt(-1);
        const int on = nextInt(-1);

        servo.setTorque((uint8_t)id, on != 0);
        Serial << "torque " << id << " = " << (on != 0) << endl;

        return;
    }

    Serial << "unknown command '" << command << "' (try help)" << endl;
}

void setup() {
    Serial.begin(115200);

    // Wait for the monitor, but don't block forever if running headless.
    while (!Serial && millis() < 8000) {
        delay(50);
    }

    delay(100);

    Serial << "ST3215 single-wire PIO console" << endl;

    if (!bus.begin(kServoPin, kServoBaud)) {
        Serial << "Failed to start PIO UART (no free state machine)" << endl;

        return;
    }

    servo.begin(&bus);

    scanBus();
    printHelp();
}

void loop() {
    while (Serial.available()) {
        const char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (lineLength > 0) {
                lineBuffer[lineLength] = '\0';
                handleCommand(lineBuffer);
                lineLength = 0;
            }
        } else if (lineLength < sizeof(lineBuffer) - 1) {
            lineBuffer[lineLength++] = c;
        }
    }
}
