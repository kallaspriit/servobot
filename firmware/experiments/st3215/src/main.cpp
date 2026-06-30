#include <Arduino.h>

#include <cstdlib>
#include <cstring>

#include "St3215.hpp"
#include "SwUartPioBus.hpp"
#include "stream_operators.hpp"

// Single GPIO shared with the servo bus, through a ~1k series resistor.
static constexpr uint kServoPin = 23;

// STS3215 servos ship at 1 Mbps.
static constexpr uint kServoBaud = 1000000;

// Highest ID probed by the "scan" command.
static constexpr uint8_t kMaxScanId = 20;

// Most servos a single "sync" command can address.
static constexpr size_t kMaxSyncServos = 8;

static SwUartPioBus bus(kServoPin, kServoBaud);
static St3215 servo(bus);

// Line buffer for the serial console.
static char lineBuffer[64];
static size_t lineLength = 0;

/** Prints a deci-volt value (tenths of a volt) as "X.Y V". */
static void printVolts(int deciVolts) {
    Serial << (deciVolts / 10) << "." << (deciVolts % 10) << " V";
}

/** Prints the servo's decoded status/fault flags. */
static void printStatusFlags(const St3215::ServoStatus &status) {
    Serial << "Status flags: ";

    if (status.raw == 0) {
        Serial << "none";
    } else {
        if (status.hasVoltageFault) {
            Serial << "voltage ";
        }

        if (status.hasSensorFault) {
            Serial << "sensor ";
        }

        if (status.hasTemperatureFault) {
            Serial << "temperature ";
        }

        if (status.hasCurrentFault) {
            Serial << "current ";
        }

        if (status.hasOverloadFault) {
            Serial << "overload ";
        }
    }

    Serial << endl;
}

/** Reads and prints a servo's identification, limits, and live feedback. */
static void printServoInfo(uint8_t id) {
    St3215::ServoInfo info;

    if (!servo.readInfo(id, info)) {
        Serial << "servo " << id << " no response" << endl;

        return;
    }

    Serial << "Firmware version: " << info.firmwareMajor << "." << info.firmwareMinor << endl;
    Serial << "Servo series version: " << info.modelMajor << "." << info.modelMinor << endl;
    Serial << "Stored ID: " << info.id << ", baud index: " << info.baudIndex << endl;
    Serial << "Voltage limits: ";
    printVolts(info.minVoltageDeciV);
    Serial << " - ";
    printVolts(info.maxVoltageDeciV);
    Serial << endl;
    Serial << "Max temperature: " << info.maxTemperatureC << " C" << endl;

    St3215::ServoFeedback feedback;

    if (servo.readFeedback(id, feedback)) {
        Serial << "Voltage: ";
        printVolts(feedback.voltageDeciV);
        Serial << endl;
        Serial << "Temperature: " << feedback.temperatureC << " C" << endl;
        Serial << "Position: " << feedback.position << endl;
        printStatusFlags(feedback.status);
    }
}

/** Reads and prints a servo's live feedback. */
static void printFeedback(uint8_t id) {
    St3215::ServoFeedback feedback;

    if (!servo.readFeedback(id, feedback)) {
        Serial << "servo " << id << " no response" << endl;

        return;
    }

    Serial << "pos=" << feedback.position << " speed=" << feedback.speed << " load=" << feedback.load << " current=" << feedback.current << " moving=" << feedback.isMoving << endl;
    Serial << "Voltage: ";
    printVolts(feedback.voltageDeciV);
    Serial << " temp=" << feedback.temperatureC << " C" << endl;
    printStatusFlags(feedback.status);
}

/** Pings every ID up to kMaxScanId and lists the ones that respond. */
static void scanBus() {
    Serial << "Scanning IDs 1.." << kMaxScanId << " ..." << endl;

    int found = 0;

    for (uint8_t id = 1; id <= kMaxScanId; id++) {
        if (servo.ping(id) != -1) {
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
    Serial << "  info <id>                  - version, limits, live feedback" << endl;
    Serial << "  feedback <id>              - live position/speed/load/etc" << endl;
    Serial << "  id <from> <to>             - change ID (ONLY ONE servo on bus!)" << endl;
    Serial << "  move <id> <pos> [spd] [acc] - move to position 0..4095" << endl;
    Serial << "  sync <pos> <id> [id...]    - move several servos together" << endl;
    Serial << "  speed <id> <val> [acc]     - continuous speed (needs mode 1)" << endl;
    Serial << "  mode <id> <0|1|2|3>        - position/speed/pwm/step" << endl;
    Serial << "  calibrate <id>             - set current position as mid (2047)" << endl;
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

        if (servo.ping(id) != -1) {
            Serial << "servo " << id << " OK" << endl;
        } else {
            Serial << "servo " << id << " no response" << endl;
        }

        return;
    }

    if (strcmp(command, "info") == 0) {
        printServoInfo((uint8_t)nextInt(-1));

        return;
    }

    if (strcmp(command, "feedback") == 0) {
        printFeedback((uint8_t)nextInt(-1));

        return;
    }

    if (strcmp(command, "id") == 0) {
        const int from = nextInt(-1);
        const int to = nextInt(-1);

        if (from < 1 || from > 253 || to < 1 || to > 253) {
            Serial << "usage: id <from 1..253> <to 1..253>" << endl;

            return;
        }

        if (servo.setId((uint8_t)from, (uint8_t)to)) {
            Serial << "ID changed " << from << " -> " << to << endl;
        } else {
            Serial << "ID change failed (is exactly one servo connected?)" << endl;
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

        servo.writePos((uint8_t)id, (int16_t)pos, (uint16_t)speed, (uint8_t)acc);
        Serial << "move " << id << " -> " << pos << endl;

        return;
    }

    if (strcmp(command, "sync") == 0) {
        const int pos = nextInt(-1);

        if (pos < 0 || pos > 4095) {
            Serial << "usage: sync <pos 0..4095> <id> [id...]" << endl;

            return;
        }

        uint8_t ids[kMaxSyncServos];
        int16_t positions[kMaxSyncServos];
        uint16_t speeds[kMaxSyncServos];
        uint8_t accs[kMaxSyncServos];
        uint8_t count = 0;

        for (int id = nextInt(-1); id > 0 && count < kMaxSyncServos; id = nextInt(-1)) {
            ids[count] = (uint8_t)id;
            positions[count] = (int16_t)pos;
            speeds[count] = 0;
            accs[count] = 0;
            count++;
        }

        if (count == 0) {
            Serial << "usage: sync <pos 0..4095> <id> [id...]" << endl;

            return;
        }

        servo.syncWritePos(ids, count, positions, speeds, accs);
        Serial << "sync " << count << " servo(s) -> " << pos << endl;

        return;
    }

    if (strcmp(command, "speed") == 0) {
        const int id = nextInt(-1);
        const int value = nextInt(0);
        const int acc = nextInt(0);

        if (id < 1) {
            Serial << "usage: speed <id> <value> [acc]  (set mode 1 first)" << endl;

            return;
        }

        servo.writeSpeed((uint8_t)id, (int16_t)value, (uint8_t)acc);
        Serial << "speed " << id << " = " << value << endl;

        return;
    }

    if (strcmp(command, "mode") == 0) {
        const int id = nextInt(-1);
        const int mode = nextInt(-1);

        if (id < 1 || mode < 0 || mode > 3) {
            Serial << "usage: mode <id> <0=pos 1=speed 2=pwm 3=step>" << endl;

            return;
        }

        if (servo.setMode((uint8_t)id, (St3215::Mode)mode)) {
            Serial << "mode " << id << " = " << mode << endl;
        } else {
            Serial << "mode change failed" << endl;
        }

        return;
    }

    if (strcmp(command, "calibrate") == 0) {
        const int id = nextInt(-1);

        if (servo.calibrateMid((uint8_t)id)) {
            Serial << "calibrated servo " << id << " mid-point" << endl;
        } else {
            Serial << "calibrate failed" << endl;
        }

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

    Serial << "ST3215 console (custom driver over single-wire PIO)" << endl;

    if (!servo.begin()) {
        Serial << "Failed to start PIO UART (no free state machine)" << endl;

        return;
    }

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
