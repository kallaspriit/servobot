#include "SerialConsole.hpp"

#include <cstdlib>
#include <cstring>

void SerialConsole::addCommand(const char* name, const char* usage, Handler handler) {
    if (commandCount_ >= kMaxCommands) {
        return;
    }

    commands_[commandCount_].name = name;
    commands_[commandCount_].usage = usage;
    commands_[commandCount_].handler = handler;
    commandCount_++;
}

void SerialConsole::loop() {
    while (io_.available()) {
        const char c = io_.read();

        if (c == '\n' || c == '\r') {
            if (length_ > 0) {
                buffer_[length_] = '\0';
                dispatch(buffer_);
                length_ = 0;
            }
        } else if (length_ < sizeof(buffer_) - 1) {
            buffer_[length_++] = c;
        }
    }
}

void SerialConsole::dispatch(char* line) {
    const char* name = strtok(line, " ");

    if (name == nullptr) {
        return;
    }

    if (strcmp(name, "help") == 0) {
        printHelp();

        return;
    }

    for (size_t i = 0; i < commandCount_; i++) {
        if (strcmp(name, commands_[i].name) == 0) {
            commands_[i].handler(*this);

            return;
        }
    }

    io_.print("unknown command '");
    io_.print(name);
    io_.println("' (try help)");
}

int SerialConsole::nextInt(int def) {
    const char* token = strtok(nullptr, " ");

    // Base 0 auto-detects 0x-prefixed hex and decimal.
    return token ? (int)strtol(token, nullptr, 0) : def;
}

const char* SerialConsole::nextToken(const char* def) {
    const char* token = strtok(nullptr, " ");

    return token ? token : def;
}

void SerialConsole::printHelp() {
    io_.println("Commands:");

    for (size_t i = 0; i < commandCount_; i++) {
        io_.print("  ");
        io_.println(commands_[i].usage);
    }

    io_.println("  help                       - this list");
}
