#pragma once

#include <Arduino.h>
#include <functional>
#include <stddef.h>

/**
 * Minimal non-blocking serial command console.
 *
 * Commands are registered with a name, a one-line usage string (shown by the
 * built-in "help"), and a handler. Handlers pull their arguments from the
 * console with nextInt()/nextToken(). The console accumulates a line of input
 * and dispatches it on newline, so loop() never blocks.
 */
class SerialConsole {
  public:
    /** Command handler; receives the console for argument access and output. */
    using Handler = std::function<void(SerialConsole&)>;

    /**
     * @param io Stream to read commands from and write responses to.
     */
    explicit SerialConsole(Stream& io = Serial)
        : io_(io) {
    }

    /**
     * Registers a command.
     *
     * @param name    Command word to match (first token).
     * @param usage   One-line help text shown by "help".
     * @param handler Called when the command is entered.
     */
    void addCommand(const char* name, const char* usage, Handler handler);

    /** Reads available input and dispatches a command on each completed line. */
    void loop();

    /**
     * Returns the next argument token parsed as an integer. Base 0, so it
     * accepts decimal and 0x-prefixed hex.
     *
     * @param def Value to return when no token remains.
     * @returns The parsed integer, or def.
     */
    int nextInt(int def = 0);

    /**
     * Returns the next raw argument token.
     *
     * @param def Value to return when no token remains.
     * @returns The token, or def.
     */
    const char* nextToken(const char* def = nullptr);

    /** Returns the output stream for handler responses. */
    Print& out() {
        return io_;
    }

    /** Prints the list of registered commands. */
    void printHelp();

  private:
    static constexpr size_t kMaxCommands = 32;
    static constexpr size_t kBufferSize = 96;

    struct Command {
        const char* name;
        const char* usage;
        Handler handler;
    };

    Stream& io_;
    Command commands_[kMaxCommands];
    size_t commandCount_ = 0;
    char buffer_[kBufferSize];
    size_t length_ = 0;

    /** Parses and dispatches one command line (modified in place). */
    void dispatch(char* line);
};
