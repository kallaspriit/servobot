#pragma once

#include <Arduino.h>
#include <Print.h>
#include <stdarg.h> // va_list
#include <stdint.h>

// Default to logging errors and warnings.
// 0=OFF, 1=ERROR, 2=WARN, 3=INFO, 4=VERBOSE
#ifndef ST3215_LOG_LEVEL
#    define ST3215_LOG_LEVEL 2
#endif

// Log levels
enum class St3215LogLevel : uint8_t {
    ST3215_LOG_OFF = 0,
    ST3215_LOG_ERROR = 1,
    ST3215_LOG_WARN = 2,
    ST3215_LOG_INFO = 3,
    ST3215_LOG_VERBOSE = 4,
};

// St3215 library logger
class St3215Log {
  public:
    // Constructor, output defaults to Serial
    explicit St3215Log(Print* output = &Serial)
        : output(output) {
    }

    // Set output stream (e.g. Serial)
    void setOutput(Print* out) {
        output = out;
    }

    // Set logging level
    void setLevel(St3215LogLevel lvl) {
        logLevel = lvl;
    }

    // Get current logging level
    St3215LogLevel getLevel() const {
        return logLevel;
    }

    // Enable or disable timestamps in log messages
    void setTimestampsEnabled(bool enabled) {
        showTimestamps = enabled;
    }

    // Provides printf-style logging. Format string should be in PROGMEM: use F("...")
    void logf(St3215LogLevel lvl, const __FlashStringHelper* fmt, ...) {
        if (!shouldLog(lvl)) {
            return;
        }

        char buf[160];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf_P(buf, sizeof(buf), (PGM_P)fmt, ap);
        va_end(ap);
        printPrefix(lvl);
        output->println(buf);
    }

    // Overload for RAM strings (fallback)
    void logf(St3215LogLevel lvl, const char* fmt, ...) {
        if (!shouldLog(lvl)) {
            return;
        }

        char buf[160];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        printPrefix(lvl);
        output->println(buf);
    }

  private:
    // Returns whether a message at this level should be logged
    bool shouldLog(St3215LogLevel lvl) const {
        return ((St3215LogLevel)ST3215_LOG_LEVEL >= lvl) && (lvl <= logLevel) && output;
    }

    // Prints the log message prefix (level, timestamp)
    void printPrefix(St3215LogLevel lvl) {
        if (!output) {
            return;
        }

        output->print(F("ST: "));

        if (showTimestamps) {
            output->print('[');
            output->print(millis());
            output->print(F("] "));
        }

        switch (lvl) {
            case St3215LogLevel::ST3215_LOG_ERROR:
                output->print(F("ERROR "));
                break;

            case St3215LogLevel::ST3215_LOG_WARN:
                output->print(F("WARN "));
                break;

            case St3215LogLevel::ST3215_LOG_VERBOSE:
                output->print(F("VERBOSE "));
                break;

            default:
                break;
        }
    }

    // Output stream to use
    Print* output = &Serial;

    // Current log level
    St3215LogLevel logLevel = (St3215LogLevel)ST3215_LOG_LEVEL;

    // Whether to show timestamps in log messages
    bool showTimestamps = false;
};

// Global logger instance the library uses internally
extern St3215Log ST_LOGGER;

// ---------- Convenience macros (compile-time gated) ----------
#if ST3215_LOG_LEVEL >= 1
#    define ST_LOG_E(fmt, ...) ST_LOGGER.logf(St3215LogLevel::ST3215_LOG_ERROR, F(fmt), ##__VA_ARGS__)
#else
#    define ST_LOG_E(...)                                                                                                                                                                               \
        do {                                                                                                                                                                                           \
        } while (0)
#endif

#if ST3215_LOG_LEVEL >= 2
#    define ST_LOG_W(fmt, ...) ST_LOGGER.logf(St3215LogLevel::ST3215_LOG_WARN, F(fmt), ##__VA_ARGS__)
#else
#    define ST_LOG_W(...)                                                                                                                                                                               \
        do {                                                                                                                                                                                           \
        } while (0)
#endif

#if ST3215_LOG_LEVEL >= 3
#    define ST_LOG_I(fmt, ...) ST_LOGGER.logf(St3215LogLevel::ST3215_LOG_INFO, F(fmt), ##__VA_ARGS__)
#else
#    define ST_LOG_I(...)                                                                                                                                                                               \
        do {                                                                                                                                                                                           \
        } while (0)
#endif

#if ST3215_LOG_LEVEL >= 4
#    define ST_LOG_V(fmt, ...) ST_LOGGER.logf(St3215LogLevel::ST3215_LOG_VERBOSE, F(fmt), ##__VA_ARGS__)
#else
#    define ST_LOG_V(...)                                                                                                                                                                               \
        do {                                                                                                                                                                                           \
        } while (0)
#endif
