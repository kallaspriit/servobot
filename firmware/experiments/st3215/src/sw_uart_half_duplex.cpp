#include "sw_uart_half_duplex.hpp"

#include "hardware/pio.h"
#include "pico/time.h"

// Both UART programs run at 8 cycles per bit (8x oversampling), the same
// scheme the pico-examples UART uses. The state-machine clock is therefore
// baud * 8.
static constexpr uint kCyclesPerBit = 8;

// 8N1 frame layout shifted out LSB first: bit 0 = start (low),
// bits 1..8 = data, bit 9 = stop (high). Ten bits total.
static constexpr uint kBitsPerFrame = 10;

bool SwUartHalfDuplex::begin(uint pin, uint baud, PIO pio) {
    pio_ = pio;
    pin_ = pin;
    frameUs_ = (float)kBitsPerFrame * 1e6f / (float)baud;

    // TX program (jmp target is 0-based; the loader relocates it by the load
    // offset). Idles by stalling on `pull` while holding the last (stop/high)
    // pin level.
    //
    //   pull block
    //   set x, 9            ; 10 bits to shift out (start + 8 data + stop)
    // bitloop:
    //   out pins, 1 [6]     ; 1 + 6 = 7 cycles
    //   jmp x-- bitloop     ; + 1 cycle = 8 cycles per bit
    const uint16_t txProgram[] = {
        pio_encode_pull(false, true),
        pio_encode_set(pio_x, kBitsPerFrame - 1),
        pio_encode_out(pio_pins, 1) | pio_encode_delay(kCyclesPerBit - 2),
        pio_encode_jmp_x_dec(2),
    };
    struct pio_program txPgm = {txProgram, count_of(txProgram), -1};

    // RX program. Waits for the falling edge of a start bit, delays 1.5 bit
    // times to the centre of the first data bit, then samples 8 bits. Autopush
    // delivers the byte once 8 bits are in.
    //
    // start:
    //   wait 0 pin 0
    //   set x, 7 [10]       ; 11 cycles ~ 1.5 bit times to first sample
    // bitloop:
    //   in pins, 1          ; 1 cycle
    //   jmp x-- bitloop [6] ; + 7 cycles = 8 cycles per bit
    const uint16_t rxProgram[] = {
        pio_encode_wait_pin(false, 0),
        pio_encode_set(pio_x, 7) | pio_encode_delay(10),
        pio_encode_in(pio_pins, 1),
        pio_encode_jmp_x_dec(2) | pio_encode_delay(kCyclesPerBit - 2),
    };
    struct pio_program rxPgm = {rxProgram, count_of(rxProgram), -1};

    int smTx = pio_claim_unused_sm(pio_, false);
    int smRx = pio_claim_unused_sm(pio_, false);

    if (smTx < 0 || smRx < 0) {
        return false;
    }

    smTx_ = (uint)smTx;
    smRx_ = (uint)smRx;
    offsetTx_ = pio_add_program(pio_, &txPgm);
    offsetRx_ = pio_add_program(pio_, &rxPgm);

    const float clockDiv = (float)clock_get_hz(clk_sys) / (float)(kCyclesPerBit * baud);

    // Hand the pin to the PIO before configuring the state machines.
    pio_gpio_init(pio_, pin_);

    // TX state machine: drives `pin_` via OUT, LSB first, no autopull.
    pio_sm_config txConfig = pio_get_default_sm_config();
    sm_config_set_wrap(&txConfig, offsetTx_, offsetTx_ + count_of(txProgram) - 1);
    sm_config_set_out_pins(&txConfig, pin_, 1);
    sm_config_set_out_shift(&txConfig, true, false, 32);
    sm_config_set_fifo_join(&txConfig, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&txConfig, clockDiv);
    pio_sm_init(pio_, smTx_, offsetTx_, &txConfig);

    // Preload the output latch high (idle level) and start released (input), so
    // we only ever drive the bus while actively transmitting.
    pio_sm_set_pins_with_mask(pio_, smTx_, 1u << pin_, 1u << pin_);
    pio_sm_set_consecutive_pindirs(pio_, smTx_, pin_, 1, false);
    pio_sm_set_enabled(pio_, smTx_, true);

    // RX state machine: samples `pin_` via IN, LSB first, autopush every 8 bits.
    pio_sm_config rxConfig = pio_get_default_sm_config();
    sm_config_set_wrap(&rxConfig, offsetRx_, offsetRx_ + count_of(rxProgram) - 1);
    sm_config_set_in_pins(&rxConfig, pin_);
    sm_config_set_in_shift(&rxConfig, true, true, 8);
    sm_config_set_fifo_join(&rxConfig, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&rxConfig, clockDiv);
    pio_sm_init(pio_, smRx_, offsetRx_, &rxConfig);
    pio_sm_set_enabled(pio_, smRx_, true);

    return true;
}

void SwUartHalfDuplex::writeBytes(const uint8_t *data, size_t len) {
    // Stop listening so we never capture our own transmission as echo.
    pio_sm_set_enabled(pio_, smRx_, false);

    // Drive the bus.
    pio_sm_set_consecutive_pindirs(pio_, smTx_, pin_, 1, true);

    for (size_t i = 0; i < len; i++) {
        const uint32_t frame = ((uint32_t)data[i] << 1) | (1u << 9);
        pio_sm_put_blocking(pio_, smTx_, frame);
    }

    // Wait until every queued frame has been pulled out of the FIFO.
    while (!pio_sm_is_tx_fifo_empty(pio_, smTx_)) {
        tight_loop_contents();
    }

    // Then wait until the final frame, including its stop bit, has actually been
    // shifted onto the wire. The TX-stall flag asserts when the state machine
    // stalls on the next (empty) pull, which only happens once the last bit is
    // out. Clearing it here is safe because the last frame is still shifting at
    // this point, so the SM is not yet stalled. This lets us release the bus the
    // moment our transmission ends, instead of after a guessed delay, so a fast
    // servo reply always lands on a freed line.
    const uint32_t txStallMask = 1u << (PIO_FDEBUG_TXSTALL_LSB + smTx_);
    pio_->fdebug = txStallMask;

    while (!(pio_->fdebug & txStallMask)) {
        tight_loop_contents();
    }

    // Pre-arm the receiver while still holding the line high (the RX wait does
    // not false-trigger on a high line), then release the bus to high-Z so the
    // servo can drive its reply into an already-listening receiver.
    restartRx();
    pio_sm_set_consecutive_pindirs(pio_, smTx_, pin_, 1, false);
}

size_t SwUartHalfDuplex::readBytes(uint8_t *buf, size_t len, uint32_t timeoutUs) {
    size_t count = 0;
    absolute_time_t deadline = make_timeout_time_us(timeoutUs);

    while (count < len) {
        if (!pio_sm_is_rx_fifo_empty(pio_, smRx_)) {
            const uint32_t value = pio_sm_get(pio_, smRx_);

            // Sampled bits land in the top byte of the 32-bit FIFO word.
            buf[count++] = (uint8_t)(value >> 24);
            deadline = make_timeout_time_us(timeoutUs);

            continue;
        }

        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            break;
        }
    }

    return count;
}

void SwUartHalfDuplex::flushRx() {
    pio_sm_clear_fifos(pio_, smRx_);
}

int SwUartHalfDuplex::rxAvailable() {
    return (int)pio_sm_get_rx_fifo_level(pio_, smRx_);
}

int SwUartHalfDuplex::readByteNonBlocking() {
    if (pio_sm_is_rx_fifo_empty(pio_, smRx_)) {
        return -1;
    }

    const uint32_t value = pio_sm_get(pio_, smRx_);

    return (int)(uint8_t)(value >> 24);
}

void SwUartHalfDuplex::restartRx() {
    pio_sm_set_enabled(pio_, smRx_, false);
    pio_sm_clear_fifos(pio_, smRx_);
    pio_sm_restart(pio_, smRx_);
    pio_sm_exec(pio_, smRx_, pio_encode_jmp(offsetRx_));
    pio_sm_set_enabled(pio_, smRx_, true);
}
