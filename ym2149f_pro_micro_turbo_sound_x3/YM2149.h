#pragma once
#include <Arduino.h>

#define DIRECT_WRITE

class YM2149Class {

public:
    static constexpr uint8_t DATA_PINS_D = 0b11111100; // D2-D7
    static constexpr uint8_t DATA_PINS_B = 0b00000011; // D8-D9
    static constexpr uint8_t BC1_BIT = _BV(PB6);    // D10 → PB6
    static constexpr uint8_t BDIR_BIT = _BV(PF5);   // D20 → PF5
    static constexpr uint8_t SEL_A_BIT = _BV(PF4);  // A3 → PF4
    static constexpr uint8_t SEL_B_BIT = _BV(PF6);  // A1 → PF6
    static constexpr uint8_t SEL_C_BIT = _BV(PF7);  // A0 → PF7
    static constexpr uint8_t ENABLE_BIT = _BV(PF5); // A2 → PF5

    /* -------- physical pins -------- */
    static constexpr uint8_t PIN_BC1   = 10;
    static constexpr uint8_t PIN_BDIR  = 20;
    static constexpr uint8_t PIN_SEL_A = A3;
    static constexpr uint8_t PIN_SEL_B = A1;
    static constexpr uint8_t PIN_SEL_C = A0;
    static constexpr uint8_t PIN_ENABLE= A2;
    static constexpr uint8_t CHIP_LED[3] = {15, 14, 16};

    static constexpr float YM_CLOCK_HZ = 500000.0f;

    static constexpr uint8_t LED_ON = LOW;
    static constexpr uint8_t LED_OFF = HIGH;

    static constexpr uint8_t YM_0 = 0;
    static constexpr uint8_t YM_1 = 1;
    static constexpr uint8_t YM_2 = 2;

    void begin();
    void write(uint8_t chip, uint8_t address, uint8_t value);

#ifdef DIRECT_WRITE
    inline void selectYM(uint8_t chip)
    {
        chip = 2 - chip;

        PORTF &= ~(SEL_A_BIT | SEL_B_BIT | SEL_C_BIT);

        if (chip & 1) PORTF |= SEL_A_BIT;
        if (chip & 2) PORTF |= SEL_B_BIT;
        if (chip & 4) PORTF |= SEL_C_BIT;
    }

    inline void busWrite(uint8_t value)
    {
        // --- Build PORTD value (PD0, PD1, PD4, PD7) ---
        uint8_t portd_mask = _BV(PD0) | _BV(PD1) | _BV(PD4) | _BV(PD7);
        uint8_t portd_val  =
            ((value & (1 << 0)) ? _BV(PD1) : 0) |
            ((value & (1 << 1)) ? _BV(PD0) : 0) |
            ((value & (1 << 2)) ? _BV(PD4) : 0) |
            ((value & (1 << 4)) ? _BV(PD7) : 0);
        PORTD = (PORTD & ~portd_mask) | portd_val;

        // --- Build PORTC value (PC6) ---
        PORTC = (PORTC & ~_BV(PC6)) | ((value & (1 << 3)) ? _BV(PC6) : 0);

        // --- Build PORTE value (PE6) ---
        PORTE = (PORTE & ~_BV(PE6)) | ((value & (1 << 5)) ? _BV(PE6) : 0);

        // --- Build PORTB value (PB4, PB5) ---
        uint8_t portb_mask = _BV(PB4) | _BV(PB5);
        uint8_t portb_val  =
            ((value & (1 << 6)) ? _BV(PB4) : 0) |
            ((value & (1 << 7)) ? _BV(PB5) : 0);
        PORTB = (PORTB & ~portb_mask) | portb_val;
    }

    inline void writeFast(uint8_t address, uint8_t value)
    {
        busWrite(address);             // 4
        PORTB |=  _BV(PB6);            // 1 – BC1
        PORTF |=  _BV(PF5);            // 1 – BDIR
        _NOP(); _NOP(); _NOP(); _NOP();// 4 – 250 ns
        PORTF &= ~_BV(PF5);            // 1 – BDIR
        PORTB &= ~_BV(PB6);            // 1 – BC1
        busWrite(value);               // 4
        PORTF |=  _BV(PF5);            // 1 – data strobe
        PORTF &= ~_BV(PF5);            // 1 – finish
    }

#else
    void busWrite(uint8_t val);
#endif

    void setPin(uint8_t chip, uint8_t pin, bool value);
    uint8_t getPin(uint8_t chip, uint8_t pin);

    void setPort(uint8_t chip, bool port, uint8_t value);
    uint8_t getPort(uint8_t chip, bool port);
    void setPortIO(uint8_t chip, bool portA, bool portB);

    void setLED(uint8_t chip, bool state);
    bool getLED(uint8_t chip);

	void setNote(uint8_t chip, uint8_t synth, float value);
	void setFreq(uint8_t chip, uint8_t voice, uint32_t freqHz);
    void setTone(uint8_t chip, uint8_t voice, uint16_t value);
    void setVolume(uint8_t chip, uint8_t voice, uint8_t value);
    void setNoise(uint8_t chip, uint8_t voice, uint8_t value);
    void setEnv(uint8_t chip, uint8_t voice, uint8_t value);
    void setEnvShape(uint8_t chip, uint8_t cont, uint8_t att, uint8_t alt, uint8_t hold);
    void mute(uint8_t chip);

	static const uint8_t REG_A_FREQ = 0x00;
    static const uint8_t REG_B_FREQ = 0x02;
    static const uint8_t REG_C_FREQ = 0x04;
    static const uint8_t REG_NOISE_FREQ = 0x06;
    static const uint8_t REG_MIXER = 0x07;
    static const uint8_t REG_A_LEVEL = 0x08;
    static const uint8_t REG_B_LEVEL = 0x09;
    static const uint8_t REG_C_LEVEL = 0x0A;
    static const uint8_t REG_ENV_FREQ = 0x0B;
    static const uint8_t REG_ENV_SHAPE = 0x0D;
    static const uint8_t REG_DATAPORT_A = 0x0E;
    static const uint8_t REG_DATAPORT_B = 0x0F;

    uint8_t currentChip = 255;

private:
    uint8_t levelValue[3][3] = {{0}};
    uint8_t portAValue[3] = {0};
    uint8_t portBValue[3] = {0};
    uint8_t mixerValue[3] = {0b00111000, 0b00111000, 0b00111000};
    bool ledState[3] = {false, false, false};
};

typedef YM2149Class YM2149;
