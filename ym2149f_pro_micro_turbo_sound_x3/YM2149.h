#pragma once
#include <Arduino.h>

class YM2149Class {
public:
    void begin();
    void write(uint8_t chip, uint8_t address, uint8_t value);

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

private:
    void selectYM(uint8_t chip);
    void busWrite(uint8_t val);

    static const uint8_t DATA_PINS[8];
    static const uint8_t CHIP_LED[3];
    static const uint8_t PIN_BC1;
    static const uint8_t PIN_BDIR;
    static const uint8_t PIN_SEL_A;
    static const uint8_t PIN_SEL_B;
    static const uint8_t PIN_SEL_C;
    static const uint8_t PIN_ENABLE;

    uint8_t levelValue[3][3] = {{0}};
    uint8_t portAValue[3] = {0};
    uint8_t portBValue[3] = {0};
    uint8_t mixerValue[3] = {0b00111000, 0b00111000, 0b00111000};
    bool ledState[3] = {false, false, false};

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

    static const float YM_CLOCK_HZ = 500000.0f;

    static const uint8_t LED_ON = LOW;
    static const uint8_t LED_OFF = HIGH;

    static const uint8_t YM_0 = 0;
    static const uint8_t YM_1 = 1;
    static const uint8_t YM_2 = 2;
};

typedef YM2149Class YM2149;
