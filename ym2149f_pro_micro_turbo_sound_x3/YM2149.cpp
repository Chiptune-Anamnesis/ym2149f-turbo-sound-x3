#include "YM2149.h"

const uint8_t YM2149Class::DATA_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const uint8_t YM2149Class::PIN_BC1      = 10;
const uint8_t YM2149Class::PIN_BDIR     = 20;
const uint8_t YM2149Class::PIN_SEL_A    = A3;
const uint8_t YM2149Class::PIN_SEL_B    = A1;
const uint8_t YM2149Class::PIN_SEL_C    = A0;
const uint8_t YM2149Class::PIN_ENABLE   = A2;
const uint8_t YM2149Class::CHIP_LED[3]  = {15, 14, 16};

void YM2149Class::begin()
{
    for (auto pin : DATA_PINS)
		pinMode(pin, OUTPUT);

    pinMode(PIN_BC1, OUTPUT);
    pinMode(PIN_BDIR, OUTPUT);
    pinMode(PIN_SEL_A, OUTPUT);
    pinMode(PIN_SEL_B, OUTPUT);
    pinMode(PIN_SEL_C, OUTPUT);

    pinMode(PIN_ENABLE, OUTPUT);
    digitalWrite(PIN_ENABLE, HIGH);

    for (int i = 0; i < 3; ++i)
	{
        pinMode(CHIP_LED[i], OUTPUT);
        digitalWrite(CHIP_LED[i], HIGH);
    }

    digitalWrite(PIN_BC1, LOW);
    digitalWrite(PIN_BDIR, LOW);
    digitalWrite(PIN_SEL_A, LOW);
    digitalWrite(PIN_SEL_B, LOW);
    digitalWrite(PIN_SEL_C, LOW);
}

void YM2149Class::busWrite(uint8_t val)
{
    for (uint8_t i = 0; i < 8; ++i)
        digitalWrite(DATA_PINS[i], (val >> i) & 1);
}

void YM2149Class::selectYM(uint8_t chip)
{
    digitalWrite(PIN_SEL_A, chip & 0x01);
    digitalWrite(PIN_SEL_B, (chip >> 1) & 0x01);
    digitalWrite(PIN_SEL_C, (chip >> 2) & 0x01);
}

void YM2149Class::write(uint8_t chip, uint8_t reg, uint8_t val)
{
    selectYM(chip);
    busWrite(reg & 0x1F);
    digitalWrite(PIN_BC1, HIGH);
    digitalWrite(PIN_BDIR, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_BDIR, LOW);
    digitalWrite(PIN_BC1, LOW);
    delayMicroseconds(1);

    busWrite(val);
    digitalWrite(PIN_BDIR, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_BDIR, LOW);
}

void YM2149Class::setPin(uint8_t chip, uint8_t pin, bool value)
{
    pin &= 0x0F;
    uint8_t mask = 1 << (pin & 0x07);

    if (pin >= 8) {
        // Port B
        if (value)
            portBValue[chip] |= mask;
        else
            portBValue[chip] &= ~mask;

        write(chip, REG_DATAPORT_B, portBValue[chip]);
    } else {
        // Port A
        if (value)
            portAValue[chip] |= mask;
        else
            portAValue[chip] &= ~mask;

        write(chip, REG_DATAPORT_A, portAValue[chip]);
    }
}

uint8_t YM2149Class::getPin(uint8_t chip, uint8_t pin)
{
    pin &= 0x0F;

    if (pin >= 8) {
        // Port B
        return (portBValue[chip] >> (pin - 8)) & 0x01;
    } else {
        // Port A
        return (portAValue[chip] >> pin) & 0x01;
    }
}

void YM2149Class::setPort(uint8_t chip, bool port, uint8_t value)
{
    if (port) {
        portBValue[chip] = value;
        write(chip, REG_DATAPORT_B, value);
    } else {
        portAValue[chip] = value;
        write(chip, REG_DATAPORT_A, value);
    }
}

uint8_t YM2149Class::getPort(uint8_t chip, bool port)
{
    return port ? portBValue[chip] : portAValue[chip];
}

void YM2149Class::setPortIO(uint8_t chip, bool portAOut, bool portBOut)
{
    // Bits 6 (port A) and 7 (port B) control I/O mode in the mixer register
    uint8_t portSettings = ((portBOut ? 1 : 0) << 7) | ((portAOut ? 1 : 0) << 6);
    mixerValue[chip] = (mixerValue[chip] & 0b00111111) | portSettings;
    write(chip, REG_MIXER, mixerValue[chip]);
}

void YM2149Class::setLED(uint8_t chip, bool state)
{
    ledState[chip] = state;
    digitalWrite(CHIP_LED[chip], ledState[chip] ? LED_ON : LED_OFF);
}

bool YM2149Class::getLED(uint8_t chip)
{
    return ledState[chip];
}

void YM2149Class::setNote(uint8_t chip, uint8_t voice, float midiNote)
{
    uint16_t freqVal;

    if (voice != 3) {
        // Convert MIDI note to frequency
        float freq = 440.0f * powf(2.0f, (midiNote - 69.0f) / 12.0f);

        if (voice == 4) {
            // Envelope frequency: 500kHz / (256 * freq)
            freqVal = static_cast<uint16_t>((YM_CLOCK_HZ / (256.0f * freq)) + 0.5f);
        } else {
            // Tone channels: 500kHz / (16 * freq)
            freqVal = static_cast<uint16_t>((YM_CLOCK_HZ / (16.0f * freq)) + 0.5f);
        }
    } else {
        // Noise: approximate mapping
        freqVal = constrain(static_cast<uint16_t>(31 - midiNote / 4), 0, 31);
    }

    setTone(chip, voice, freqVal);
}

void YM2149Class::setFreq(uint8_t chip, uint8_t voice, uint32_t freqHz)
{
    if (freqHz == 0) return; // avoid divide-by-zero

    uint16_t divisor = (voice == 4) ? 256 : 16; // 256 for envelope, 16 for tone
    uint16_t freqVal = static_cast<uint16_t>((YM_CLOCK_HZ / freqHz) / divisor + 0.5f);
    setTone(chip, voice, freqVal);
}

void YM2149Class::setTone(uint8_t chip, uint8_t voice, uint16_t value)
{
    switch (voice)
    {
        case 0: // Channel A
            write(chip, REG_A_FREQ,     value & 0xFF);
            write(chip, REG_A_FREQ + 1, (value >> 8) & 0x0F);
            break;

        case 1: // Channel B
            write(chip, REG_B_FREQ,     value & 0xFF);
            write(chip, REG_B_FREQ + 1, (value >> 8) & 0x0F);
            break;

        case 2: // Channel C
            write(chip, REG_C_FREQ,     value & 0xFF);
            write(chip, REG_C_FREQ + 1, (value >> 8) & 0x0F);
            break;

        case 3: // Noise (5 bits)
            write(chip, REG_NOISE_FREQ, value & 0x1F);
            break;

        case 4: // Envelope
            // IMPORTANT: value must already be calculated as:
            // YM_CLOCK_HZ / (256.0f * frequency) + 0.5
            write(chip, REG_ENV_FREQ,     value & 0xFF);
            write(chip, REG_ENV_FREQ + 1, (value >> 8) & 0x0F);
            break;
    }
}

void YM2149Class::setVolume(uint8_t chip, uint8_t voice, uint8_t value)
{
    if (voice > 2) return;
    value &= 0x0F;
    levelValue[chip][voice] = (levelValue[chip][voice] & 0x10) | value;
    write(chip, REG_A_LEVEL + voice, levelValue[chip][voice]);
}

void YM2149Class::setNoise(uint8_t chip, uint8_t voice, uint8_t mode)
{
    if (voice > 2) return;

    uint8_t toneMask  = 1 << voice;        // Bits 0,1,2 for tone A/B/C
    uint8_t noiseMask = 1 << (voice + 3);  // Bits 3,4,5 for noise A/B/C

    // Clear only this voice's tone+noise mask from the mixer
    mixerValue[chip] &= ~(toneMask | noiseMask);

    switch (mode)
    {
        case 1: // Tone only
            mixerValue[chip] |= noiseMask;
            break;

        case 2: // Noise only
            mixerValue[chip] |= toneMask;
            break;

        case 3: // Mute both (optional case)
            mixerValue[chip] |= (toneMask | noiseMask);
            break;

        case 0: // Both tone and noise enabled
        default:
            // leave both cleared
            break;
    }

    write(chip, REG_MIXER, mixerValue[chip]);
}

void YM2149Class::setEnv(uint8_t chip, uint8_t voice, uint8_t value)
{
    if (voice > 2) return;
    uint8_t *state = &levelValue[chip][voice];

    value &= 0b00000001;
    value <<= 4; // Bit 4 enables envelope for this voice
    *state = (*state & 0x0F) | value;
    write(chip, REG_A_LEVEL + voice, *state);
}

void YM2149Class::setEnvShape(uint8_t chip, uint8_t cont, uint8_t att, uint8_t alt, uint8_t hold)
{
    uint8_t val = ((cont & 1) << 3) | ((att & 1) << 2) | ((alt & 1) << 1) | (hold & 1);

    // Force re-trigger envelope
    write(chip, REG_ENV_SHAPE, 0);   // dummy write to retrigger
    write(chip, REG_ENV_SHAPE, val); // real value
}

void YM2149Class::mute(uint8_t chip)
{
    for (uint8_t v = 0; v < 3; ++v) {
        levelValue[chip][v] = 0;
        write(chip, REG_A_LEVEL + v, 0);
    }
    write(chip, REG_MIXER, 0b00111000); // disable tone
}
