#include "YMPlayerSerial.h"

// http://leonard.oxg.free.fr/ymformat.html
// http://lynn3686.com/ym3456_tidy.html

static const uint8_t regMask[14] = {
    0xFF, 0x0F,    // A period low/high
    0xFF, 0x0F,    // B period low/high
    0xFF, 0x0F,    // C period low/high
    0x1F,          // Noise period
    0x3F,          // Mixer
    0x1F,          // A volume
    0x1F,          // B volume
    0x1F,          // C volume
    0xFF, 0xFF,    // Envelope period low/high
    0x0F           // Envelope shape
};

void YMPlayerSerialClass::begin() {
    Ym.begin();

    for (int i = 0; i < 3; ++i) {
        Ym.setPortIO(i, 1, 1);     // Both ports as outputs
        Ym.setPin(i, 0, 1);        // A0 high
        Ym.mute(i);                // Mute this chip
    }

    // Use high-speed CDC for streaming
    Serial.begin(2000000);
}

// Non-blocking update: read full YM packets (17 bytes) and apply at PSG
void YMPlayerSerialClass::update() {
    const int packetSize = 17;
    // Only proceed if a full packet is waiting
    if (Serial.available() >= packetSize) {
        uint8_t buffer[packetSize];
        Serial.readBytes((char*)buffer, packetSize);

        uint8_t chip = buffer[0];
        if (chip > 2) return;

        // Toggle activity LED on YM chip
        Ym.setLED(chip, !Ym.getLED(chip));

        // Write PSG registers
        for (int i = 0; i < 14; ++i) {
            Ym.write(chip, i, buffer[i + 1] & regMask[i]);
        }
    }
}
