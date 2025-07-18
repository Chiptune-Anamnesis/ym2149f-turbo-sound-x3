#include <Arduino.h>
#include <MIDIUSB.h>
#include <math.h>

// Toggle YM file streaming via USB CDC
#define USE_YMPLAYER_SERIAL 0
#if USE_YMPLAYER_SERIAL
  #include "YMPlayerSerial.h"
#endif

#define LED_ON  LOW
#define LED_OFF HIGH

// YM2149F control pins
const uint8_t DATA_PINS[8]  = {2,3,4,5,6,7,8,9};
const uint8_t PIN_BC1       = 10;
const uint8_t PIN_BDIR      = 20;
const uint8_t PIN_SEL_A     = A3;
const uint8_t PIN_SEL_B     = A1;
const uint8_t PIN_SEL_C     = A0;
const uint8_t PIN_ENABLE    = A2;

// LEDs per chip
const uint8_t CHIP_LED[3]   = {15,14,16};
// MIDI→chip map
const uint8_t midiToChip[9] = {0,0,0, 1,1,1, 2,2,2};

const float YM_CLOCK_HZ = 500000.0f;

// ——— placeholder tables ———
// Paste your full tables here:
const uint16_t freqTable[1281]        = { /* … */ };
const uint16_t softFreqTable[1281]    = { /* … */ };
const uint8_t  volumeEnvelopeTable[256] = { /* … */ };

#if USE_YMPLAYER_SERIAL
  YMPlayerSerial player;
#endif

// per-channel state
float   modWheel[9]            = {0};
float   vibPhase[9]            = {0};
float   vibRate[9]             = {4.0f,4.0f,4.0f,4.0f,4.0f,4.0f,4.0f,4.0f,4.0f};
float   vibRangeSemi[9]        = {0.3f,0.3f,0.3f,0.3f,0.3f,0.3f,0.3f,0.3f,0.3f};
unsigned long vibStartTime[9]  = {0};          // timestamp when note-on occurred
uint16_t vibDelayMs[9]         = {0};          // per-channel vibrato delay in ms

float   pitchBendSemis[9]      = {0};
float   pitchEnvPhase[9]       = {0};
float   pitchEnvIncrement[9]   = {0};
float   pitchEnvAmt[9]         = {0};
uint8_t pitchEnvShape[9]       = {0};
uint8_t expressionVal[9]       = {127,127,127,127,127,127,127,127,127};
bool    portamentoOn[9]        = {false};
float   portamentoSpeed[9]     = {0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f};
const float PORTA_MIN = 0.005f;
const float PORTA_MAX = 0.5f;
bool    laserMode[9]   = { false,false,false,false,false,false,false,false,false };
float   laserAmt[9]    = { 1,1,1,1,1,1,1,1,1 };  // 1.0 = full zero jump, 0.0 = no jump
bool    laserTriggered[3][3] = {{false}};  // per-voice one-shot flag

// CC4 volume envelope
uint8_t cc4Shape[9]            = {0};
bool    volEnvOn[9]            = {false};
bool    volEnvDir[9]           = {true};   // true=ramp up, false=ramp down
float   volEnvPhase[9]         = {0.0f};
float   volEnvIncrement[9]     = {0.0f};
bool sustainOn[9] = {
  false, false, false,
  false, false, false,
  false, false, false
};
bool    pendingRelease[3][3]   = {{false}};

// per-voice state
bool    voiceActive[3][3]      = {{false}};
uint8_t voiceNote[3][3]        = {{0}};
uint8_t voiceChan[3][3]        = {{0}};
uint8_t voiceVol[3][3]         = {{0}};
uint8_t nextVoice[3]           = {0};
float   curPeriod[3][3]        = {{0}};

// LED flash timers
unsigned long ledOnTime[3]     = {0};
const unsigned long ledFlashMs = 100;

// MIDI parser
enum SerState { WAIT_STATUS, WAIT_D1, WAIT_D2 };
static SerState serState        = WAIT_STATUS;
static uint8_t serStatus, serD1;

// Helpers
uint16_t noteToPeriod(uint8_t note) {
  float freq = 440.0f * powf(2.0f, ((int)note - 69) / 12.0f);
  return (uint16_t)(YM_CLOCK_HZ / (16.0f * freq) + 0.5f);
}

void busWrite(uint8_t val) {
  for(uint8_t i=0; i<8; i++)
    digitalWrite(DATA_PINS[i], (val >> i) & 1);
}

void selectYM(uint8_t chip) {
  chip = 2 - chip;
  digitalWrite(PIN_SEL_A, (chip >> 0) & 1);
  digitalWrite(PIN_SEL_B, (chip >> 1) & 1);
  digitalWrite(PIN_SEL_C, (chip >> 2) & 1);
}

void psgWrite(uint8_t chip, uint8_t reg, uint8_t val) {
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

void setVoice(uint8_t chip, uint8_t v, uint16_t per, uint8_t vol) {
  psgWrite(chip, v*2,        per & 0xFF);
  psgWrite(chip, v*2 + 1, (per >> 8) & 0x0F);
  psgWrite(chip, 8 + v,     vol & 0x0F);
}

void stopVoice(uint8_t chip, uint8_t v) {
  setVoice(chip, v, 0, 0);
}

void enableTones(uint8_t chip) {
  psgWrite(chip, 7, 0b00111000);
}

// Core update (~5ms)
// ——— Modified updatePitchMod (bounds‑checks ch 0–15) ———
void updatePitchMod(uint8_t ch) {
  if (ch >= 16) return;
  uint8_t chip = midiToChip[ch];
  unsigned long now = millis();

  // vibrato LFO
  float lfo = 0;
  if (now - vibStartTime[ch] >= vibDelayMs[ch]) {
    vibPhase[ch] += vibRate[ch] * 0.005f;
    if (vibPhase[ch] >= 1.0f) vibPhase[ch] -= 1.0f;
    lfo = sinf(vibPhase[ch] * 2 * PI)
        * (modWheel[ch] / 127.0f)
        * vibRangeSemi[ch];
  }

  for (int v = 0; v < 3; v++) {
    if (!voiceActive[chip][v] || voiceChan[chip][v] != ch) continue;

    // target period with pitch‑bend + vibrato
    float base = noteToPeriod(voiceNote[chip][v]);
    float tp   = base / powf(2.0f, (pitchBendSemis[ch] + lfo) / 12.0f);

    // pitch envelope
    if (pitchEnvAmt[ch] > 0) {
      pitchEnvPhase[ch] += pitchEnvIncrement[ch];
      if (pitchEnvPhase[ch] > 1.0f) pitchEnvPhase[ch] = 1.0f;
      float ev = (pitchEnvShape[ch] < 64)
                   ? pitchEnvPhase[ch]
                   : 1.0f - pitchEnvPhase[ch];
      tp /= powf(2.0f, (pitchEnvAmt[ch] * ev) / 12.0f);
    }

    // one‑shot laser
    if (laserTriggered[chip][v]) {
      laserTriggered[chip][v] = false;
      // curPeriod stays as set by noteOn()
    }
    // portamento slide
    else if (portamentoOn[ch]) {
      curPeriod[chip][v] += (tp - curPeriod[chip][v]) * portamentoSpeed[ch];
    }
    // immediate jump
    else {
      curPeriod[chip][v] = tp;
    }

    // compute volume
    uint16_t outP = uint16_t(curPeriod[chip][v] + 0.5f);
    uint8_t vol   = (voiceVol[chip][v] * expressionVal[ch] + 63) / 127;

    // CC4 volume envelope
    if (volEnvOn[ch]) {
      if (volEnvDir[ch]) {
        volEnvPhase[ch] += volEnvIncrement[ch];
        if (volEnvPhase[ch] >= 1.0f) {
          volEnvPhase[ch] = 1.0f;
          volEnvOn[ch]    = false;
        }
      } else {
        volEnvPhase[ch] -= volEnvIncrement[ch];
        if (volEnvPhase[ch] <= 0.0f) {
          volEnvPhase[ch] = 0.0f;
          volEnvOn[ch]    = false;
        }
      }
      vol = uint8_t(vol * volEnvPhase[ch] + 0.5f);
    }

    setVoice(chip, v, outP, vol);
  }
}

void noteOn(uint8_t ch, uint8_t note, uint8_t vel) {
  // ——— special noise on MIDI Channel 10 (ch==9) using chip 2 ———
  if (ch == 9) {
    noiseOn(note, vel);
    return;
  }

  // ——— existing tone‐voice handling for channels 1–9 ———

  // start vibrato delay timer
  vibStartTime[ch] = millis();
  vibPhase[ch]     = 0;

  uint8_t chip = midiToChip[ch];
  uint8_t v;
  // find a free voice, or round-robin if all are active
  for (v = 0; v < 3; v++) {
    if (!voiceActive[chip][v]) break;
  }
  if (v >= 3) {
    v = nextVoice[chip];
    nextVoice[chip] = (v + 1) % 3;
  }

  // assign voice parameters
  voiceActive[chip][v] = true;
  voiceNote[chip][v]   = note;
  voiceChan[chip][v]   = ch;
  voiceVol[chip][v]    = vel >> 3;

  // LASER-JUMP EFFECT
  float prevP = curPeriod[chip][v];
  if (laserMode[ch]) {
    curPeriod[chip][v] = prevP * (1.0f - laserAmt[ch]);
  }
  // NORMAL PORTAMENTO ZEROING
  else if (!portamentoOn[ch]) {
    curPeriod[chip][v] = 0;
  }
  // else leave curPeriod untouched for glide

  // reset pitch envelope phase
  pitchEnvPhase[ch] = 0;

  // CC4 volume envelope trigger
  if (cc4Shape[ch] == 0) {
    volEnvOn[ch] = false;
  } else if (cc4Shape[ch] < 64) {
    volEnvOn[ch]        = true;
    volEnvDir[ch]       = true;
    volEnvPhase[ch]     = 0;
    uint16_t t = map(cc4Shape[ch], 1, 63, 20, 200);
    volEnvIncrement[ch] = 1.0f / t;
  } else {
    volEnvOn[ch]        = true;
    volEnvDir[ch]       = false;
    volEnvPhase[ch]     = 1.0f;
    uint16_t t = map(cc4Shape[ch], 64, 127, 20, 200);
    volEnvIncrement[ch] = 1.0f / t;
  }

  // apply pitch (with laser or portamento)
  updatePitchMod(ch);

  // flash LED on the tone-chip
  digitalWrite(CHIP_LED[chip], LED_ON);
  ledOnTime[chip] = millis();
}

void noteOff(uint8_t ch, uint8_t note) {
  // ——— special noise-off on MIDI Channel 10 (ch==9) using chip 2 ———
  if (ch == 9) {
    noiseOff();
    return;
  }

  // ——— sustain pedal logic for channels 1–9 ———
  if (sustainOn[ch]) {
    uint8_t chip = midiToChip[ch];
    for (uint8_t v = 0; v < 3; v++) {
      if (voiceActive[chip][v] && voiceNote[chip][v] == note) {
        pendingRelease[chip][v] = true;
        break;
      }
    }
    return;
  }

  // ——— immediate note-off for channels 1–9 ———
  {
    uint8_t chip = midiToChip[ch];
    for (uint8_t v = 0; v < 3; v++) {
      if (voiceActive[chip][v] && voiceNote[chip][v] == note) {
        voiceActive[chip][v] = false;
        stopVoice(chip, v);
        pendingRelease[chip][v] = false;
        break;
      }
    }
  }
}



void pitchBend(uint8_t ch, uint8_t lsb, uint8_t msb) {
  int val = (msb << 7) | lsb;
  pitchBendSemis[ch] = ((float)val - 8192) / 8192 * 2.0f;
  updatePitchMod(ch);
}

// ——— Modified handleMidiMsg — now allows CC on channels 0–15
// ——— Modified handleMidiMsg (now allows CC on 0–15) ———
void handleMidiMsg(uint8_t status, uint8_t d1, uint8_t d2) {
  uint8_t cmd = status & 0xF0;
  uint8_t ch  = status & 0x0F;

  if (cmd == 0x90 && d2 > 0) {
    noteOn(ch, d1, d2);
  }
  else if (cmd == 0x80 || (cmd == 0x90 && d2 == 0)) {
    noteOff(ch, d1);
  }
  else if (cmd == 0xB0 && ch < 16) {  // ← expanded
    switch (d1) {
      case 1:   modWheel[ch]     = d2; updatePitchMod(ch); break;
      case 4:   cc4Shape[ch]     = d2;                          break;
      case 5: { // portamento speed
        float norm  = d2 / 127.0f;
        float curve = norm * norm;
        portamentoSpeed[ch] = 0.5f - (0.5f - PORTA_MIN) * curve;
        updatePitchMod(ch);
      } break;
      case 7:
      case 11:  expressionVal[ch] = d2; updatePitchMod(ch);     break;
      case 9:   pitchEnvAmt[ch]       = (d2 / 127.0f) * 2.0f;
                  pitchEnvIncrement[ch] = 1.0f / (200.0f / 5.0f); break;
      case 10:  pitchEnvShape[ch] = d2;                          break;
      case 64:  sustainOn[ch]     = (d2 >= 64);
                  if (!sustainOn[ch]) {
                    uint8_t chip = midiToChip[ch];
                    for (uint8_t v = 0; v < 3; v++) {
                      if (pendingRelease[chip][v]
                       && voiceActive[chip][v]
                       && voiceChan[chip][v] == ch) {
                        voiceActive[chip][v]   = false;
                        stopVoice(chip, v);
                        pendingRelease[chip][v] = false;
                      }
                    }
                  }
                  break;
      case 65:  {
                  bool on = (d2 >= 64);
                  if (!on) {
                    uint8_t chip = midiToChip[ch];
                    for (uint8_t v = 0; v < 3; v++) {
                      if (voiceActive[chip][v]
                       && voiceChan[chip][v] == ch) {
                        curPeriod[chip][v] = 0;
                      }
                    }
                  }
                  portamentoOn[ch] = on;
                  updatePitchMod(ch);
                }
                break;
      case 68:  laserMode[ch]     = (d2 >= 64);                 break;
      case 69:  laserAmt[ch]      = d2 / 127.0f;                break;
      case 76:  vibRate[ch]       = (d2 / 127.0f) * 10.0f; updatePitchMod(ch); break;
      case 77:  vibRangeSemi[ch]  = (d2 / 127.0f) * 2.0f;  updatePitchMod(ch); break;
      case 85:  vibDelayMs[ch]    = map(d2, 0, 127, 0, 2000);   break;
      case 120:
      case 123: {
                  uint8_t chip = midiToChip[ch];
                  for (uint8_t v = 0; v < 3; v++) {
                    if (voiceActive[chip][v]
                     && voiceChan[chip][v] == ch) {
                      stopVoice(chip, v);
                      voiceActive[chip][v] = false;
                    }
                  }
                } break;
    }
  }
  else if (cmd == 0xE0) {
    pitchBend(ch, d1, d2);
  }
}

void parseSerialMidi(uint8_t b) {
  // real‑time messages (0xF8–0xFF): ignore, don't disturb serState
  if (b >= 0xF8) {
    return;
  }

  // system‑common (0xF0–0xF7): skip and re‑sync parser
  if (b >= 0xF0) {
    serState = WAIT_STATUS;
    return;
  }

  // channel status bytes (0x80–0xEF)
  if (b & 0x80) {
    serStatus = b;
    serState  = WAIT_D1;
  }
  // first data byte
  else if (serState == WAIT_D1) {
    serD1     = b;
    serState  = WAIT_D2;
  }
  // second data byte → full MIDI message
  else if (serState == WAIT_D2) {
    handleMidiMsg(serStatus, serD1, b);
    serState  = WAIT_D1;
  }
  // otherwise (data in WAIT_STATUS): just drop it
}

void setup() {
  for (auto p : DATA_PINS) pinMode(p, OUTPUT);
  pinMode(PIN_BC1, OUTPUT);
  pinMode(PIN_BDIR, OUTPUT);
  pinMode(PIN_SEL_A, OUTPUT);
  pinMode(PIN_SEL_B, OUTPUT);
  pinMode(PIN_SEL_C, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(CHIP_LED[i], OUTPUT);
    digitalWrite(CHIP_LED[i], LED_OFF);
  }
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(CHIP_LED[i], LED_ON);
    delay(200);
    digitalWrite(CHIP_LED[i], LED_OFF);
    delay(100);
  }
  for (uint8_t c = 0; c < 3; c++) {
    enableTones(c);
    for (uint8_t v = 0; v < 3; v++) stopVoice(c, v);
  }
    for (uint8_t ch = 0; ch < 9; ++ch) {
    sustainOn[ch] = false;
  }

  for (uint8_t ch = 0; ch < 16; ++ch) {
    sustainOn[ch] = false;
  }
  Serial1.begin(31250);
#if USE_YMPLAYER_SERIAL
  Serial.begin(115200);
  player.begin();
#endif
}

// ——— Modified loop() — calls updatePitchMod(ch) for 0…15 ———
void loop() {
  while (Serial1.available()) parseSerialMidi(Serial1.read());

  unsigned long now = millis();
  for (uint8_t c = 0; c < 3; c++) {
    if (ledOnTime[c] && now - ledOnTime[c] >= ledFlashMs) {
      digitalWrite(CHIP_LED[c], LED_OFF);
      ledOnTime[c] = 0;
    }
  }

#if !USE_YMPLAYER_SERIAL
  midiEventPacket_t rx;
  while ((rx = MidiUSB.read()).header) {
    handleMidiMsg(rx.byte1, rx.byte2, rx.byte3);
  }
#else
  player.update();
#endif

  static unsigned long last = millis();
  unsigned long m = millis();
  if (m - last >= 3) {
    last = m;
    for (uint8_t ch = 0; ch < 16; ++ch) {
      updatePitchMod(ch);
    }
  }
}

void noiseOn(uint8_t note, uint8_t vel) {
  uint8_t chip = 2;
  uint8_t nf = constrain((int)note - 24, 2, 31);
  psgWrite(chip, 6, nf);                 // noise period
  psgWrite(chip, 7, 0b00011100);         // enable noise C, disable tone A/B/C, noise A/B
  psgWrite(chip, 8 + 2, vel >> 3);       // volume on C
}

void noiseOff() {
  uint8_t chip = 2;
  enableTones(chip);                     // restore all tones
  psgWrite(chip, 8 + 2, 0);              // silence voice C
}
