// Ym2149Synth.ino — YM2149F MIDI + CC support + LED feedback + optional YMPlayerSerial

#include <Arduino.h>
#include <MIDIUSB.h>
#include <math.h>

// Toggle YM file streaming via USB CDC
#define USE_YMPLAYER_SERIAL 1

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
// Map MIDI channels 0–8 to chips 0–2
const uint8_t midiToChip[9] = {0,0,0, 1,1,1, 2,2,2};
const float   YM_CLOCK_HZ   = 500000.0f;

#if USE_YMPLAYER_SERIAL
YMPlayerSerial player;
#endif

// Synthesis state
float   modWheel[9]            = {0};
float   vibPhase[9]            = {0};
float   pitchEnvPhase[9]       = {0};    // 0…1 envelope phase
float   pitchEnvIncrement[9]   = {0};    // advance per 5ms tick
float   pitchEnvAmt[9]         = {0};    // max semitones
uint8_t pitchEnvShape[9]       = {0};    // CC10 raw value

const float vibRate            = 5.0f;
const float vibRangeSemi       = 1.0f;
float   pitchBendSemis[9]      = {0};
const float bendRangeSemi      = 2.0f;
uint8_t expressionVal[9]       = {127,127,127,127,127,127,127,127,127};
bool    portamentoOn[9]        = {false};
float   portamentoSpeed[9]     = {0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f};

bool    voiceActive[3][3]      = {{false}};
uint8_t voiceNote[3][3]        = {{0}};
uint8_t voiceChan[3][3]        = {{0}};
uint8_t voiceVol[3][3]         = {{0}};
uint8_t nextVoice[3]           = {0,0,0};
float   curPeriod[3][3]        = {{0}};

// LED flash timers
unsigned long ledOnTime[3]     = {0,0,0};
const unsigned long ledFlashMs = 100;

// MIDI parse state machine
enum SerState { WAIT_STATUS, WAIT_D1, WAIT_D2 };
static SerState serState       = WAIT_STATUS;
static uint8_t serStatus, serD1;

uint16_t noteToPeriod(uint8_t note) {
  float freq = 440.0f * powf(2.0f, ((int)note - 69) / 12.0f);
  return (uint16_t)(YM_CLOCK_HZ / (16.0f * freq) + 0.5f);
}

void busWrite(uint8_t val) {
  for (uint8_t i=0; i<8; ++i)
    digitalWrite(DATA_PINS[i], (val>>i)&1);
}
void selectYM(uint8_t chip) {
  chip = 2 - chip;
  digitalWrite(PIN_SEL_A, chip & 1);
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

void setVoice(uint8_t chip, uint8_t voice, uint16_t period, uint8_t volume) {
  psgWrite(chip, voice*2,       period & 0xFF);
  psgWrite(chip, voice*2 + 1, (period >> 8) & 0x0F);
  psgWrite(chip, 8 + voice,    volume & 0x0F);
}
void stopVoice(uint8_t chip, uint8_t voice) {
  setVoice(chip, voice, 0, 0);
}
void enableTones(uint8_t chip) {
  psgWrite(chip, 7, 0b00111000);
}

void noiseOn(uint8_t note, uint8_t vel) {
  uint8_t nf = constrain((int)note - 24, 2, 31);
  psgWrite(2, 6, nf);
  psgWrite(2, 7, 0b00011100);
  psgWrite(2, 8+2, vel >> 3);
}
void noiseOff() {
  enableTones(2);
  psgWrite(2, 8+2, 0);
}

void updatePitchMod(uint8_t channel) {
  uint8_t chip = midiToChip[channel];

  // Vibrato LFO
  vibPhase[channel] += vibRate * 0.005f;
  if (vibPhase[channel] >= 1.0f) vibPhase[channel] -= 1.0f;
  float lfo = sinf(vibPhase[channel]*2*PI)
            * (modWheel[channel]/127.0f)
            * vibRangeSemi;

  for (int v=0; v<3; ++v) {
    if (!voiceActive[chip][v] || voiceChan[chip][v] != channel) continue;

    float base = (float)noteToPeriod(voiceNote[chip][v]);
    float tp   = base / powf(2.0f, (pitchBendSemis[channel] + lfo)/12.0f);

    // Pitch Envelope
    if (pitchEnvAmt[channel] > 0) {
      // advance phase (cap at 1.0)
      pitchEnvPhase[channel] += pitchEnvIncrement[channel];
      if (pitchEnvPhase[channel] > 1.0f) pitchEnvPhase[channel] = 1.0f;

      // simple attack/release shape
      float envVal;
      if (pitchEnvShape[channel] < 64) {
        // attack ramp 0→1
        envVal = pitchEnvPhase[channel];
      } else {
        // release ramp 1→0
        envVal = 1.0f - pitchEnvPhase[channel];
      }

      // scale into semitones
      float semi = pitchEnvAmt[channel] * envVal;
      tp /= powf(2.0f, semi/12.0f);
    }

    // Portamento
    if (portamentoOn[channel]) {
      if (curPeriod[chip][v] == 0) curPeriod[chip][v] = tp;
      curPeriod[chip][v] += (tp - curPeriod[chip][v]) * portamentoSpeed[channel];
    } else {
      curPeriod[chip][v] = tp;
    }

    // Expression & set voice
    uint8_t expr = expressionVal[channel] ? expressionVal[channel] : 127;
    uint8_t vol  = (voiceVol[chip][v] * expr + 63) / 127;
    setVoice(chip, v, (uint16_t)(curPeriod[chip][v] + 0.5f), vol);
  }
}

void noteOn(uint8_t ch, uint8_t note, uint8_t vel) {
  if (ch == 9) { noiseOn(note, vel); return; }
  uint8_t chip = midiToChip[ch];
  int8_t v; for (v=0; v<3; ++v) if (!voiceActive[chip][v]) break;
  if (v >= 3) v = nextVoice[chip], nextVoice[chip] = (v+1)%3;
  voiceActive[chip][v] = true;
  voiceNote[chip][v]   = note;
  voiceChan[chip][v]   = ch;
  voiceVol[chip][v]    = vel >> 3;
  curPeriod[chip][v]   = 0;

  // reset pitch envelope on new note
  pitchEnvPhase[ch] = 0;

  updatePitchMod(ch);
  // flash LED
  digitalWrite(CHIP_LED[chip], LED_ON);
  ledOnTime[chip] = millis();
}
void noteOff(uint8_t ch, uint8_t note) {
  if (ch == 9) { noiseOff(); return; }
  uint8_t chip = midiToChip[ch];
  for (int v=0; v<3; ++v) {
    if (voiceActive[chip][v] && voiceNote[chip][v]==note) {
      voiceActive[chip][v] = false;
      stopVoice(chip, v);
      break;
    }
  }
}

void pitchBend(uint8_t ch, uint8_t lsb, uint8_t msb) {
  int val = (msb<<7) | lsb;
  pitchBendSemis[ch] = ((float)val - 8192)/8192 * bendRangeSemi;
  updatePitchMod(ch);
}

void handleMidiMsg(uint8_t status, uint8_t d1, uint8_t d2) {
  uint8_t cmd = status & 0xF0, ch = status & 0x0F;
  if (cmd == 0x90 && d2) noteOn(ch,d1,d2);
  else if (cmd==0x80 || (cmd==0x90&&d2==0)) noteOff(ch,d1);
  else if (cmd==0xB0 && ch<9) {
    switch (d1) {
      case 1:  modWheel[ch]=d2; updatePitchMod(ch); break;
      case 5:  portamentoSpeed[ch]=constrain(d2/127.0f,0.005f,0.5f); updatePitchMod(ch); break;
      case 7:  expressionVal[ch]=d2; updatePitchMod(ch); break;
      case 9:  // Pitch Envelope amount
        pitchEnvAmt[ch] = (d2 / 127.0f) * 2.0f;           // 0–2 semitones
        pitchEnvIncrement[ch] = 1.0f / (200.0f / 5.0f);   // envelope over ~200ms
        break;
      case 10: // Pitch Envelope shape
        pitchEnvShape[ch] = d2;
        break;
      case 11: expressionVal[ch]=d2; updatePitchMod(ch); break;
      case 65:
        portamentoOn[ch]=(d2>=64);
        // reset periods so glide restarts
        for (int v=0; v<3; ++v)
          if (voiceActive[midiToChip[ch]][v] && voiceChan[midiToChip[ch]][v]==ch)
            curPeriod[midiToChip[ch]][v]=0;
        updatePitchMod(ch);
        break;
      case 120: case 123:
        for (int v=0; v<3; ++v) {
          if (voiceActive[midiToChip[ch]][v] && voiceChan[midiToChip[ch]][v]==ch) {
            stopVoice(midiToChip[ch],v);
            voiceActive[midiToChip[ch]][v]=false;
          }
        }
        break;
    }
  } else if (cmd==0xE0) pitchBend(ch,d1,d2);
}

void parseSerialMidi(uint8_t b) {
  if (b & 0x80)      { serStatus=b; serState=WAIT_D1; }
  else if (serState==WAIT_D1) { serD1=b; serState=WAIT_D2; }
  else if (serState==WAIT_D2) { handleMidiMsg(serStatus,serD1,b); serState=WAIT_D1; }
}

void loop() {
  while (Serial1.available()) parseSerialMidi(Serial1.read());

  // Turn off LEDs after flash timeout
  unsigned long now = millis();
  for (uint8_t c=0; c<3; ++c) {
    if (ledOnTime[c] && (now - ledOnTime[c] >= ledFlashMs)) {
      digitalWrite(CHIP_LED[c], LED_OFF);
      ledOnTime[c] = 0;
    }
  }

#if !USE_YMPLAYER_SERIAL
  midiEventPacket_t rx;
  while ((rx = MidiUSB.read()).header)
    handleMidiMsg(rx.byte1, rx.byte2, rx.byte3);
#else
  player.update();
#endif

  // Periodic pitch modulation & envelope updates (~ every 5ms)
  static unsigned long last = millis();
  unsigned long m = millis();
  if (m - last >= 5) {
    last = m;
    for (uint8_t ch=0; ch<9; ++ch)
      updatePitchMod(ch);
  }
}

void setup() {
  // Initialize PSG bus pins
  for (auto p: DATA_PINS) pinMode(p, OUTPUT);
  pinMode(PIN_BC1, OUTPUT);
  pinMode(PIN_BDIR, OUTPUT);
  pinMode(PIN_SEL_A, OUTPUT);
  pinMode(PIN_SEL_B, OUTPUT);
  pinMode(PIN_SEL_C, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);

  // Initialize activity LEDs
  for (uint8_t i=0; i<3; ++i) {
    pinMode(CHIP_LED[i], OUTPUT);
    digitalWrite(CHIP_LED[i], LED_OFF);
  }
  // Startup blink
  for (uint8_t i=0; i<3; ++i) {
    digitalWrite(CHIP_LED[i], LED_ON);
    delay(200);
    digitalWrite(CHIP_LED[i], LED_OFF);
    delay(100);
  }

  // Reset all voices
  for (uint8_t c=0; c<3; ++c) {
    enableTones(c);
    for (uint8_t v=0; v<3; ++v) stopVoice(c,v);
  }

  // MIDI input
  pinMode(0, INPUT_PULLUP);
  Serial1.begin(31250);

#if USE_YMPLAYER_SERIAL
  Serial.begin(115200);
  player.begin();
#endif
}
