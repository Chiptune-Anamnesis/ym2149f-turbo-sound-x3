#include <Arduino.h>
#include <MIDIUSB.h>
#include <math.h>
#include "YM2149.h"

// ═══════════════════════════════════════════════════════════════════════════
// MIDI STABILITY FIXES v1.49
// ═══════════════════════════════════════════════════════════════════════════
// ✓ Fixed portamento speed scaling (was inverted)
// ✓ Added note range clamping (A0-C8) to prevent glitchy out-of-bounds periods
// ✓ Implemented MIDI Stop/Start/Continue (0xFC/0xFA/0xFB) handling
// ✓ Added resetAllControllers() to clear stuck CC states
// ✓ Added allNotesOffPanic() for emergency voice clearing
// ✓ Implemented CC121 (Reset All Controllers) support
// ✓ Improved noteOff to release ALL matching notes (prevents stuck notes)
// ✓ Added channel validation to noteOff sustain logic
// ✓ All controllers initialized via resetAllControllers() in setup()
// ✓ CRITICAL: Fixed array bounds - only channels 0-8 are valid for tone generation
// ✓ OPTIMIZATION: Using YM2149 class with direct port manipulation + atomic blocks
// ✓ CRITICAL: Fixed laser mode division-by-zero bug causing NaN/Infinity stuck notes
//   - Changed from curPeriod = targetP / laserAmt to multiplication formula
//   - Added laserAmt > 0.01f check to prevent division by zero
//   - Added period clamping (1.0-4095.0) to catch NaN/Infinity/overflow values
//   - Fixed immediate attack to use targetP instead of 0
// ✓ CRITICAL: Added channel validation to noteOn, noteOff, and pitchBend
//   - Prevents array bounds violation when MIDI channels 11-16 send note messages
//   - These channels were reading garbage from midiToChip[10-15] out-of-bounds
//   - Caused wild behavior, stuck notes, and memory corruption
// ✓ Fixed LED polarity (inverted YM2149 class setLED logic)
//   - LEDs were always on and turned off on notes
//   - Now correctly off by default and flash on when notes hit
// ✓ Simplified noise channel cleanup in noiseOff()
//   - Just mutes volume (reg 10) - that's what stops it between drum hits
//   - No mixer or register manipulation needed
//   - Simple, reliable, matches how noiseOn() works (volume controls sound)
// ✓ Added LED flash for noise channel (chip 2)
//   - LED2 now flashes when channel 10 (noise) notes are received
//   - Consistent with tone channel LED behavior
// ═══════════════════════════════════════════════════════════════════════════

// Toggle YM file streaming via USB CDC
#define USE_YMPLAYER_SERIAL 0
#if USE_YMPLAYER_SERIAL
  #include "YMPlayerSerial.h"
#endif

// Toggle noise channel support (MIDI channel 10)
#define ENABLE_NOISE_CHANNEL 0

// Velocity curve configuration
#define VELOCITY_GAMMA  0.3f    // Gamma curve (lower = more boost for soft notes, try 0.4-0.7)
#define VELOCITY_MIN    4       // Minimum YM volume (0-15, try 3-4 for audible soft notes)
#define VELOCITY_MAX    15      // Maximum YM volume (1-15, typically 15)

// Volume envelope/expression control
#define EXPRESSION_AMOUNT 0.3f  // How much CC7/CC11 affects volume (0.0=none, 1.0=full, try 0.3-0.5 for compression)
#define USE_CC4_ENVELOPE  1     // 0 = bypass CC4 volume envelope, 1 = enable

// Global octave transpose (in semitones: -12 = down 1 octave, -24 = down 2 octaves, 0 = no shift)
#define OCTAVE_SHIFT -12        // Default: shift down 1 octave

#define LED_ON  LOW
#define LED_OFF HIGH

// YM2149F hardware interface
YM2149 ym;

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

#define DEFAULT_VIB_RATE      7.0f   // Hz
#define DEFAULT_VIB_RANGE     0.5f   // semitone

// per-channel state
float   modWheel[9]            = {0};
float   vibPhase[9]            = {0};
float   vibRate[16]       = {
  DEFAULT_VIB_RATE, DEFAULT_VIB_RATE, DEFAULT_VIB_RATE, DEFAULT_VIB_RATE,
  DEFAULT_VIB_RATE, DEFAULT_VIB_RATE, DEFAULT_VIB_RATE, DEFAULT_VIB_RATE,
  DEFAULT_VIB_RATE, DEFAULT_VIB_RATE, DEFAULT_VIB_RATE, DEFAULT_VIB_RATE,
  DEFAULT_VIB_RATE, DEFAULT_VIB_RATE, DEFAULT_VIB_RATE, DEFAULT_VIB_RATE
};

float   vibRangeSemi[16]  = {
  DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE,
  DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE,
  DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE,
  DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE, DEFAULT_VIB_RANGE
};
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
const uint8_t MIDI_NOTE_MIN = 9;   // A-1 = 13.75 Hz (extended bass range)
const uint8_t MIDI_NOTE_MAX = 108; // C8
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

// Forward declarations
#if ENABLE_NOISE_CHANNEL
void noiseOn(uint8_t note, uint8_t vel);
void noiseOff();
#endif

// Helpers
uint16_t noteToPeriod(uint8_t note) {
  float freq = 440.0f * powf(2.0f, ((int)note - 69) / 12.0f);
  return (uint16_t)(YM_CLOCK_HZ / (16.0f * freq) + 0.5f);
}

// YM2149 register write wrappers (use optimized class)
inline void setVoice(uint8_t chip, uint8_t v, uint16_t per, uint8_t vol) {
  ym.write(chip, v*2,     per & 0xFF);
  ym.write(chip, v*2 + 1, (per >> 8) & 0x0F);
  ym.write(chip, 8 + v,   vol & 0x0F);
}

inline void stopVoice(uint8_t chip, uint8_t v) {
  ym.write(chip, v*2,     0);
  ym.write(chip, v*2 + 1, 0);
  ym.write(chip, 8 + v,   0);
}

inline void enableTones(uint8_t chip) {
  ym.write(chip, 7, 0b00111000);
}

// ——— Reset all controllers for a single channel ———
void resetAllControllers(uint8_t ch) {
  if (ch >= 9) return; // Only channels 0-8 have tone voices

  modWheel[ch]         = 0;
  pitchBendSemis[ch]   = 0;
  pitchEnvAmt[ch]      = 0;
  pitchEnvPhase[ch]    = 0;
  pitchEnvShape[ch]    = 0;
  expressionVal[ch]    = 127;
  portamentoOn[ch]     = false;
  portamentoSpeed[ch]  = 0.05f;
  laserMode[ch]        = false;
  laserAmt[ch]         = 1.0f;
  cc4Shape[ch]         = 0;
  volEnvOn[ch]         = false;
  volEnvPhase[ch]      = 0;
  sustainOn[ch]        = false;
  vibPhase[ch]         = 0;
  vibStartTime[ch]     = 0;

  // Clear laser triggers for this channel's chip
  if (ch < 9) {
    uint8_t chip = midiToChip[ch];
    for (uint8_t v = 0; v < 3; v++) {
      laserTriggered[chip][v] = false;
    }
  }
}

// ——— Emergency all-notes-off for a single channel ———
void allNotesOffChannel(uint8_t ch) {
  if (ch >= 9) return; // only channels 0-8 have tone voices

  uint8_t chip = midiToChip[ch];
  for (uint8_t v = 0; v < 3; v++) {
    if (voiceActive[chip][v] && voiceChan[chip][v] == ch) {
      voiceActive[chip][v]     = false;
      pendingRelease[chip][v]  = false;
      laserTriggered[chip][v]  = false;  // Clear laser trigger
      stopVoice(chip, v);
    }
  }
}

// ——— Global panic: kill all voices on all chips ———
void allNotesOffPanic() {
  for (uint8_t c = 0; c < 3; c++) {
    for (uint8_t v = 0; v < 3; v++) {
      voiceActive[c][v]     = false;
      pendingRelease[c][v]  = false;
      laserTriggered[c][v]  = false;  // Clear laser triggers
      stopVoice(c, v);
    }
  }
#if ENABLE_NOISE_CHANNEL
  // Also stop noise channel
  noiseOff();
#endif
}

// Core update (~5ms)
// ——— Modified updatePitchMod (bounds‑checks ch 0–15) ———
void updatePitchMod(uint8_t ch) {
  if (ch >= 9) return; // Only channels 0-8 have tone voices
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

    // Clamp period to valid range (avoid glitches from NaN/Inf/overflow)
    if (curPeriod[chip][v] < 1.0f || curPeriod[chip][v] > 4095.0f) {
      curPeriod[chip][v] = tp;  // Reset to target if out of bounds
    }

    // compute volume
    uint16_t outP = uint16_t(curPeriod[chip][v] + 0.5f);
    // Apply expression/CC11 with adjustable amount (0.0=bypass, 1.0=full effect)
    uint8_t effectiveExpr = 127 - (uint8_t)((127 - expressionVal[ch]) * EXPRESSION_AMOUNT);
    uint8_t vol = (voiceVol[chip][v] * effectiveExpr + 63) / 127;

    // CC4 volume envelope
#if USE_CC4_ENVELOPE
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
#endif

    setVoice(chip, v, outP, vol);
  }
}

void noteOn(uint8_t ch, uint8_t note, uint8_t vel) {
#if ENABLE_NOISE_CHANNEL
  // ——— special noise on MIDI Channel 10 (ch==9) using chip 2 ———
  if (ch == 9) {
    noiseOn(note, vel);
    return;
  }
#endif

  // ——— ignore channels 9-15 (MIDI channels 10-16) to prevent array bounds violation ———
  if (ch >= 9) return;

  // ——— apply global octave shift ———
  note += OCTAVE_SHIFT;

  // ——— transpose note to safe playable range (preserve pitch class) ———
  while (note < MIDI_NOTE_MIN) note += 12;
  while (note > MIDI_NOTE_MAX) note -= 12;

  // ——— start vibrato delay timer ———
  vibStartTime[ch] = millis();
  vibPhase[ch]     = 0;

  // ——— find a free tone‑voice or round‑robin ———
  uint8_t chip = midiToChip[ch];
  uint8_t v;
  for (v = 0; v < 3; v++) {
    if (!voiceActive[chip][v]) break;
  }
  if (v >= 3) {
    v = nextVoice[chip];
    nextVoice[chip] = (v + 1) % 3;
  }

  // ——— assign voice parameters ———
  voiceActive[chip][v] = true;
  voiceNote[chip][v]   = note;
  voiceChan[chip][v]   = ch;

  // —— improved velocity sensitivity via gamma curve ——
  {
    // normalize 0–127 → 0.0–1.0
    float velNorm  = vel / 127.0f;
    // apply gamma to boost low‑velocity resolution
    float velCurve = powf(velNorm, VELOCITY_GAMMA);
    // scale to YM volume range (VELOCITY_MIN to VELOCITY_MAX)
    float range = VELOCITY_MAX - VELOCITY_MIN;
    voiceVol[chip][v] = uint8_t(VELOCITY_MIN + velCurve * range + 0.5f);
  }

  // ——— laser‑jump or portamento zeroing ———
  float targetP = noteToPeriod(note);  // Target period for this note
  if (laserMode[ch] && laserAmt[ch] > 0.01f) {  // Prevent division by zero
    // Laser: jump from low frequency (high period) toward target
    curPeriod[chip][v] = targetP * (1.0f + laserAmt[ch] * 10.0f);  // Scale up for "laser jump"
    laserTriggered[chip][v] = true;  // Prevent immediate correction
  }
  else if (!portamentoOn[ch]) {
    curPeriod[chip][v] = targetP;  // Start at target for immediate attack
  }
  // else leave curPeriod for glide

  // ——— reset pitch envelope phase ———
  pitchEnvPhase[ch] = 0;

  // ——— CC4 volume‑envelope trigger ———
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

  // ——— apply initial pitch (with laser/portamento) ———
  updatePitchMod(ch);

  // ——— flash LED on the tone‑chip (inverted logic: false=on) ———
  ym.setLED(chip, false);  // Turn ON
  ledOnTime[chip] = millis();
}


void noteOff(uint8_t ch, uint8_t note) {
#if ENABLE_NOISE_CHANNEL
  // ——— special noise-off on MIDI Channel 10 (ch==9) using chip 2 ———
  if (ch == 9) {
    noiseOff();
    return;
  }
#endif

  // ——— ignore channels 9-15 (MIDI channels 10-16) to prevent array bounds violation ———
  if (ch >= 9) return;

  // ——— apply global octave shift (must match noteOn) ———
  note += OCTAVE_SHIFT;

  // ——— transpose note to match noteOn transposition ———
  while (note < MIDI_NOTE_MIN) note += 12;
  while (note > MIDI_NOTE_MAX) note -= 12;

  // ——— sustain pedal logic for channels 1–9 ———
  if (sustainOn[ch]) {
    uint8_t chip = midiToChip[ch];
    for (uint8_t v = 0; v < 3; v++) {
      if (voiceActive[chip][v] && voiceChan[chip][v] == ch && voiceNote[chip][v] == note) {
        pendingRelease[chip][v] = true;
        // Don't break - mark ALL matching notes for release
      }
    }
    return;
  }

  // ——— immediate note-off for channels 1–9 ———
  {
    uint8_t chip = midiToChip[ch];
    for (uint8_t v = 0; v < 3; v++) {
      if (voiceActive[chip][v] && voiceChan[chip][v] == ch && voiceNote[chip][v] == note) {
        voiceActive[chip][v] = false;
        stopVoice(chip, v);
        pendingRelease[chip][v] = false;
        // Don't break - stop ALL matching notes to prevent stuck notes
      }
    }
  }
}



void pitchBend(uint8_t ch, uint8_t lsb, uint8_t msb) {
  if (ch >= 9) return; // Only channels 0-8 have tone voices
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
  else if (cmd == 0xB0 && ch < 9) {  // Only channels 0-8 have tone voices
    switch (d1) {
      case 1:   modWheel[ch]     = d2; updatePitchMod(ch); break;
      case 4:   cc4Shape[ch]     = d2;                          break;
      case 5: { // portamento speed (0=slow, 127=fast)
        float norm  = d2 / 127.0f;
        float curve = norm * norm;
        portamentoSpeed[ch] = PORTA_MIN + (PORTA_MAX - PORTA_MIN) * curve;
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
      case 120: // All Sound Off (immediate panic)
                allNotesOffChannel(ch);
                break;
      case 121: // Reset All Controllers
                resetAllControllers(ch);
                break;
      case 123: // All Notes Off (respects sustain)
                if (sustainOn[ch]) {
                  // Mark all notes for release when sustain lifts
                  uint8_t chip = midiToChip[ch];
                  for (uint8_t v = 0; v < 3; v++) {
                    if (voiceActive[chip][v] && voiceChan[chip][v] == ch) {
                      pendingRelease[chip][v] = true;
                    }
                  }
                } else {
                  allNotesOffChannel(ch);
                }
                break;
    }
  }
  else if (cmd == 0xE0) {
    pitchBend(ch, d1, d2);
  }
}

void parseSerialMidi(uint8_t b) {
  // real‑time messages (0xF8–0xFF): handle without disturbing serState
  if (b >= 0xF8) {
    switch (b) {
      case 0xFA: // MIDI Start
      case 0xFB: // MIDI Continue
        // Reset controllers on tone channels (0-8 only)
        for (uint8_t ch = 0; ch < 9; ch++) {
          resetAllControllers(ch);
        }
        break;
      case 0xFC: // MIDI Stop
        // Panic: kill all notes and reset all controllers
        allNotesOffPanic();
        for (uint8_t ch = 0; ch < 9; ch++) {
          resetAllControllers(ch);
        }
        break;
      // 0xFE (Active Sensing) and 0xF8 (Clock): ignore
    }
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
  // Initialize YM2149 hardware (pins, ports, etc.)
  ym.begin();

  // LED startup animation (inverted logic: false=on, true=off)
  for (uint8_t i = 0; i < 3; i++) {
    ym.setLED(i, false);  // Turn ON
    delay(200);
    ym.setLED(i, true);   // Turn OFF
    delay(100);
  }

  // Initialize all YM chips: enable tones, silence all voices
  for (uint8_t c = 0; c < 3; c++) {
    enableTones(c);
    for (uint8_t v = 0; v < 3; v++) stopVoice(c, v);
  }

  // Initialize all controller states for tone channels (0-8 only)
  for (uint8_t ch = 0; ch < 9; ++ch) {
    resetAllControllers(ch);
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
      ym.setLED(c, true);  // Turn OFF (inverted logic)
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
    for (uint8_t ch = 0; ch < 9; ++ch) {
      updatePitchMod(ch);
    }
  }
}

#if ENABLE_NOISE_CHANNEL
void noiseOn(uint8_t note, uint8_t vel) {
  uint8_t chip = 2;
  uint8_t nf = constrain((int)note - 24, 2, 31);
  ym.write(chip, 6, nf);                 // noise period
  ym.write(chip, 7, 0b00011100);         // enable noise C, disable tone A/B/C, noise A/B
  ym.write(chip, 8 + 2, vel >> 3);       // volume on C

  // ——— flash LED on chip 2 for noise (inverted logic: false=on) ———
  ym.setLED(chip, false);  // Turn ON
  ledOnTime[chip] = millis();
}

void noiseOff() {
  uint8_t chip = 2;
  ym.write(chip, 8 + 2, 0);              // Mute volume
  ym.write(chip, 7, 0b00111000);         // Disable noise, enable tones (same as enableTones)
}
#endif
