// YM2149F MIDI + LED inverted logic + Vibrato + Noise Drum on Channel 10
// Arduino Pro Micro + 3 YM2149F chips + 74HC138 addressing

#include <Arduino.h>
#include <MIDIUSB.h>
#include <math.h>

// === LED logic inversion ===
#define LED_ON  LOW
#define LED_OFF HIGH

// === hardware pin assignments ===
const uint8_t DATA_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const uint8_t PIN_BC1      = 10;
const uint8_t PIN_BDIR     = 20;
const uint8_t PIN_SEL_A    = A3;
const uint8_t PIN_SEL_B    = A1;
const uint8_t PIN_SEL_C    = A0;
const uint8_t PIN_ENABLE   = A2; // drive high for 74LS08 input

// LED pins for each chip (inverted: LOW=on, HIGH=off)
const uint8_t CHIP_LED[3]  = {16, 14, 15};

// map MIDI channel (0–8) → chip number (0–2)
const uint8_t midiToChip[9] = {0,0,0,  1,1,1,  2,2,2};

// YM2149 clock rate (after prescalers, e.g. 8 MHz/16 → 500 kHz)
const float YM_CLOCK_HZ = 500000.0f;

// Vibrato parameters
float   modWheel[9]        = {0};      // CC1 values 0–127
float   vibPhase[9]        = {0};      // LFO phase (0–1)
const float vibRate        = 5.0f;     // LFO frequency in Hz
const float vibRangeSemi   = 1.0f;     // max ±1 semitone

// Pitch bend state
float   pitchBendSemis[9]  = {0};      // per-channel bend
const float bendRangeSemi  = 2.0f;     // ±2 semitones

// dynamic voice state
bool    voiceActive[3][3]  = {{false}};
uint8_t voiceNote[3][3]    = {{0}};
uint8_t voiceChan[3][3]    = {{0}};
uint8_t voiceVol[3][3]     = {{0}};
uint8_t nextVoice[3]       = {0,0,0};

// Convert MIDI note → YM period
uint16_t noteToPeriod(uint8_t note) {
  float freq = 440.0f * powf(2.0f, ((int)note - 69) / 12.0f);
  return (uint16_t)(YM_CLOCK_HZ / (16.0f * freq) + 0.5f);
}

// low-level bus writes
void busWrite(uint8_t val) {
  for (uint8_t i = 0; i < 8; ++i)
    digitalWrite(DATA_PINS[i], (val >> i) & 1);
}
void selectYM(uint8_t chip) {
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

// set tone & volume for a single voice
void setVoice(uint8_t chip, uint8_t voice, uint16_t period, uint8_t volume) {
  psgWrite(chip, voice*2,        period & 0xFF);
  psgWrite(chip, voice*2 + 1,  (period >> 8) & 0x0F);
  psgWrite(chip, 8+voice,       volume & 0x0F);
}
// silence a voice
void stopVoice(uint8_t chip, uint8_t voice) {
  setVoice(chip, voice, 0, 0);
}
// enable tone channels, disable noise
void enableTones(uint8_t chip) {
  psgWrite(chip, 7, 0b00111000);
}

// trigger noise on chip 1 (index=1)
void noiseOn(uint8_t note, uint8_t vel) {
  uint8_t chip = 0;
  // map note to noise frequency (0–31)
  uint8_t noiseFreq = constrain(note > 24 ? note - 24 : 0, 0, 31);
  psgWrite(chip, 6, noiseFreq);
  // mixer: disable all tones, enable noise C
  psgWrite(chip, 7, 0x1F);
  // amplitude on channel C
  psgWrite(chip, 8+2, vel >> 3);
}
void noiseOff() {
  uint8_t chip = 0;            // correct physical chip index
  enableTones(chip);           // writes 0b00111000 → enable A/B/C, disable noise
  psgWrite(chip, 8+2, 0);      // clear channel-C volume
}

// apply combined pitch bend and vibrato to active voices on channel
void updatePitchMod(uint8_t channel) {
  uint8_t chip = midiToChip[channel];
  float dt = 0.005f;
  vibPhase[channel] += vibRate * dt;
  if (vibPhase[channel] >= 1.0f) vibPhase[channel] -= 1.0f;
  float lfo = sinf(vibPhase[channel] * 2.0f * PI);
  float vibSem = (modWheel[channel]/127.0f) * vibRangeSemi * lfo;
  for (int v = 0; v < 3; ++v) {
    if (!voiceActive[chip][v] || voiceChan[chip][v] != channel) continue;
    uint16_t baseP = noteToPeriod(voiceNote[chip][v]);
    float totalSem = pitchBendSemis[channel] + vibSem;
    float factor = powf(2.0f, totalSem/12.0f);
    uint16_t modP = (uint16_t)(baseP / factor + 0.5f);
    setVoice(chip, v, modP, voiceVol[chip][v]);
  }
}

// handle Note-On
void noteOn(uint8_t channel, uint8_t note, uint8_t vel) {
  if (channel == 9) { noiseOn(note, vel); return; }
  uint8_t chip   = midiToChip[channel];
  uint8_t volume = vel >> 3;
  int8_t v;
  for (v=0; v<3; ++v) if (!voiceActive[chip][v]) break;
  if (v >= 3) v = nextVoice[chip];
  nextVoice[chip] = (v+1) % 3;
  voiceActive[chip][v] = true;
  voiceNote[chip][v]   = note;
  voiceChan[chip][v]   = channel;
  voiceVol[chip][v]    = volume;
  updatePitchMod(channel);
  digitalWrite(CHIP_LED[chip], LED_ON);
}
// handle Note-Off
void noteOff(uint8_t channel, uint8_t note) {
  if (channel == 9) { noiseOff(); return; }
  uint8_t chip = midiToChip[channel];
  for (int v=0; v<3; ++v) {
    if (voiceActive[chip][v] && voiceNote[chip][v] == note) {
      voiceActive[chip][v] = false;
      stopVoice(chip, v);
      break;
    }
  }
  digitalWrite(CHIP_LED[chip], LED_OFF);
}
// handle Pitch Bend
void pitchBend(uint8_t channel, uint8_t lsb, uint8_t msb) {
  int val = (msb<<7) | lsb;
  float norm = ((float)val - 8192.0f)/8192.0f;
  pitchBendSemis[channel] = norm * bendRangeSemi;
  updatePitchMod(channel);
}

// parse MIDI messages
enum SerState { S_WAIT_STATUS, S_WAIT_D1, S_WAIT_D2 };
static SerState serState = S_WAIT_STATUS;
static uint8_t  serStatus, serD1;
void handleMidiMsg(uint8_t status, uint8_t d1, uint8_t d2) {
  uint8_t cmd = status & 0xF0;
  uint8_t ch  = status & 0x0F;
  if (cmd == 0x90 && d2>0)      noteOn(ch,d1,d2);
  else if(cmd==0x80 || (cmd==0x90 && d2==0)) noteOff(ch,d1);
  else if(cmd==0xB0 && d1==1)   modWheel[ch] = d2;
  else if(cmd==0xE0)            pitchBend(ch,d1,d2);
}
void parseSerialMidi(uint8_t b) {
  if(b & 0x80){ serStatus=b; serState=S_WAIT_D1; }
  else if(serState==S_WAIT_D1){ serD1=b; serState=S_WAIT_D2; }
  else if(serState==S_WAIT_D2){ handleMidiMsg(serStatus,serD1,b); serState=S_WAIT_D1; }
}

void setup() {
  for(auto p:DATA_PINS) pinMode(p,OUTPUT);
  pinMode(PIN_BC1,OUTPUT); pinMode(PIN_BDIR,OUTPUT);
  pinMode(PIN_SEL_A,OUTPUT); pinMode(PIN_SEL_B,OUTPUT); pinMode(PIN_SEL_C,OUTPUT);
  pinMode(PIN_ENABLE,OUTPUT); digitalWrite(PIN_ENABLE,HIGH);
  for(uint8_t i=0;i<3;i++){ pinMode(CHIP_LED[i],OUTPUT); digitalWrite(CHIP_LED[i],LED_OFF); }
  for(uint8_t i=0;i<3;i++){ digitalWrite(CHIP_LED[i],LED_ON); delay(200); digitalWrite(CHIP_LED[i],LED_OFF); delay(100); }
  for(uint8_t c=0;c<3;c++){ enableTones(c); for(uint8_t v=0;v<3;v++) stopVoice(c,v); }
  Serial1.begin(31250);
}

void loop() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if(now - last >= 5){ last = now; for(uint8_t ch=0;ch<9;ch++) if(modWheel[ch]>0) updatePitchMod(ch); }
  midiEventPacket_t rx;
  while((rx=MidiUSB.read()).header) handleMidiMsg(rx.byte1,rx.byte2,rx.byte3);
  while(Serial1.available()) parseSerialMidi(Serial1.read());
}

