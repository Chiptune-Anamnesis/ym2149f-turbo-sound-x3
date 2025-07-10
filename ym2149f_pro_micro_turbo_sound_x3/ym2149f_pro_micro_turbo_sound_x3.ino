// Ym2149Synth.ino — YM2149F MIDI + CC support + CC4 Volume Envelope Shape (fixed ramp-down) + LED feedback

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
float   modWheel[9]          = {0};
float   vibPhase[9]          = {0};
float   vibRate              = 5.0f;
float   vibRangeSemi         = 1.0f;
float   pitchBendSemis[9]    = {0};
float   pitchEnvPhase[9]     = {0};
float   pitchEnvIncrement[9] = {0};
float   pitchEnvAmt[9]       = {0};
uint8_t pitchEnvShape[9]     = {0};
uint8_t expressionVal[9]     = {127,127,127,127,127,127,127,127,127};
bool    portamentoOn[9]      = {false};
float   portamentoSpeed[9]   = {0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f,0.05f};

// CC4 volume envelope
uint8_t cc4Shape[9]        = {0};
bool    volEnvOn[9]        = {false};
bool    volEnvDir[9]       = {true};   // true=ramp up, false=ramp down
float   volEnvPhase[9]     = {0.0f};
float   volEnvIncrement[9] = {0.0f};

// per-voice state
bool    voiceActive[3][3] = {{false}};
uint8_t voiceNote[3][3]   = {{0}};
uint8_t voiceChan[3][3]   = {{0}};
uint8_t voiceVol[3][3]    = {{0}};
uint8_t nextVoice[3]      = {0};
float   curPeriod[3][3]   = {{0}};

// LED flash timers
unsigned long ledOnTime[3]     = {0};
const unsigned long ledFlashMs = 100;

// MIDI parser
enum SerState { WAIT_STATUS, WAIT_D1, WAIT_D2 };
static SerState serState    = WAIT_STATUS;
static uint8_t serStatus, serD1;

// Helpers
uint16_t noteToPeriod(uint8_t note) {
  float freq = 440.0f * powf(2.0f, ((int)note - 69) / 12.0f);
  return (uint16_t)(YM_CLOCK_HZ / (16.0f * freq) + 0.5f);
}
void busWrite(uint8_t val) { for(uint8_t i=0;i<8;i++) digitalWrite(DATA_PINS[i], (val>>i)&1); }
void selectYM(uint8_t chip) {
  chip = 2 - chip;
  digitalWrite(PIN_SEL_A, (chip>>0)&1);
  digitalWrite(PIN_SEL_B, (chip>>1)&1);
  digitalWrite(PIN_SEL_C, (chip>>2)&1);
}
void psgWrite(uint8_t chip, uint8_t reg, uint8_t val) {
  selectYM(chip);
  busWrite(reg & 0x1F);
  digitalWrite(PIN_BC1, HIGH); digitalWrite(PIN_BDIR, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_BDIR, LOW); digitalWrite(PIN_BC1, LOW);
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
void stopVoice(uint8_t chip, uint8_t v) { setVoice(chip,v,0,0); }
void enableTones(uint8_t chip) { psgWrite(chip,7,0b00111000); }

// Core update (~5ms)
void updatePitchMod(uint8_t ch) {
  uint8_t chip = midiToChip[ch];
  // vibrato
  vibPhase[ch] += vibRate * 0.005f;
  if(vibPhase[ch]>=1.0f) vibPhase[ch]-=1.0f;
  float lfo = sinf(vibPhase[ch]*2*PI)*(modWheel[ch]/127.0f)*vibRangeSemi;

  for(int v=0;v<3;v++){
    if(!voiceActive[chip][v]||voiceChan[chip][v]!=ch) continue;
    float base = noteToPeriod(voiceNote[chip][v]);
    float tp   = base / powf(2.0f,(pitchBendSemis[ch]+lfo)/12.0f);

    // pitch env...
    if(pitchEnvAmt[ch]>0){
      pitchEnvPhase[ch]+=pitchEnvIncrement[ch];
      if(pitchEnvPhase[ch]>1.0f) pitchEnvPhase[ch]=1.0f;
      float ev = (pitchEnvShape[ch]<64)? pitchEnvPhase[ch]: 1.0f-pitchEnvPhase[ch];
      tp /= powf(2.0f,(pitchEnvAmt[ch]*ev)/12.0f);
    }
    // portamento...
    if(portamentoOn[ch]){
      if(curPeriod[chip][v]==0) curPeriod[chip][v]=tp;
      curPeriod[chip][v]+=(tp-curPeriod[chip][v])*portamentoSpeed[ch];
    } else curPeriod[chip][v]=tp;

    uint16_t outP = (uint16_t)(curPeriod[chip][v]+0.5f);
    uint8_t vol   = (voiceVol[chip][v]*expressionVal[ch]+63)/127;

    // CC4 volume envelope with correct ramp-down
    if(volEnvOn[ch]){
      if(volEnvDir[ch]){
        // ramp up
        volEnvPhase[ch] += volEnvIncrement[ch];
        if(volEnvPhase[ch]>=1.0f){
          volEnvPhase[ch]=1.0f;
          volEnvOn[ch]=false;
        }
      } else {
        // ramp down
        volEnvPhase[ch] -= volEnvIncrement[ch];
        if(volEnvPhase[ch]<=0.0f){
          volEnvPhase[ch]=0.0f;
          volEnvOn[ch]=false;
        }
      }
      vol = (uint8_t)(vol * volEnvPhase[ch] + 0.5f);
    }

    setVoice(chip,v,outP,vol);
  }
}

void noteOn(uint8_t ch,uint8_t note,uint8_t vel){
  if(ch==9) return; // skip noise
  uint8_t chip=midiToChip[ch],v;
  for(v=0;v<3;v++) if(!voiceActive[chip][v]) break;
  if(v>=3){ v=nextVoice[chip]; nextVoice[chip]=(v+1)%3; }
  voiceActive[chip][v]=true;
  voiceNote[chip][v]=note;
  voiceChan[chip][v]=ch;
  voiceVol[chip][v]=vel>>3;
  curPeriod[chip][v]=0;
  pitchEnvPhase[ch]=0;

  // re-trigger CC4 envelope
  if(cc4Shape[ch]==0){
    volEnvOn[ch]=false;
  } else if(cc4Shape[ch]<64){
    volEnvOn[ch]=true;
    volEnvDir[ch]=true;
    volEnvPhase[ch]=0.0f;
    uint16_t t=map(cc4Shape[ch],1,63,20,200);
    volEnvIncrement[ch]=1.0f/t;
  } else {
    volEnvOn[ch]=true;
    volEnvDir[ch]=false;
    volEnvPhase[ch]=1.0f;
    uint16_t t=map(cc4Shape[ch],64,127,20,200);
    volEnvIncrement[ch]=1.0f/t;
  }

  updatePitchMod(ch);
  digitalWrite(CHIP_LED[chip],LED_ON);
  ledOnTime[chip]=millis();
}

void noteOff(uint8_t ch,uint8_t note){
  if(ch==9) return;
  uint8_t chip=midiToChip[ch];
  for(int v=0;v<3;v++){
    if(voiceActive[chip][v]&&voiceNote[chip][v]==note){
      voiceActive[chip][v]=false;
      stopVoice(chip,v);
      break;
    }
  }
}

void pitchBend(uint8_t ch,uint8_t lsb,uint8_t msb){
  int val=(msb<<7)|lsb;
  pitchBendSemis[ch]=((float)val-8192)/8192*2.0f;
  updatePitchMod(ch);
}

void handleMidiMsg(uint8_t status,uint8_t d1,uint8_t d2){
  uint8_t cmd=status&0xF0,ch=status&0x0F;
  if(cmd==0x90&&d2) noteOn(ch,d1,d2);
  else if(cmd==0x80||(cmd==0x90&&d2==0)) noteOff(ch,d1);
  else if(cmd==0xB0&&ch<9){
    switch(d1){
      case 1: modWheel[ch]=d2; updatePitchMod(ch); break;
      case 4:
        cc4Shape[ch]=d2;
        break;
      case 5: portamentoSpeed[ch]=constrain(d2/127.0f,0.005f,0.5f); updatePitchMod(ch); break;
      case 7: expressionVal[ch]=d2; updatePitchMod(ch); break;
      case 9:
        pitchEnvAmt[ch]=(d2/127.0f)*2.0f;
        pitchEnvIncrement[ch]=1.0f/(200.0f/5.0f);
        break;
      case 10: pitchEnvShape[ch]=d2; break;
      case 11: expressionVal[ch]=d2; updatePitchMod(ch); break;
      case 65:
        portamentoOn[ch]=(d2>=64);
        for(int v=0;v<3;v++) if(voiceActive[midiToChip[ch]][v]&&voiceChan[midiToChip[ch]][v]==ch)
          curPeriod[midiToChip[ch]][v]=0;
        updatePitchMod(ch);
        break;
      case 76: vibRate=(d2/127.0f)*10.0f; break;
      case 77: vibRangeSemi=(d2/127.0f)*2.0f; break;
      case 120: case 123:
        for(int v=0;v<3;v++) if(voiceActive[midiToChip[ch]][v]&&voiceChan[midiToChip[ch]][v]==ch){
          stopVoice(midiToChip[ch],v);
          voiceActive[midiToChip[ch]][v]=false;
        }
        break;
    }
  } else if(cmd==0xE0) pitchBend(ch,d1,d2);
}

void parseSerialMidi(uint8_t b){
  if(b&0x80){ serStatus=b; serState=WAIT_D1; }
  else if(serState==WAIT_D1){ serD1=b; serState=WAIT_D2; }
  else if(serState==WAIT_D2){ handleMidiMsg(serStatus,serD1,b); serState=WAIT_D1; }
}

void setup(){
  for(auto p:DATA_PINS) pinMode(p,OUTPUT);
  pinMode(PIN_BC1,OUTPUT); pinMode(PIN_BDIR,OUTPUT);
  pinMode(PIN_SEL_A,OUTPUT); pinMode(PIN_SEL_B,OUTPUT);
  pinMode(PIN_SEL_C,OUTPUT); pinMode(PIN_ENABLE,OUTPUT);
  digitalWrite(PIN_ENABLE,HIGH);
  for(uint8_t i=0;i<3;i++){ pinMode(CHIP_LED[i],OUTPUT); digitalWrite(CHIP_LED[i],LED_OFF);}
  for(uint8_t i=0;i<3;i++){ digitalWrite(CHIP_LED[i],LED_ON); delay(200); digitalWrite(CHIP_LED[i],LED_OFF); delay(100);}
  for(uint8_t c=0;c<3;c++){ enableTones(c); for(uint8_t v=0;v<3;v++) stopVoice(c,v);}
  Serial1.begin(31250);
  #if USE_YMPLAYER_SERIAL
    Serial.begin(115200);
    player.begin();
  #endif
}

void loop(){
  while(Serial1.available()) parseSerialMidi(Serial1.read());
  unsigned long now=millis();
  for(uint8_t c=0;c<3;c++) if(ledOnTime[c]&&(now-ledOnTime[c]>=ledFlashMs)){
    digitalWrite(CHIP_LED[c],LED_OFF);
    ledOnTime[c]=0;
  }
  #if !USE_YMPLAYER_SERIAL
    midiEventPacket_t rx;
    while((rx=MidiUSB.read()).header) handleMidiMsg(rx.byte1,rx.byte2,rx.byte3);
  #else
    player.update();
  #endif
  static unsigned long last=millis();
  unsigned long m=millis();
  if(m-last>=5){
    last=m;
    for(uint8_t ch=0;ch<9;ch++) updatePitchMod(ch);
  }
}
