# YM2149F Polyphonic Synth - Turbo Sound x 3

An Arduino Pro Micro-based polyphonic synthesizer using three Yamaha YM2149F sound chips, complete with MIDI input, per-channel pitch bend, vibrato, expression, portamento and noise-based drum channel, and LED indicators.

## Features

- **3× YM2149F Chips**: Provides 9 independent tone voices (3 per chip).  
- **MIDI Input**: Supports USB-MIDI (via MIDIUSB library) and TRS-serial MIDI (31250 baud).  
- **Per-Channel Voice Routing**: Maps MIDI channels 1–9 to specific chip voices:  
  - Channels 1–3 → Chip 0 voices A/B/C  
  - Channels 4–6 → Chip 1 voices A/B/C  
  - Channels 7–9 → Chip 2 voices A/B/C  
- **Pitch Bend & Vibrato**: Standard pitch bend (±2 semitones) and CC1 mod-wheel vibrato (±1 semitone) per channel.
- **Other Standard CC**: Velocity, Expression, Portamento
- **Drum Noise Channels**: MIDI channels 10 trigger noise percussion on chip 2.  
- **LED Indicators**: LEDs light when each YM chip is active, with inverted logic (LOW = on).  
- **74HC138 Decoder**: Selects among the three chips via A/B/C lines.
- **Serial playback support**: Compatible player is a work in progress..  

## Hardware

<h1 align="center">
    <img width="55%" src="ym2149f-tsx3.png">
</h1>

### Components

- Arduino Pro Micro (ATmega32U4, 5 V)  
- 3 × YM2149F sound chips  
- 74HC138 3-to-8 decoder for chip select  
- Logic gates (74LS08, 74LS04), optional for reset or additional addressing  
- 8 × digital lines for data bus (D0–D7)  
- Control lines: BC1, BDIR, RESET, clock output  
- Audio mixing: resistors + capacitor per channel  
- 3 × LEDs for chip activity  
- 3.5 mm stereo jack for audio output  

### Pin Connections

| Arduino Pin | Function       | YM2149F / 74HC138                    |
|-------------|----------------|--------------------------------------|
| 2–9         | D0–D7          | Data bus D0–D7                       |
| 10          | BC1            | YM2149F BC1                          |
| 20          | BDIR           | YM2149F BDIR                         |
| A3, A1, A0  | SEL A, B, C    | 74HC138 A, B, C inputs               |
| A2          | Enable (high)  | 74HC138 enable                       |
| 9           | Clock out      | YM2149F CLK                          |
| 15 (RESET)  | nRESET         | YM2149F RESET                        |
| LEDs        | 16, 14, 15     | Chip 0, 1, 2 indicator (LOW = on)    |

## Software Setup

1. **Install Arduino IDE** (v1.8 or later).  
2. **Add MIDIUSB library** via Library Manager.  
3. **Load `YM2149Synth.ino`** in Arduino IDE.  
4. **Select “Arduino Pro Micro”** the COM port, and upload.  

## Usage

- Connect a USB-MIDI controller or TRS-MIDI cable.  
- Send notes on channels 1–9 to play polyphonic voices.  
- Use pitch bend wheel for ±2 semitone bends.  
- Use mod wheel (CC1) for vibrato effect.
- Expression (CC11)
- Expression Value (CC7)
- Portamento (CC65) > 64
- Portamento Time (CC5)  
- Send on MIDI channel 10 (Chip 3) for noise.  
- LEDs will flash when each chip is active. 

Special thanks to Ben Baker for his support and code contributions: https://baker76.com, https://github.com/benbaker76
