/*
 * DaisySound - audio coprocessor firmware for the Daisy computer.
 * Copyright (C) 2026 Joe Cassara
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include "buffer.h"
#include "timer.h"
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

typedef struct {
  uint16_t freq;
  uint16_t durationMs;
} Note;

// ===========================================================================
// How the audio is generated
// ===========================================================================
//
// There is no DAC on this board. Each of the three voices drives a single
// digital pin that can only be high or low, and the waveform is produced by
// switching that pin at audio rates. A resistor and capacitor on the output
// average the switching into a usable analog signal.
//
// A timer interrupt fires at a fixed rate, kSampleRateHz, and every voice is
// updated exactly once per interrupt. That fixed rate is the sample rate of
// the whole synthesizer, and it is the reason the interrupt handler has to
// stay short: any work added there eats into the time available before the
// next sample is due.
//
// Pitch comes from a technique called a phase accumulator. Each voice owns a
// 32-bit counter, and on every sample a fixed amount is added to it. The
// counter is treated as a fraction of one cycle of the waveform, so it wraps
// from its maximum back to zero naturally on overflow, which is exactly what
// one period of a repeating wave does. The amount added per sample, called
// the increment, sets how fast the counter wraps and therefore the frequency:
//
//     increment = frequency * 2^32 / sample_rate
//
// This is convenient because 32-bit overflow does the wrapping for free, with
// no comparison or reset, and because frequency resolution is very fine.
//
// Voices 0 and 1 are pulse waves. The top 8 bits of the phase counter are
// compared against a duty threshold, and the pin is set high when the phase
// is below it. A threshold at the midpoint gives a square wave; moving it
// changes the pulse width, which changes the harmonic content and so the
// timbre. An optional low-frequency oscillator sweeps that threshold slowly
// up and down, which is the familiar sweeping sound of pulse-width
// modulation. The LFO uses a second phase accumulator of its own.
//
// Voice 2 is noise, produced by a 23-bit linear feedback shift register. The
// register is shifted one bit at a time and the bit shifted in is the
// exclusive-or of two chosen positions, called taps. With well chosen taps
// the register cycles through a very long sequence before repeating, so the
// output sounds random even though it is completely deterministic. Its own
// phase accumulator controls how often the register advances, which is what
// gives the noise a pitch.
//
// Portamento, the audible glide between two notes, works by moving a voice's
// increment gradually toward the target instead of jumping to it. The glide
// is stepped at a slower rate than the sample rate to save processor time in
// the interrupt.

// Voice count. Voices 0 and 1 are pulse waves and support pulse-width
// modulation and portamento; voice 2 is the noise generator.
#define NUM_VOICES 3
#define NUM_PW_VOICES 2

// Output pins. Each voice drives one PORTB bit directly rather than going
// through digitalWrite, which is far too slow to call at the sample rate. The
// Arduino pin numbers and the PORTB bit numbers name the same three pins.
const uint8_t kVoice0Pin = 9;
const uint8_t kVoice1Pin = 10;
const uint8_t kVoice2Pin = 11;
const uint8_t kVoice0Bit = PB1;
const uint8_t kVoice1Bit = PB2;
const uint8_t kVoice2Bit = PB3;

// Interrupt rate, and therefore the synthesizer's sample rate.
const float kSampleRateHz = 31250.0f;

// Timer1 runs with no prescaler and clears on compare match, so the interrupt
// rate is F_CPU / (kTimer1Top + 1). At 16 MHz that is 16000000 / 512 = 31250.
const uint16_t kTimer1Top = 511;

// The phase accumulator spans the full 32-bit range, treated as one cycle of
// the waveform. kPhaseSpan is used when converting a frequency into a
// per-sample increment, and kPhaseMax clamps that result so the conversion
// cannot overflow on the way back to an integer.
const double kPhaseSpan = 4294967296.0;
const uint32_t kPhaseMax = 4294967295u;

// Top bit of the accumulator. It is set for the second half of every cycle,
// which makes it a cheap way to ask which half of the waveform we are in.
const uint32_t kPhaseSecondHalf = 0x80000000u;

// Shifting the accumulator down by this much leaves its most significant
// byte, an 8-bit position within the cycle. Comparisons are done at 8 bits
// because the AVR has no barrel shifter and 32-bit compares are expensive.
const uint8_t kPhaseToByteShift = 24;

// Duty threshold for a 50 percent square wave, out of a full scale of 256.
const uint8_t kPulseWidthCenter = 128;

// The pulse-width LFO folds the top of its own phase into a triangle by
// masking to 7 bits and inverting on the second half of the cycle. Offsetting
// that 0..127 triangle upward keeps the resulting duty inside roughly 25 to
// 75 percent, which sweeps audibly without ever silencing the voice by
// collapsing the pulse to nothing.
const uint8_t kLfoTriangleMask = 0x7Fu;
const uint8_t kLfoDutyOffset = 64u;

// Noise generator. The shift register is 23 bits wide, and the bit fed back
// in is the exclusive-or of the two taps below. The seed only has to be
// nonzero; an all-zero register would be a fixed point and produce silence.
const uint32_t kLfsrSeed = 0x1F37A5u;
const uint32_t kLfsrMask23 = 0x7FFFFFu;
const uint8_t kLfsrHighByteShift = 16;
const uint8_t kLfsrTapA = 6;
const uint8_t kLfsrTapB = 1;

// Portamento glide is stepped once every kPortaTickInterval samples rather
// than on every sample, which keeps the cost out of the hot path. That works
// out to 31250 / 128, roughly kPortaGlideHz updates per second, and that rate
// is what converts a glide time in milliseconds into a number of steps.
const uint8_t kPortaTickInterval = 128;
const uint16_t kPortaGlideHz = 244;
const uint16_t kMsPerSecond = 1000;

volatile uint32_t phase[NUM_VOICES] = {0, 0, 0};
volatile uint32_t inc[NUM_VOICES] = {0, 0, 0};

volatile uint8_t pw[NUM_PW_VOICES] = {kPulseWidthCenter, kPulseWidthCenter};
volatile uint32_t lfo_phase[NUM_PW_VOICES] = {0, 0};
volatile uint32_t lfo_inc[NUM_PW_VOICES] = {0, 0};

volatile uint32_t target_inc[NUM_PW_VOICES] = {0, 0};
volatile int32_t porta_step[NUM_PW_VOICES] = {0, 0};
volatile uint16_t porta_ms[NUM_PW_VOICES] = {0, 0};
static uint8_t porta_tick = 0;

volatile uint32_t noise_lfsr23 = kLfsrSeed;
volatile uint32_t noise_phase = 0;
volatile uint8_t noiseEnabled = 0;

// Converts a frequency in Hz to the per-sample phase increment for a 32-bit
// phase accumulator. Clamped to at least 1 so audible-but-tiny frequencies
// still advance, and to the 32-bit max so the cast cannot wrap.
static uint32_t freqToInc(float f) {
  if (f <= 0.0f) {
    return 0;
  }
  double x = (f * kPhaseSpan) / kSampleRateHz;
  if (x < 1.0) {
    x = 1.0;
  }
  if (x > (double)kPhaseMax) {
    x = (double)kPhaseMax;
  }
  return (uint32_t)(x + 0.5);
}

// The synthesizer. Runs at the audio sample rate and generates all three
// voices directly on PORTB: two pulse voices with optional PWM sweep, plus a
// 23-bit LFSR noise voice. Everything is written to PORTB in one store so the
// three outputs change together. This is the hottest code in the firmware --
// added work here shows up as audible jitter.
ISR(TIMER1_COMPA_vect) {
  uint32_t p0 = phase[0] + inc[0];
  phase[0] = p0;
  uint32_t p1 = phase[1] + inc[1];
  phase[1] = p1;

  if (++porta_tick >= kPortaTickInterval) {
    porta_tick = 0;
    for (uint8_t v = 0; v < NUM_PW_VOICES; v++) {
      int32_t step = porta_step[v];
      if (step) {
        uint32_t cur = inc[v];
        uint32_t tgt = target_inc[v];
        if (cur != tgt) {
          int32_t diff = (int32_t)(tgt - cur);
          if ((step > 0 && diff <= step) || (step < 0 && diff >= step)) {
            inc[v] = tgt;
          } else {
            inc[v] = cur + step;
          }
        }
      }
    }
  }

  uint8_t out = PORTB;
  out &= ~((1 << kVoice0Bit) | (1 << kVoice1Bit) | (1 << kVoice2Bit));

  if (inc[0]) {
    uint8_t duty;
    uint32_t li0 = lfo_inc[0];
    if (li0) {
      uint32_t lp = lfo_phase[0] + li0;
      lfo_phase[0] = lp;
      uint8_t tri7;
      if (lp & kPhaseSecondHalf) {
        tri7 = ~(uint8_t)(lp >> kPhaseToByteShift) & kLfoTriangleMask;
      } else {
        tri7 = (uint8_t)(lp >> kPhaseToByteShift) & kLfoTriangleMask;
      }
      duty = tri7 + kLfoDutyOffset;
    } else {
      duty = pw[0];
    }
    if ((uint8_t)(p0 >> kPhaseToByteShift) < duty) {
      out |= (1 << kVoice0Bit);
    }
  }

  if (inc[1]) {
    uint8_t duty;
    uint32_t li1 = lfo_inc[1];
    if (li1) {
      uint32_t lp = lfo_phase[1] + li1;
      lfo_phase[1] = lp;
      uint8_t tri7;
      if (lp & kPhaseSecondHalf) {
        tri7 = ~(uint8_t)(lp >> kPhaseToByteShift) & kLfoTriangleMask;
      } else {
        tri7 = (uint8_t)(lp >> kPhaseToByteShift) & kLfoTriangleMask;
      }
      duty = tri7 + kLfoDutyOffset;
    } else {
      duty = pw[1];
    }
    if ((uint8_t)(p1 >> kPhaseToByteShift) < duty) {
      out |= (1 << kVoice1Bit);
    }
  }

  if (noiseEnabled) {
    uint32_t prev_np = noise_phase;
    uint32_t np = prev_np + inc[2];
    noise_phase = np;
    if (!(prev_np & kPhaseSecondHalf) && (np & kPhaseSecondHalf)) {
      uint32_t lfsr = noise_lfsr23;
      uint8_t b2 = (uint8_t)(lfsr >> kLfsrHighByteShift);
      uint8_t fb = ((b2 >> kLfsrTapA) ^ (b2 >> kLfsrTapB)) & 1u;
      noise_lfsr23 = ((lfsr << 1) & kLfsrMask23) | fb;
    }
    uint32_t lfsr = noise_lfsr23;
    uint8_t rb0 = (uint8_t)(lfsr);
    uint8_t rb1 = (uint8_t)(lfsr >> 8);
    uint8_t rb2 = (uint8_t)(lfsr >> kLfsrHighByteShift);
    uint8_t nb = (rb0 ^ (rb0 >> 2) ^ (rb0 >> 5) ^ (rb1 >> 1) ^ (rb1 >> 3) ^
                  (rb1 >> 6) ^ (rb2 >> 2) ^ (rb2 >> 4)) &
                 1u;
    if (nb) {
      out |= (1 << kVoice2Bit);
    }
  }

  PORTB = out;
}

// Puts Timer1 in CTC mode at the audio sample rate and enables the compare-A
// interrupt that drives the synth ISR.
static void audio_setupTimer1(void) {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = kTimer1Top;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

// Brings up the audio engine: configures the three output pins, clears all
// per-voice state, and starts the sample-rate timer.
void audio_init(void) {
  pinMode(kVoice0Pin, OUTPUT);
  pinMode(kVoice1Pin, OUTPUT);
  pinMode(kVoice2Pin, OUTPUT);

  DDRB |= (1 << kVoice0Bit) | (1 << kVoice1Bit) | (1 << kVoice2Bit);

  for (uint8_t v = 0; v < NUM_VOICES; ++v) {
    phase[v] = 0;
    inc[v] = 0;
  }
  for (uint8_t v = 0; v < NUM_PW_VOICES; ++v) {
    pw[v] = kPulseWidthCenter;
    lfo_phase[v] = 0;
    lfo_inc[v] = 0;
    target_inc[v] = 0;
    porta_step[v] = 0;
    porta_ms[v] = 0;
  }
  noise_lfsr23 = kLfsrSeed;
  noise_phase = 0;
  noiseEnabled = 0;

  audio_setupTimer1();
}

// Sets a voice's pitch. If portamento is armed and the voice is already
// sounding, this instead sets a glide target and the per-tick step the ISR
// walks toward it. Updates are wrapped in cli/sei because the ISR reads the
// same multi-byte fields.
void audio_setVoice(uint8_t voice, float freqHz) {
  if (voice >= NUM_VOICES) {
    return;
  }
  uint32_t newInc = freqToInc(freqHz);
  cli();
  if (voice < NUM_PW_VOICES && porta_ms[voice] && inc[voice]) {
    target_inc[voice] = newInc;
    int32_t diff = (int32_t)(newInc - inc[voice]);
    uint16_t ticks =
        (uint32_t)porta_ms[voice] * kPortaGlideHz / kMsPerSecond;
    if (ticks < 1) {
      ticks = 1;
    }
    porta_step[voice] = diff / (int32_t)ticks;
    if (porta_step[voice] == 0) {
      porta_step[voice] = (diff > 0) ? 1 : -1;
    }
  } else {
    inc[voice] = newInc;
    if (voice < NUM_PW_VOICES) {
      target_inc[voice] = newInc;
      porta_step[voice] = 0;
    }
  }
  if (voice == 2) {
    noiseEnabled = true;
  }
  sei();
}

// Silences a voice by zeroing its phase increment, and cancels any glide in
// progress so a later note does not slide from a stale pitch.
void audio_voiceOff(uint8_t voice) {
  if (voice >= NUM_VOICES) {
    return;
  }
  cli();
  inc[voice] = 0;
  if (voice < NUM_PW_VOICES) {
    target_inc[voice] = 0;
    porta_step[voice] = 0;
  }
  if (voice == 2) {
    noiseEnabled = false;
  }
  sei();
}

// Sets pulse width for voice 0 or 1, and the LFO rate that sweeps it.
// An lfo_hz of 0 pins the width, giving a static pulse timbre.
void audio_setPW(uint8_t voice, uint8_t pwVal, float lfo_hz) {
  if (voice >= NUM_PW_VOICES) {
    return;
  }
  uint32_t new_lfo_inc = (lfo_hz > 0.0f) ? freqToInc(lfo_hz) : 0u;
  cli();
  pw[voice] = pwVal;
  lfo_inc[voice] = new_lfo_inc;
  lfo_phase[voice] = 0;
  sei();
}

// Sets the glide time between notes for voice 0 or 1. Zero disables glide and
// cancels any that is already running.
void audio_setPortamento(uint8_t voice, uint16_t ms) {
  if (voice >= NUM_PW_VOICES) {
    return;
  }
  cli();
  porta_ms[voice] = ms;
  if (ms == 0) {
    porta_step[voice] = 0;
  }
  sei();
}

// Power-on chirp: C6 for a short moment, audible proof the board booted.
const uint16_t kStartupBeepHz = 1047;
const uint16_t kStartupBeepMs = 75;

// Link to DaisyOS.
const uint32_t kHostBaud = 115200;

const uint8_t kSOP = 0x5c;
const uint8_t kMinFrameSize = 4;
const uint8_t kOffsetPayloadLen = 2;

// Two's-complement checksum over a frame, so a valid frame plus its checksum
// byte sums to zero. Must match the sender in DaisyOS.
int8_t calcchecksum(uint8_t* data_in, size_t data_len) {
  uint8_t raw_sum = 0;

  for (size_t i = 0; i < data_len; i++) {
    raw_sum += data_in[i];
  }

  return (uint8_t)(~raw_sum + 1);
}

typedef enum {
  kAudioSilence = 0x01,
  kAudioVoiceOn = 0x02,
  kAudioVoiceOff = 0x03,
  kAudioProgram = 0x04,
  kAudioPlayProgram = 0x05,
  kAudioStopProgram = 0x06,
  kAudioClearProgram = 0x07,
  kAudioSetProgramRepeat = 0x08,
  kAudioToneOn,
  kAudioToneOff,
  kAudioSetPW,
  kAudioProgramAppend,
  kAudioSetPortamento = 0x10,
  kAudioShutUp = 0x99,
  kAudioReboot = 0x9A,
} AudioMsgId;

#define kMAX_NOTES 64
#define kMAX_PROGRAMS 3
#define kMAX_VOICES 3

typedef struct {
  Note notes[kMAX_NOTES];
  uint8_t note_pos;
  uint8_t num_notes;
  uint8_t status_flags;
} AudioProgram;

typedef struct {
  uint16_t freq;
  Timer countdown_timer;
  bool is_active;
} Voice;

typedef struct {
  Voice voice;
} AudioPlayer;

AudioProgram audio_programs[kMAX_PROGRAMS];
AudioPlayer audio_player[kMAX_VOICES];
Voice tone_player[kMAX_VOICES];

enum {
  kPlayBit = 1 << 0,
  kIsPLayingBit = 1 << 1,
  kRepeatBit = 1 << 2,
};

// Unpacks a note list from a wire payload into a program slot. Notes arrive as
// 4-byte big-endian pairs of frequency and duration; a payload that is not a
// whole number of notes is rejected.
bool EncodeAudioProgram(uint8_t* payload, uint8_t len) {
  if ((len - 1) % 4 != 0) {
    return false;
  }
  uint8_t program_num = payload[0];
  uint8_t ram_pos = 0;
  for (uint8_t pos = 0; pos < len - 1; pos += 4) {
    audio_programs[program_num].notes[ram_pos].freq =
        (payload[1 + pos] << 8) | (payload[2 + pos]);
    audio_programs[program_num].notes[ram_pos].durationMs =
        (payload[3 + pos] << 8) | (payload[4 + pos]);
    ram_pos++;
  }
  audio_programs[program_num].status_flags = 0;

  audio_programs[program_num].note_pos = 0;

  audio_programs[program_num].num_notes = ram_pos;
  return true;
}

// Starts or stops a program, rewinding it to its first note either way.
void SetStateToPlay(uint8_t program_num, bool en) {
  if (en) {
    audio_programs[program_num].status_flags |= kPlayBit;
  } else {
    audio_programs[program_num].status_flags &= (uint8_t)~(kPlayBit | kIsPLayingBit);
  }
  audio_programs[program_num].note_pos = 0;
}

// Ends a one-shot tone and frees its voice slot.
void StopPlayingTone(uint8_t voice) {
  audio_voiceOff(voice);
  tone_player[voice].is_active = false;
}

// Starts a one-shot tone and arms the timer that will stop it. Returns
// immediately; HandlePlayTones does the stopping.
void StartPlayingTone(uint8_t voice, uint16_t freq, uint32_t time) {
  audio_setVoice(voice, freq);
  tone_player[voice].freq = freq;
  tone_player[voice].is_active = true;
  TimerCreate(&tone_player[voice].countdown_timer, (uint64_t)time);
#if 0
  char buf[50];
  sprintf(buf, "%d,%d,%d\n", voice, freq, time);
  Serial.println(buf);
#endif
}

// Polled from the main loop: silences any one-shot tone whose duration has
// elapsed. Keeps note timing off the ISR.
void HandlePlayTones() {
  for (uint8_t voice = 0; voice < kMAX_VOICES; voice++) {
    Voice* voice_ptr = &tone_player[voice];
    if (voice_ptr->is_active) {
      if (TimerIsDone(&voice_ptr->countdown_timer)) {
        StopPlayingTone(voice);
      }
    }
  }
}

// Advances one sequencer program by one main-loop step: starts the current
// note, or moves to the next once its timer expires, looping or halting at the
// end depending on the repeat flag. Notes are not silenced between steps when
// portamento is on, so the next note can glide from the current pitch.
void PlayProgram(uint8_t program_num) {
  uint8_t voice = program_num % 3;
  AudioProgram* pgm = &audio_programs[program_num];

  if ((pgm->status_flags & kPlayBit) == 0) {
    return;
  }
  if ((pgm->status_flags & kIsPLayingBit) == 0) {
    pgm->status_flags |= kIsPLayingBit;
    audio_player[voice].voice.freq = pgm->notes[pgm->note_pos].freq;
    TimerCreate(&audio_player[voice].voice.countdown_timer,
                pgm->notes[pgm->note_pos].durationMs);
    audio_setVoice(voice, audio_player[voice].voice.freq);
  } else {
    if (TimerIsDone(&audio_player[voice].voice.countdown_timer)) {
      if (voice >= NUM_PW_VOICES || porta_ms[voice] == 0) {
        audio_voiceOff(voice);
      }
      pgm->note_pos++;
      pgm->status_flags &= (uint8_t)~kIsPLayingBit;
      if (pgm->note_pos == kMAX_NOTES || pgm->note_pos == pgm->num_notes) {
        audio_voiceOff(voice);
        pgm->status_flags &= ~(kIsPLayingBit | kPlayBit);
        if (pgm->status_flags & kRepeatBit) {
          pgm->status_flags |= kPlayBit;
          pgm->note_pos = 0;
        }
      }
    }
  }
}
// Short power-on chirp, audible proof the board booted and audio works.
void StartupBeep(void) {
  audio_setVoice(0, kStartupBeepHz);
  delay(kStartupBeepMs);
  audio_voiceOff(0);
}

// Validates and dispatches one complete command frame from DaisyOS, acking the
// message id back over the serial link.
void HandleAudioCommand(Buffer* buffer) {
  const uint8_t msg_id = buffer->buffer[1];
  const uint8_t payload_len = buffer->buffer[2];
  const uint8_t checksum = buffer->buffer[buffer->buffer_len - 1];
  uint8_t* payload = &(buffer->buffer[3]);
  uint8_t calc_checksum = calcchecksum(buffer->buffer, buffer->buffer_len - 1);
  Serial.write(msg_id);
#if 0
  audio_setVoice(0, 440);
  delay(25);
  audio_voiceOff(0);
#endif
  if (checksum == calc_checksum) {
    switch (msg_id) {
      case kAudioSilence: {
        audio_voiceOff(0);
        audio_voiceOff(1);
        audio_voiceOff(2);
        break;
      }
      case kAudioVoiceOn: {
        uint16_t note = 0;
        uint8_t voice = 0;
        voice = payload[0];
        note = payload[1] << 8;
        note |= payload[2];
        audio_setVoice(voice, note);
        break;
      }

      case kAudioVoiceOff: {
        uint8_t voice = 0;
        voice = payload[0];
        audio_voiceOff(voice);
        break;
      }
      case kAudioToneOn: {
        uint16_t note = 0;
        uint8_t voice = 0;
        uint32_t time = 0;
        voice = payload[0];
        note = payload[1] << 8;
        note |= payload[2];
        time = (uint32_t)payload[3] << 24;
        time |= (uint32_t)payload[4] << 16;
        time |= (uint32_t)payload[5] << 8;
        time |= (uint32_t)payload[6];
        StartPlayingTone(voice, note, time);
        break;
      }

      case kAudioToneOff: {
        uint8_t voice = 0;
        voice = payload[0];
        audio_voiceOff(voice);
        StopPlayingTone(voice);
        break;
      }

      case kAudioProgram: {
        EncodeAudioProgram(payload, payload_len);
        break;
      }

      case kAudioProgramAppend: {
        uint8_t program_num = payload[0];
        if (program_num < kMAX_PROGRAMS) {
          uint8_t ram_pos = audio_programs[program_num].num_notes;
          for (uint8_t pos = 0; pos < payload_len - 1 && ram_pos < kMAX_NOTES;
               pos += 4) {
            audio_programs[program_num].notes[ram_pos].freq =
                (payload[1 + pos] << 8) | (payload[2 + pos]);
            audio_programs[program_num].notes[ram_pos].durationMs =
                (payload[3 + pos] << 8) | (payload[4 + pos]);
            ram_pos++;
          }
          audio_programs[program_num].num_notes = ram_pos;
        }
        break;
      }

      case kAudioPlayProgram: {
        uint8_t program_num = payload[0];
        SetStateToPlay(program_num, true);
        break;
      }

      case kAudioStopProgram: {
        uint8_t program_num = payload[0];
        audio_voiceOff(program_num % 3);
        SetStateToPlay(program_num, false);
        break;
      }

      case kAudioClearProgram: {
        uint8_t program_num = payload[0];
        SetStateToPlay(program_num, false);
        memset(&audio_programs[program_num], 0,
               sizeof(audio_programs[program_num]));
        break;
      }

      case kAudioSetPW: {
        uint8_t voice = payload[0];
        uint8_t pwVal = payload[1];
        uint16_t lfo_rate_x100 = ((uint16_t)payload[2] << 8) | payload[3];
        float lfo_hz = (float)lfo_rate_x100 / 100.0f;
        audio_setPW(voice, pwVal, lfo_hz);
        break;
      }

      case kAudioSetPortamento: {
        uint8_t voice = payload[0];
        uint16_t ms = ((uint16_t)payload[1] << 8) | payload[2];
        audio_setPortamento(voice, ms);
        break;
      }

      case kAudioSetProgramRepeat: {
        uint8_t program_num = payload[0];
        bool repeat = (payload[1] == 1);
        if (repeat) {
          audio_programs[program_num].status_flags |= kRepeatBit;
        } else {
          audio_programs[program_num].status_flags &= (uint8_t)~kRepeatBit;
        }
        break;
      }

      case kAudioShutUp: {
        memset(&audio_player, 0, sizeof(audio_player));

        audio_voiceOff(0);
        audio_voiceOff(1);
        audio_voiceOff(2);

        for (uint8_t v = 0; v < NUM_PW_VOICES; v++) {
          audio_setPW(v, kPulseWidthCenter, 0.0f);
          audio_setPortamento(v, 0);
        }

        for (uint8_t prg_num = 0; prg_num < kMAX_PROGRAMS; prg_num++) {
          audio_programs[prg_num].status_flags &= ~(kIsPLayingBit | kPlayBit);
        }

        break;
      }

      case kAudioReboot: {
        cli();
        wdt_enable(WDTO_15MS);
        for (;;) {
        }
      }

      default: {
        audio_voiceOff(0);
        audio_voiceOff(1);
        audio_voiceOff(2);
        break;
      }
    }
  } else {
#if 0
    audio_setVoice(0, 1470);
    delay(25);
    audio_voiceOff(0);
#endif
  }
}

// Drains the UART, reassembling framed commands. Bytes outside a frame are
// discarded until a start-of-packet marker appears, so the link resynchronises
// on its own after a garbled frame.
bool PollSerial() {
  static Buffer buffer;
  static uint8_t data = 0;
  while (Serial.available()) {
    data = Serial.read();
    if (buffer.in_frame == false && data == kSOP) {
      BufferInit(&buffer);
      buffer.in_frame = true;
    }
    if (buffer.in_frame == true) {
      BufferAdd(data, &buffer);
      if (buffer.buffer_len ==
          kMinFrameSize + buffer.buffer[kOffsetPayloadLen]) {
        HandleAudioCommand(&buffer);
        BufferInit(&buffer);
      }
    } else {
    }
  }
  return true;
}

// Boot: disable the watchdog, open the link to DaisyOS, start the synth, and
// clear all sequencer state.
void setup() {
  wdt_disable();
  Serial.begin(kHostBaud);
  audio_init();
  memset(&audio_programs, 0, sizeof(AudioProgram) * kMAX_PROGRAMS);
  memset(&audio_player, 0, sizeof(AudioPlayer) * kMAX_VOICES);
  memset(&tone_player, 0, sizeof(Voice) * kMAX_VOICES);
  StartupBeep();
}

// Main superloop. Audio generation itself lives in the ISR; this only handles
// incoming commands and sequencer note timing.
void loop() {
  PollSerial();
  HandlePlayTones();
  PlayProgram(0);
  PlayProgram(1);
  PlayProgram(2);
}
