#pragma once

// FreeInk buzzer (LEDC PWM tone beeper).
//
// Drives the passive buzzer on BoardConfig::ACTIVE.audio.buzzer (e.g. the
// Sticky's GPIO48, Murphy's GPIO46) as a square-wave tone generator via the
// ESP32 LEDC peripheral — the same approach as the vendor demo. This is a tone
// device, NOT PCM audio, so it is independent of AudioManager (which streams WAV
// through an I2S codec). Boards without a buzzer (FREEINK_CAP_BUZZER off, or the
// buzzer pin unassigned) link stub bodies and present() returns false.

#include <Arduino.h>

#include <cstdint>

namespace freeink {

class Buzzer {
 public:
  static constexpr uint32_t kDefaultBeepFreq = 2000;  // Hz
  static constexpr uint32_t kDefaultBeepMs = 80;

  // Attaches LEDC to the buzzer pin. Returns false when the active board has no
  // buzzer.
  bool begin();
  bool present() const { return begun_; }

  // Plays a square-wave tone at freqHz. durationMs > 0 blocks for that long and
  // then stops; durationMs == 0 starts a continuous tone (end it with noTone()).
  void tone(uint32_t freqHz, uint32_t durationMs = 0);

  // A short default beep (kDefaultBeepFreq for kDefaultBeepMs).
  void beep() { tone(kDefaultBeepFreq, kDefaultBeepMs); }

  // Silences a continuous tone.
  void noTone();

  // Loudness of later tones, 1-100 % of the full 50 % duty square wave. A
  // passive buzzer is loudest at 50 % duty; a narrower pulse at the same pitch
  // drives it with less energy, so it sounds quieter (not linearly).
  void setVolume(uint8_t percent) { volume_ = percent == 0 ? 1 : (percent > 100 ? 100 : percent); }
  uint8_t volume() const { return volume_; }

  // Releases the LEDC channel / pin. begin() re-attaches.
  void end();

 private:
  bool begun_ = false;
  uint8_t volume_ = 100;
};

}  // namespace freeink

using Buzzer = freeink::Buzzer;
