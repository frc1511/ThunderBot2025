#pragma once

#include <functional>

#include <frc/AddressableLED.h>
#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"
#include "GamEpiece/Gamepiece.h"

class BlinkyBlinky : public Component {
  public:
    BlinkyBlinky(Gamepiece* gamepiece_);

    void process();
    void resetToMatchMode(MatchMode priorMode, MatchMode mode);

    enum class Mode {
      OFF,
      RAINBOW
    };

    Mode currentMode = Mode::RAINBOW;
  private:
    bool isDisabled = true;
    // Slot not confirmed
    frc::AddressableLED leds {PWM_SLOT_9};
    std::array<frc::AddressableLED::LEDData, BLINKY_BLINKY_LED_TOTAL> ledBuffer;

    void applyPercentOverLeds(std::function<frc::AddressableLED::LEDData(double)> func);

    Gamepiece* gamepiece;
};