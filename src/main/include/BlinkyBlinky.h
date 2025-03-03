#pragma once

#include <functional>
#include <frc/AddressableLED.h>

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"
#include "GamEpiece/Gamepiece.h"
#include "Hang.h"

class BlinkyBlinky : public Component {
  public:
    BlinkyBlinky(Gamepiece* gamepiece_, Hang* hang_);

    void process();
    void resetToMatchMode(MatchMode priorMode, MatchMode mode);
    void sendFeedback() override;

    enum class Mode {
      UNSET,
      OFF,
      RAINBOW
    };

    Mode currentMode = Mode::RAINBOW;
  
    bool neuralyze = false;

  private:
    bool isDisabled = true;
    std::string currentSideStatus = "NONE";

    bool flashFinished = false;
    int flashTimer = 0;

    // Slot not confirmed
    frc::AddressableLED leds {PWM_BLINKY};
    std::array<frc::AddressableLED::LEDData, PreferencesBlinkyBlinky::LED_TOTAL> mainLEDBuffer;
    std::array<frc::AddressableLED::LEDData, PreferencesBlinkyBlinky::LED_SIDE_STRIP_TOTAL> sideBuffer;
    std::array<frc::AddressableLED::LEDData, PreferencesBlinkyBlinky::LED_STATUS_STRIP_TOTAL> statusBuffer;

    void applyPercentOverLeds(std::function<frc::AddressableLED::LEDData(double)> func);

    void flash(int spacing, int flashCount);

    Gamepiece* gamepiece;
    Hang* hang;
};