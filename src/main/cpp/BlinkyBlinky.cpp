#include "BlinkyBlinky.h"

BlinkyBlinky::BlinkyBlinky(Gamepiece* gamepiece_)
: gamepiece(gamepiece_) {
    leds.SetLength(BLINKY_BLINKY_PREFERENCE.LED_TOTAL);
    leds.SetData(ledBuffer);
    leds.Start();
}

void BlinkyBlinky::resetToMatchMode(MatchMode priorMode, MatchMode mode) {
    if (mode == MatchMode::DISABLED) {
        isDisabled = true;
        currentMode = Mode::RAINBOW;
    } else {
        isDisabled = false;
    }
}

void BlinkyBlinky::process() {
    using LEDData = frc::AddressableLED::LEDData;

    bool overridePatterns = false;

    if (!isDisabled) {
        // MARK: Triple Flash for GP Intake
        static bool flashFinished = false;
        static int flashTimer = 0;
        if (gamepiece->hasGamepiece() && !flashFinished) {
            const int flashDuration = 6;
            if (flashTimer % (flashDuration * 2) < flashDuration) {
                ledBuffer.fill(LEDData(255, 255, 255));
            } else {
                ledBuffer.fill(LEDData(0, 0, 0));
            }
            flashTimer++;
            if (flashTimer >= flashDuration * 5) {
                flashFinished = true;
                flashTimer = 0;
            }
            overridePatterns = true;
        } else if (!gamepiece->hasGamepiece()) {
            flashFinished = false;
        }
    }

    if (!overridePatterns) {
        switch (currentMode) {
        case Mode::OFF:
            ledBuffer.fill(LEDData(0, 0, 0));
            break;
        case Mode::RAINBOW:
            static int rainbowPosition = 0;
            applyPercentOverLeds([&](double percent) -> LEDData {
                LEDData color {};
                int hue = int(percent * 180 + rainbowPosition) % 180;
                color.SetHSV(hue, 255, 100);
                return color;
            });
            rainbowPosition++;
            break;
        
        default:
            break;
        }
    }

    leds.SetData(ledBuffer);
}

void BlinkyBlinky::applyPercentOverLeds(std::function<frc::AddressableLED::LEDData(double)> func) {
    // ~AI~ Wrote this
    for (int i = 0; i < ledBuffer.size(); ++i) {
        double percent = i / (double)ledBuffer.size();
        ledBuffer[i] = func(percent);
    }
}