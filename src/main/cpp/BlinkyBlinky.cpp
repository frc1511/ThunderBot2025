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
    // If it's enabled and no "other thing" happened outside of blinkyBlinky that set the current mode (like switchboard led disable)
    if (!isDisabled && currentMode == Mode::UNSET) {
        // MARK: Triple Flash for GP Intake
        if (gamepiece->hasGamepiece() && !flashFinished) {
            flash(6, 3);
            overridePatterns = true;
        } else if (!gamepiece->hasGamepiece()) {
            flashFinished = false;
        }

        // MARK: NEURALYZE
        if (neuralyze) {
            flash(2, -1);
            overridePatterns = true;
        }

        if (!gamepiece->elevator->atPreset() && gamepiece->elevator->getCurrentPreset() != Elevator::Preset::kSTOP) {
            double elevatorPercentHeight = gamepiece->elevator->getPercentHeight();
            int litLEDS = floor(elevatorPercentHeight * BLINKY_BLINKY_PREFERENCE.LED_TOTAL);
            for (int i = 0; i < ledBuffer.size(); i++) {
                if (i <= litLEDS) {
                    ledBuffer[i] = LEDData(0, 255, 0); // green
                } else {
                    ledBuffer[i] = LEDData(0, 0, 0); // black
                }
            }
            overridePatterns = true;
        }
        if (gamepiece->calgae->hasCoral()) {
            ledBuffer.fill(LEDData(250, 0, 220)); // pink
            overridePatterns = true;
        }
        
        if (gamepiece->calgae->hasAlgae()) {
            ledBuffer.fill(LEDData(0, 255, 255)); // cyan
            overridePatterns = true;
        }


        if (overridePatterns == false) {
            currentMode = Mode::RAINBOW;
        }
    }

    if (!overridePatterns) {
        switch (currentMode) {
        case Mode::OFF:
            ledBuffer.fill(LEDData(0, 0, 0)); //black (off)
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

    currentMode = Mode::UNSET;
}

void BlinkyBlinky::applyPercentOverLeds(std::function<frc::AddressableLED::LEDData(double)> func) {
    // ~AI~ Wrote this
    for (int i = 0; i < ledBuffer.size(); ++i) {
        double percent = i / (double)ledBuffer.size();
        ledBuffer[i] = func(percent);
    }
}

void BlinkyBlinky::flash(int spacing, int flashCount) {
    using LEDData = frc::AddressableLED::LEDData;
    
    if (flashTimer % (spacing * 2) < spacing) {
        ledBuffer.fill(LEDData(255, 255, 255)); //white 
    } else {
        ledBuffer.fill(LEDData(0, 0, 0));
    }
    flashTimer++;
    if (flashTimer >= (spacing * flashCount) && flashCount != -1) {
        flashFinished = true;
        flashTimer = 0;
    }
}