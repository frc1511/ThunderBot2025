#include "BlinkyBlinky.h"

BlinkyBlinky::BlinkyBlinky(Gamepiece* gamepiece_, Hang* hang_)
: gamepiece(gamepiece_),
  hang(hang_) {
    leds.SetLength(BLINKY_BLINKY_PREFERENCE.LED_TOTAL);
    leds.SetData(mainLEDBuffer);
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
        if (gamepiece->elevator != nullptr) {
            // Elevator Sides
            if (!gamepiece->elevator->atPreset() && gamepiece->elevator->getCurrentPreset() != Elevator::Preset::kSTOP) {
                double elevatorPercentHeight = gamepiece->elevator->getPercentHeight();
                int litLEDS = floor(elevatorPercentHeight * BLINKY_BLINKY_PREFERENCE.LED_TOTAL);
                int sideBufferSize = (int)sideBuffer.size();
                for (int i = 0; i < sideBufferSize; i++) {
                    if (i <= litLEDS) {
                        sideBuffer[i] = LEDData(0, 255, 0);                                         // Green
                    } else {
                        sideBuffer[i] = LEDData(0, 0, 0);                                           // Black
                    }
                }
                overridePatterns = true;
            }

            // Elevator Status
            if (gamepiece->elevator->getUpperLimit() && gamepiece->elevator->getLowerLimit()) {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.ELEVATOR_STATUS_ID] = LEDData(255, 255, 255); // White
            } else if (gamepiece->elevator->getUpperLimit()) {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.ELEVATOR_STATUS_ID] = LEDData(0, 255, 0);     // Green
            } else if (gamepiece->elevator->getLowerLimit()) {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.ELEVATOR_STATUS_ID] = LEDData(255, 0, 0);     // Red
            } else {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.ELEVATOR_STATUS_ID] = LEDData(0, 0, 255);     // Blue
            }
        }

        if (gamepiece->calgae != nullptr) {
            if (gamepiece->calgae->hasCoral()) {
                sideBuffer.fill(LEDData(250, 0, 220));                                              // Pink
                statusBuffer[BLINKY_BLINKY_PREFERENCE.CORAL_STATUS_ID] = LEDData(0, 255, 0);        // Green
                overridePatterns = true;
            } else {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.CORAL_STATUS_ID] = LEDData(255, 0, 0);        // Red
            }
            
            if (gamepiece->calgae->hasAlgae()) {
                sideBuffer.fill(LEDData(0, 255, 255));                                              // Cyan
                statusBuffer[BLINKY_BLINKY_PREFERENCE.ALGAE_STATUS_ID] = LEDData(0, 255, 0);        // Green
                overridePatterns = true;
            } else {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.ALGAE_STATUS_ID] = LEDData(255, 0, 0);        // Red
            }
        }

        if (hang != nullptr) {
            if (hang->isHung() && hang->isSolenoidUp()) {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.CAGE_STATUS_ID] = LEDData(255, 255, 255);     // White
            } else if (hang->isHung()) {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.CAGE_STATUS_ID] = LEDData(0, 255, 0);         // Green
            } else if (hang->isSolenoidUp()) {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.CAGE_STATUS_ID] = LEDData(255, 0, 0);         // Red
            } else {
                statusBuffer[BLINKY_BLINKY_PREFERENCE.CAGE_STATUS_ID] = LEDData(0, 0, 255);         // Blue
        }
    }

        if (overridePatterns == false) {
            currentMode = Mode::RAINBOW;
        }
    }

    if (!overridePatterns) {
        switch (currentMode) {
        case Mode::OFF:
            sideBuffer.fill(LEDData(0, 0, 0));                                                      // Black (off)
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

    int sideBufferSize = (int)sideBuffer.size();
    int mainBufferSize = (int)mainLEDBuffer.size();
    for (int i = 0; i < sideBufferSize; i++) {
        mainLEDBuffer[i] = sideBuffer[i];
        mainLEDBuffer[mainBufferSize - i] = sideBuffer[i];
    }

    int statusBufferSize = (int)statusBuffer.size();
    for (int i = 0; i < statusBufferSize; i++) {
        mainLEDBuffer[i + BLINKY_BLINKY_PREFERENCE.LED_SIDE_STRIP_TOTAL] = statusBuffer[i];
    }

    leds.SetData(mainLEDBuffer);

    currentMode = Mode::UNSET;
}

void BlinkyBlinky::applyPercentOverLeds(std::function<frc::AddressableLED::LEDData(double)> func) {
    // ~AI~ Wrote this
    int sideBufferSize = (int)sideBuffer.size();
    for (int i = 0; i < sideBufferSize; ++i) {
        double percent = i / (double)sideBufferSize;
        sideBuffer[i] = func(percent);
    }
}

void BlinkyBlinky::flash(int spacing, int flashCount) {
    using LEDData = frc::AddressableLED::LEDData;
    
    if (flashTimer % (spacing * 2) < spacing) {
        sideBuffer.fill(LEDData(255, 255, 255)); //white 
        statusBuffer.fill(LEDData(255, 255, 255));
    } else {
        sideBuffer.fill(LEDData(0, 0, 0));
    }
    flashTimer++;
    if (flashTimer >= (spacing * flashCount) && flashCount != -1) {
        flashFinished = true;
        flashTimer = 0;
    }
}