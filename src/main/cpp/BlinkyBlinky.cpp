#include "BlinkyBlinky.h"

BlinkyBlinky::BlinkyBlinky(Gamepiece* gamepiece_, Hang* hang_)
: gamepiece(gamepiece_),
  hang(hang_) {
    leds.SetLength((int)PreferencesBlinkyBlinky::LED_TOTAL);
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

    // MARK: Side Lights

    bool overridePatterns = false;
    // If it's enabled and no "other thing" happened outside of blinkyBlinky that set the current mode (like switchboard led disable)
    if (!isDisabled && currentMode == Mode::UNSET) {
        bool showWrist = frc::SmartDashboard::GetBoolean("Controls Elevator(False) or Wrist(True) LED Fade", false);
        if (showWrist) {
            if (gamepiece->wrist != nullptr) {
                if (!gamepiece->wrist->atPreset()) {
                    double wristPercentRotation = gamepiece->wrist->getPercentRotation();
                    double litLEDS = wristPercentRotation * PreferencesBlinkyBlinky::LED_SIDE_STRIP_TOTAL;
                    int majorLEDS = floor(litLEDS);
                    double finalFade = std::clamp((int(litLEDS * 100) % 100) / 100.0, 0.0, 1.0);
                    int sideBufferSize = (int)sideBuffer.size();

                    for (int i = 0; i < sideBufferSize; i++) { 
                        if (i <= majorLEDS) {
                            if (i == majorLEDS) {
                                sideBuffer[i] = LEDData(0, 255 * finalFade, 0); // Green
                            } else {
                                sideBuffer[i] = LEDData(0, 255, 0);
                            }
                        } else {
                            sideBuffer[i] = LEDData(0, 0, 0);                   // Black
                        }
                    }
                    currentSideStatus = "Wrist Percentage";
                    overridePatterns = true;
                }
            }
        } else { // Show Elevator
            if (gamepiece->elevator != nullptr) {
                // Elevator Sides
                if (!gamepiece->elevator->atPreset() && gamepiece->elevator->getCurrentPreset() != Elevator::Preset::kSTOP) {
                    double elevatorPercentHeight = gamepiece->elevator->getPercentHeight();
                    double litLEDS = elevatorPercentHeight * PreferencesBlinkyBlinky::LED_SIDE_STRIP_TOTAL;
                    int majorLEDS = floor(litLEDS);
                    double finalFade = std::clamp((int(litLEDS * 100) % 100) / 100.0, 0.0, 1.0);
                    int sideBufferSize = (int)sideBuffer.size();

                    for (int i = 0; i < sideBufferSize; i++) { 
                        if (i <= majorLEDS) {
                            if (i == majorLEDS) {
                                sideBuffer[i] = LEDData(0, 255 * finalFade, 0); // Green
                            } else {
                                sideBuffer[i] = LEDData(0, 255, 0);
                            }
                        } else {
                            sideBuffer[i] = LEDData(0, 0, 0);                   // Black
                        }
                    }
                    currentSideStatus = "Elevator Percentage";
                    overridePatterns = true;
                }
            }
        }

        if (!overridePatterns && gamepiece->calgae != nullptr) { // Something else is not already overriding
            if (gamepiece->calgae->hasCoral()) {
                sideBuffer.fill(LEDData(250, 0, 220)); // Pink
                currentSideStatus = "Has Coral";
                overridePatterns = true;
            } else if (gamepiece->calgae->hasAlgae()) {
                sideBuffer.fill(LEDData(0, 255, 255)); // Cyan
                currentSideStatus = "Has Algae";
                overridePatterns = true;
            }
        }

        if (hang != nullptr) {
            if (hang->isHung()) {
                static int blueWavePosition = 0;

                applyPercentOverLeds([&](double percent) -> LEDData {
                    LEDData color {};
                    int brightness = (abs((percent - 0.5) * 100) + blueWavePosition) % 100;
                    color.SetHSV(120, 255, brightness);
                    return color;
                });

                blueWavePosition += 2;
                currentSideStatus = "Hang Blue Wave";
                overridePatterns = true;
            }
        }

        // MARK: Triple Flash for GP Intake
        if (gamepiece->hasGamepiece() && !flashFinished) {
            flash(6, 3);
            overridePatterns = true;
            currentSideStatus = "GP Intake Flashes";
        } else if (!gamepiece->hasGamepiece()) {
            flashFinished = false;
        }

        // MARK: NEURALYZE
        if (neuralyze) {
            flash(3, -1);
            overridePatterns = true;
            currentSideStatus = "Neuralyze";
        }

        if (overridePatterns == false) {
            currentMode = Mode::RAINBOW;
        }
    }

    if (!overridePatterns) {
        if (currentMode == Mode::UNSET) {
            currentMode = Mode::RAINBOW;
        }

        if (settings.pitMode) { // (fr) Nous sommes en le Pit Mode | We are in pit mode
            static double pitModePosition = 0;
            applyPercentOverLeds([&](double percent) -> LEDData {
                LEDData color {};
                int hue = int((1 - fabs(percent - 0.5)) * 90 + pitModePosition) % 90 + 90;
                color.SetHSV(hue, 255, 100);
                return color;
            });

            pitModePosition += 0.5;
        } else {
            switch (currentMode) {
                case Mode::OFF:
                    sideBuffer.fill(LEDData(0, 0, 0)); // Black (off)
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
    }

    int sideBufferSize = (int)sideBuffer.size();
    int mainBufferSize = (int)mainLEDBuffer.size();

    for (int i = 0; i < sideBufferSize; i++) {
        mainLEDBuffer[i] = sideBuffer[i];
        mainLEDBuffer[mainBufferSize - PreferencesBlinkyBlinky::LED_SIDE_STRIP_TOTAL + i] = sideBuffer[i];
    }


    // MARK: Status Lights

    
    if (gamepiece->elevator != nullptr) {
        /// Elevator Status
        if (gamepiece->elevator->getUpperLimit() && gamepiece->elevator->getLowerLimit()) {
            statusBuffer[PreferencesBlinkyBlinky::ELEVATOR_STATUS_ID] = LEDData(255, 255, 255); // White
        } else if (gamepiece->elevator->getUpperLimit()) {
            statusBuffer[PreferencesBlinkyBlinky::ELEVATOR_STATUS_ID] = LEDData(0, 255, 0);     // Green
        } else if (gamepiece->elevator->getLowerLimit()) {
            statusBuffer[PreferencesBlinkyBlinky::ELEVATOR_STATUS_ID] = LEDData(255, 0, 0);     // Red
        } else {
            statusBuffer[PreferencesBlinkyBlinky::ELEVATOR_STATUS_ID] = LEDData(0, 0, 255);     // Blue
        }
    }

    if (gamepiece->calgae != nullptr) {
        if (gamepiece->calgae->coralRetroreflectiveTripped()) {
            statusBuffer[PreferencesBlinkyBlinky::CORAL_STATUS_ID] = LEDData(0, 255, 0);        // Green
        } else {
            statusBuffer[PreferencesBlinkyBlinky::CORAL_STATUS_ID] = LEDData(255, 0, 0);        // Red
        }

        if (gamepiece->calgae->algaeRetroreflectiveTripped()) {
            statusBuffer[PreferencesBlinkyBlinky::ALGAE_STATUS_ID] = LEDData(0, 255, 0);        // Green
        } else {
            statusBuffer[PreferencesBlinkyBlinky::ALGAE_STATUS_ID] = LEDData(255, 0, 0);        // Red
        }
    }

    if (hang != nullptr) {
        if (hang->isHung() && hang->isSolenoidUp()) {
            statusBuffer[PreferencesBlinkyBlinky::CAGE_STATUS_ID] = LEDData(255, 255, 255);     // White
        } else if (hang->isHung()) {
            statusBuffer[PreferencesBlinkyBlinky::CAGE_STATUS_ID] = LEDData(0, 255, 0);         // Green
        } else if (hang->isSolenoidUp()) {
            statusBuffer[PreferencesBlinkyBlinky::CAGE_STATUS_ID] = LEDData(255, 0, 0);         // Red
        } else {
            statusBuffer[PreferencesBlinkyBlinky::CAGE_STATUS_ID] = LEDData(0, 0, 255);         // Blue
        }
    }

    int statusBufferSize = (int)statusBuffer.size();

    for (int i = 0; i < statusBufferSize; i++) {
        mainLEDBuffer[PreferencesBlinkyBlinky::LED_SIDE_STRIP_TOTAL + PreferencesBlinkyBlinky::LED_STATUS_STRIP_TOTAL - i - 1] = statusBuffer[i];
    }

    leds.SetData(mainLEDBuffer);

    currentMode = Mode::UNSET;
}

void BlinkyBlinky::sendFeedback() {
    frc::SmartDashboard::PutString("BlinkyBlinky Side LED Status", currentSideStatus);
    currentSideStatus = "NONE";
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

    if (flashTimer >= (spacing * (flashCount + 1)) && flashCount != -1) {
        flashFinished = true;
        flashTimer = 0;
    }
}