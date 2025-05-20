#include "Controls.h"

Controls::Controls(Drive* drive_, Gamepiece* gamepiece_, BlinkyBlinky* blinkyBlinky_, Hang* hang_, Limelight* limelight_):
    drive(drive_),
    gamepiece(gamepiece_),
    blinkyBlinky(blinkyBlinky_),
    hang(hang_),
    limelight(limelight_)
{
    frc::SmartDashboard::PutBoolean("Controls Elevator(False) or Wrist(True) LED Fade", false);
}

void Controls::process() {
    utilizeSwitchBoard();
    // MARK: Drive
    if (drive != nullptr && driveController.IsConnected() && !driveDisable) {
        // Drive limiting based on elevator position
        double speedReduction = 0.0;

        if (gamepiece->elevator != nullptr && !EPDSLDisable) {
            // Percentage from L3 -> NET
            double elevatorPercent = ((gamepiece->elevator->getPosition()                    - gamepiece->elevator->Position[Elevator::Preset::kL3]) / 
                                      (gamepiece->elevator->Position[Elevator::Preset::kNET] - gamepiece->elevator->Position[Elevator::Preset::kL3])).value();

            if (elevatorPercent > 0) {
                elevatorPercent = 0.92 - pow(elevatorPercent - 1, 2);
                elevatorPercent = std::clamp(elevatorPercent, 0.0, 1.0);
                speedReduction = elevatorPercent;
            }
        }

        // drive->setAccelerationReduction(accelerationReduction);

        double xPercent = driveController.GetLeftX();
        if (fabs(xPercent) < PreferencesControls::AXIS_DEADZONE)
            xPercent = 0;
        double yPercent = driveController.GetLeftY();
        if (fabs(yPercent) < PreferencesControls::AXIS_DEADZONE)
            yPercent = 0;
        double rotPercent = driveController.GetRightX();
        if (fabs(rotPercent) < PreferencesControls::AXIS_DEADZONE)
            rotPercent = 0;

        /// In case we need them in ~THE PIT~
        // bool lockX = fabs(driveController.GetLeftTriggerAxis()) > PreferencesControls::AXIS_DEADZONE;
        // bool lockY = fabs(driveController.GetRightTriggerAxis()) > PreferencesControls::AXIS_DEADZONE;
        bool lockX = false;
        bool lockY = false;

        bool slowDrive = driveController.GetLeftBumperButtonPressed();
        bool speedUpDrive = driveController.GetRightBumperButtonPressed();
        bool lockRot = false;//driveController.GetStartButtonPressed();
        bool resetIMU = driveController.GetStartButtonPressed();
        bool brickMode = driveController.GetAButton();
        bool robotCentricFromController = fabs(driveController.GetRightTriggerAxis()) > PreferencesControls::AXIS_DEADZONE;
        unsigned flags = 0;

        if (lockX)
            flags |= Drive::ControlFlag::LOCK_X;

        if (lockY)
            flags |= Drive::ControlFlag::LOCK_Y;

        if (lockRot)
            flags |= Drive::ControlFlag::LOCK_ROT;

        if (brickMode)
            flags |= Drive::ControlFlag::BRICK;


        if (resetIMU) {
            printf("Driver pressed the funny button (reset IMU was pressed)\n");
            drive->resetOdometry();
        }

        if (slowDrive)
            drive->slowYourRoll();

        if (speedUpDrive)
            drive->unslowYourRoll();

        double finalSpeedReduction = std::clamp(1 - speedReduction, 0.0, 1.0) * -1;

        frc::SmartDashboard::PutNumber("Controls Final Speed Reduction", finalSpeedReduction);
        
        bool isLiningUp = driveController.GetBButton(); // Line up if B button held

        if (flags == 0 && xPercent == 0 && yPercent == 0 && rotPercent == 0 && !robotCentricFromController && isLiningUp) {
            isL4AutoAlign = driveController.GetXButtonPressed() ? !isL4AutoAlign : isL4AutoAlign;   // Toggle if start pressed, else retain state

            LineupHorizontal lineUpHoriz = LineupHorizontal::kCENTER;
            if (driveController.GetPOV() == 270) { // Left
                lineUpHoriz = LineupHorizontal::kLEFT;
            } else if (driveController.GetPOV() == 90) { // Right
                lineUpHoriz = LineupHorizontal::kRIGHT;
            }
            feedbackHorizontalAutoAlign = lineUpHoriz;

            bool finalL4Align = lineUpHoriz == LineupHorizontal::kCENTER ? false : isL4AutoAlign; // Don't obay L4 auto align if we are doing the center (algae) only for the branches

            drive->beginLineup(lineUpHoriz, finalL4Align);
        } else {
            isLiningUp = false;
            if (fieldCentric && !robotCentricFromController) {
                if (auto ally = frc::DriverStation::GetAlliance()) {
                    if (ally == frc::DriverStation::Alliance::kRed) {
                        finalSpeedReduction *= -1;
                        rotPercent *= -1;
                    }
                }
                flags |= Drive::ControlFlag::FIELD_CENTRIC;
            }

            // SWAP: 90_deg offset for drive
            drive->driveFromPercents(yPercent * finalSpeedReduction, xPercent * finalSpeedReduction, rotPercent * finalSpeedReduction, flags);
        }

    }

    bool hasBeenSetByAux = false;
    // MARK: Aux
    if (gamepiece != nullptr && auxController.IsConnected() && !auxDisable) {
        lastShouldToggleStationPreset = controllerToggleStationPreset;
        controllerToggleStationPreset = fabs(auxController.GetLeftTriggerAxis()) > PreferencesControls::AXIS_DEADZONE;
        bool shouldToggleStationPreset = controllerToggleStationPreset != lastShouldToggleStationPreset && controllerToggleStationPreset; // If we changed to true
        frc::SmartDashboard::PutBoolean("Controls should Toggle Station", shouldToggleStationPreset);

        if (shouldToggleStationPreset) {
            switch (currentStationState) {
                case StationState::kNOT_ACTIVE:
                    currentStationState = StationState::kHIGH;
                    break;
                case StationState::kHIGH:
                    currentStationState = StationState::kLOW;
                    break;
                case StationState::kLOW:
                    currentStationState = StationState::kHIGH;
                    break;
            }
        }

        bool toCoralStation = currentStationState == StationState::kHIGH;
        bool toCoralStationLow = currentStationState == StationState::kLOW;
        bool toL1 = auxController.GetAButton();
        bool toL2 = auxController.GetBButton();
        bool toL3 = auxController.GetXButton();
        bool toL4 = auxController.GetYButton();
        bool toStop = auxController.GetRightStickButtonPressed();
        bool toTransit = auxController.GetLeftBumperButton();
        bool toReefLow = auxController.GetPOV(0) == 90;
        bool toReefHigh = auxController.GetPOV(0) == 0;
        bool toGround = auxController.GetPOV() == 180;
        bool toProcessor = auxController.GetPOV(0) == 270;
        if (blinkyBlinky != nullptr)
            blinkyBlinky->neuralyze = auxController.GetStartButton(); // Flash leds/signal light/limelight for Human Player attention acquisition

        hasBeenSetByAux = true;
        bool coralStationActive = true; // It's too late to come up with a better solution
        if (toStop)                   { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kSTOP);
        } else if (toTransit)         { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kTRANSIT);
        } else if (toL4)              { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kL4);
        } else if (toL3)              { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kL3);
        } else if (toL2)              { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kL2);
        } else if (toL1)              { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kL1);
        } else if (toReefLow)         { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kREEF_LOW); 
        } else if (toReefHigh)        { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kREEF_HIGH); 
        } else if (toProcessor)       { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kPROCESSOR);
        } else if (toGround)          { coralStationActive = false; gamepiece->moveToPreset(Gamepiece::Preset::kGROUND); 
        } else if (toCoralStation)    { coralStationActive = true;  gamepiece->moveToPreset(Gamepiece::Preset::kCORAL_STATION);
        } else if (toCoralStationLow) { coralStationActive = true;  gamepiece->moveToPreset(Gamepiece::Preset::kCORAL_STATION_LOW);
        } else {
            hasBeenSetByAux = false;
        }

        if (!coralStationActive)
            currentStationState = StationState::kNOT_ACTIVE;
    }

    // MARK: GamEpiece

    if (gamepiece != nullptr && driveController.IsConnected() && !hasBeenSetByAux && !driveDisable) {
        bool toNet = driveController.GetPOV() == 0;
        bool toFloor = driveController.GetLeftTriggerAxis() > PreferencesControls::AXIS_DEADZONE;

        if (toNet) 
            gamepiece->moveToPreset(Gamepiece::Preset::kNET);
        else if (toFloor)
            gamepiece->moveToPreset(Gamepiece::Preset::kFLOOR);
    }

    if (gamepiece->calgae != nullptr && auxController.IsConnected() && !auxDisable) {
        bool algaeIntake = fabs(auxController.GetRightTriggerAxis()) > PreferencesControls::AXIS_DEADZONE;
        bool shoot = auxController.GetRightBumperButton();
        bool resetGamepieceState = auxController.GetBackButtonPressed();
        bool shootDone = auxController.GetRightBumperButtonReleased();

        if (algaeIntake) {
            gamepiece->calgaeAutopilot = false;
            gamepiece->calgae->setMotorMode(Calgae::MotorModes::kALGAE_INTAKE);
        } else if (shoot) {
            gamepiece->calgaeAutopilot = false;
            #ifndef CALGAE_SENSOR_BROKEN 
                gamepiece->calgae->setMotorMode(Calgae::MotorModes::kSHOOT);
            #else 
                gamepiece->calgae->setMotorMode(Calgae::MotorModes::kSHOOT_OVERRIDE);
            #endif
        } else if (shootDone) {
            gamepiece->calgaeAutopilot = false;
            gamepiece->calgae->setMotorMode(Calgae::MotorModes::kDONE_SHOOTING);
        } else {
            if (!gamepiece->calgaeAutopilot) {
                gamepiece->calgae->setMotorMode(Calgae::MotorModes::kSTOP);
            }
        }

        if (resetGamepieceState) {
            gamepiece->calgaeAutopilot = false;
            gamepiece->calgae->resetHadGamepiece();
        }
    }

    // MARK: Hang

    if (hang != nullptr && auxController.IsConnected() && !auxDisable && /*!hangDisable &&*/ !manualMode) { // No hang in manual mode
        double hangPercent = -auxController.GetLeftY();
        double hangPercentFasty = -auxController.GetRightY();

        if (fabs(hangPercent) < PreferencesControls::AXIS_DEADZONE && fabs(hangPercentFasty) < PreferencesControls::AXIS_DEADZONE) {
            hangPercent = 0;
            hang->setControlMode(Hang::ControlMode::STOPPED);
            hang->fastyFast = false;
        } else {
            if (hangPercent > 0) {
                hang->setControlMode(Hang::ControlMode::GOING_UP);
            } else if (hangPercent < 0 || hangPercentFasty < 0) {
                if (hangPercentFasty < 0) {
                    hang->fastyFast = true;
                } else {
                    hang->fastyFast = false;
                }
                hang->setControlMode(Hang::ControlMode::GOING_DOWN);
            }
        }

    }

    // Elevator Manual Movement Code, re-implement if needed
    if (gamepiece->elevator && auxController.IsConnected() && manualMode && !auxDisable) {
        double movementPercent = -auxController.GetLeftY();

        if (fabs(movementPercent) < PreferencesControls::AXIS_DEADZONE)
            movementPercent = 0;
        gamepiece->elevator->manualMovement(movementPercent / 3);
    }

    
    // Wrist Manual Movement Code, re-implement if needed
    if (gamepiece->wrist != nullptr && auxController.IsConnected() && manualMode && !auxDisable) {
        double movementPercent = -auxController.GetRightY();

        if (fabs(movementPercent) < PreferencesControls::AXIS_DEADZONE)
            movementPercent = 0;
        gamepiece->wrist->manualMovement(units::degree_t(movementPercent));
    }
}

void Controls::sendFeedback() {
    Alert::sendDisconnectAndDisableStates(auxController.IsConnected(), driveController.IsConnected(), switchBoard.IsConnected());
    Alert::sendFeedback();

    frc::SmartDashboard::PutBoolean("Controls Manual Mode", manualMode);
    frc::SmartDashboard::PutBoolean("Controls Pit Mode", settings.pitMode);
    frc::SmartDashboard::PutBoolean("Controls Field Centric", fieldCentric);

    frc::SmartDashboard::PutBoolean("Controls last ST Station Preset", lastShouldToggleStationPreset);
    frc::SmartDashboard::PutBoolean("Controls cont ST Station Preset", controllerToggleStationPreset);

    frc::SmartDashboard::PutNumber("Controls current Station State", (int)currentStationState);

    frc::SmartDashboard::PutString ("Controls AutoAlign Horizontal", [&]() -> std::string {
        if (feedbackHorizontalAutoAlign == LineupHorizontal::kCENTER) return "Center";
        if (feedbackHorizontalAutoAlign == LineupHorizontal::kLEFT)   return "Left";
        if (feedbackHorizontalAutoAlign == LineupHorizontal::kRIGHT)  return "Right";
        return "Unknown";
    }());
    frc::SmartDashboard::PutBoolean("Controls AutoAlign in L4 mode",   isL4AutoAlign);

    frc::SmartDashboard::PutNumber("Match Time", frc::DriverStation::GetMatchTime().value());
}

void Controls::utilizeSwitchBoard() {
    if (!switchBoard.IsConnected()) {
        manualMode = false;
        return;
    }

    gamepiece->elevatorDisable = switchBoard.GetRawButton(1);
    gamepiece->wristDisable = switchBoard.GetRawButton(2);
    manualMode = switchBoard.GetRawButton(3);
    // hangDisable = switchBoard.GetRawButton(4);
    hangDisable = false;
    driveDisable = switchBoard.GetRawButton(5);
    auxDisable = switchBoard.GetRawButton(6);
    fieldCentric = switchBoard.GetRawButton(7);
    bool disableLimelight = switchBoard.GetRawButton(8);

    if (gamepiece->wrist != nullptr)
        gamepiece->wrist->setEncoderBroken(gamepiece->wristDisable);

    if (limelight != nullptr)
        limelight->setFunctioningState(!disableLimelight);

    if (switchBoard.GetRawButton(9)) { // LED Disable
        if (blinkyBlinky != nullptr) {
            blinkyBlinky->currentMode = BlinkyBlinky::Mode::OFF;
        }
    }

    EPDSLDisable = switchBoard.GetRawButton(10);
    settings.pitMode = switchBoard.GetRawButton(11);
}

bool Controls::shouldPersistentConfig() {
    utilizeSwitchBoard(); // This is run in disable, might want to rename function

    if (auxController.IsConnected()) {
        if (auxController.GetLeftBumperButton() &&
            auxController.GetRightBumperButton() &&
            auxController.GetStartButtonPressed()) {

            return true;
        }
    }

    return false;
}