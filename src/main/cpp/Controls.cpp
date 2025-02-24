#include "Controls.h"

Controls::Controls(Drive* drive_, Gamepiece* gamepiece_, BlinkyBlinky* blinkyBlinky_, Hang* hang_):
    drive(drive_),
    gamepiece(gamepiece_),
    blinkyBlinky(blinkyBlinky_),
    hang(hang_)
{}

void Controls::process() {
    utilizeSwitchBoard();
    // MARK: Drive
    if (drive != nullptr && driveController.IsConnected()) {
        // Drive limiting based on elevator position
        double speedReduction = 0.0;
        
        if (gamepiece->elevator != nullptr) {
            // Percentage from L3 -> NET
            double elevatorPercent = ((gamepiece->elevator->getPosition() - gamepiece->elevator->Position[Elevator::Preset::kL3]) / gamepiece->elevator->Position[Elevator::Preset::kNET]).value() - 1;

            if (elevatorPercent > 0) {
                elevatorPercent = pow(elevatorPercent, 2);
                elevatorPercent = std::clamp(elevatorPercent, 0.0, 1.0);
                speedReduction = elevatorPercent;
            }
        }

        // drive->setAccelerationReduction(accelerationReduction);

        double xPercent = driveController.GetLeftX();
        if (fabs(xPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
            xPercent = 0;
        double yPercent = driveController.GetLeftY();
        if (fabs(yPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
            yPercent = 0;
        double rotPercent = driveController.GetRightX();
        if (fabs(rotPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
            rotPercent = 0;
        bool lockX = fabs(driveController.GetLeftTriggerAxis()) > CONTROLS_PREFERENCE.AXIS_DEADZONE;
        bool lockY = fabs(driveController.GetRightTriggerAxis()) > CONTROLS_PREFERENCE.AXIS_DEADZONE;
        bool slowDrive = driveController.GetXButtonPressed();
        bool speedUpDrive = driveController.GetBButtonPressed();
        bool lockRot = driveController.GetRightStickButtonPressed();
        bool resetIMU = driveController.GetYButtonPressed();
        bool brickMode = driveController.GetAButton();
        unsigned flags = 0;
        if (lockX)
            flags |= Drive::ControlFlag::LOCK_X;
        if (lockY)
            flags |= Drive::ControlFlag::LOCK_Y; 
        if (lockRot)
            flags |= Drive::ControlFlag::LOCK_ROT;
        if (brickMode)
            flags |= Drive::ControlFlag::BRICK;

        // flags |= Drive::ControlFlag::FIELD_CENTRIC;

        if (resetIMU)
            drive->resetOdometry();

        if (slowDrive) {
            drive->slowYourRoll();
        }

        if (speedUpDrive) {
            drive->unslowYourRoll();
        }

        // SWAP: 90_deg offset for drive
        double finalSpeedReduction = 1; //1 - speedReduction

        if (fabs(driveController.GetLeftTriggerAxis()) > CONTROLS_PREFERENCE.AXIS_DEADZONE) {
            finalSpeedReduction -= driveController.GetLeftTriggerAxis() * 0.75;
        }

        drive->driveFromPercents(yPercent * finalSpeedReduction, xPercent * finalSpeedReduction, rotPercent * finalSpeedReduction, flags);
    }

    // MARK: Aux
    if (gamepiece != nullptr && auxController.IsConnected()) {
        bool hasBeenSetByAux = false;
        if (auxController.IsConnected()) { // TODO: Noticed this (and its also on each check below), is this if statement extraneous?
            bool toCoralStation = auxController.GetPOV(0) == 90;
            bool toCoralStationLow = auxController.GetPOV(0) == 270;
            bool toL1 = auxController.GetAButton();
            bool toL2 = auxController.GetBButton();
            bool toL3 = auxController.GetXButton();
            bool toL4 = auxController.GetYButton();
            bool toTransit = auxController.GetStartButtonPressed();
            if (blinkyBlinky != nullptr)
                blinkyBlinky->neuralyze = auxController.GetBackButton(); // Flash leds/signal light/limelight for Human Player attention acquisition

            hasBeenSetByAux = true;
            if (toL4)              { gamepiece->moveToPreset(Gamepiece::Preset::kL4);
            } else if (toL3)              { gamepiece->moveToPreset(Gamepiece::Preset::kL3);
            } else if (toL2)              { gamepiece->moveToPreset(Gamepiece::Preset::kL2);
            } else if (toL1)              { gamepiece->moveToPreset(Gamepiece::Preset::kL1);
            } else if (toCoralStation)    { gamepiece->moveToPreset(Gamepiece::Preset::kCORAL_STATION);
            } else if (toCoralStationLow) { gamepiece->moveToPreset(Gamepiece::Preset::kCORAL_STATION_LOW);
            } else if (toTransit)         { gamepiece->moveToPreset(Gamepiece::Preset::kTRANSIT); 
            } else {
                hasBeenSetByAux = false;
            }
        }
        if (driveController.IsConnected() && !hasBeenSetByAux) {
            bool toReefLow = driveController.GetLeftBumperButtonPressed();
            bool toReefHigh = driveController.GetRightBumperButtonPressed();
            bool toGround = driveController.GetPOV() == 180;
            bool toProcessor = driveController.GetPOV(0) == 270;
            bool toNet = driveController.GetPOV() == 0;

            // Prioritize Highest
            if (toReefLow)             { gamepiece->moveToPreset(Gamepiece::Preset::kREEF_LOW); 
            } else if (toReefHigh)     { gamepiece->moveToPreset(Gamepiece::Preset::kREEF_HIGH); 
            } else if (toProcessor)    { gamepiece->moveToPreset(Gamepiece::Preset::kPROCESSOR);
            } else if (toGround)       { gamepiece->moveToPreset(Gamepiece::Preset::kGROUND); 
            } else if (toNet)          { gamepiece->moveToPreset(Gamepiece::Preset::kNET);
            }
        }
    }

    // #define CALGAE_SENSOR_BROKEN false// Replace with switchboard?
    if (gamepiece->calgae != nullptr && auxController.IsConnected()) {
        bool coralIntake = fabs(auxController.GetRightTriggerAxis()) > CONTROLS_PREFERENCE.AXIS_DEADZONE; //fabs is extraneous but might as well 
        bool algaeIntake = fabs(auxController.GetLeftTriggerAxis()) > CONTROLS_PREFERENCE.AXIS_DEADZONE;
        bool shoot = auxController.GetRightBumperButton();
        bool resetGamepieceState = auxController.GetLeftBumperButtonPressed();
        bool shootDone = auxController.GetRightBumperButtonReleased();

        if (coralIntake) {
            gamepiece->calgaeAutopilot = false;
            gamepiece->calgae->setMotorMode(Calgae::MotorModes::kCORAL_INTAKE);
        } else if (algaeIntake) {
            gamepiece->calgaeAutopilot = false;
            gamepiece->calgae->setMotorMode(Calgae::MotorModes::kALGAE_INTAKE);
        } else if (shoot) {
            gamepiece->calgaeAutopilot = false;
            #ifndef CALGAE_SENSOR_BROKEN 
                gamepiece->calgae->setMotorMode(Calgae::MotorModes::kSHOOT);
            #else 
                calgae->setMotorMode(Calgae::MotorModes::kSHOOT_OVERRIDE);
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

    if (hang != nullptr && auxController.IsConnected()) {
        double hangPercent = auxController.GetLeftY();
        if (fabs(hangPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE) {
            hangPercent = 0;
            hang->setControlMode(Hang::ControlMode::STOPPED);
        }

        if (hangPercent > 0) {
            hang->setControlMode(Hang::ControlMode::GOING_UP);
        } else if (hangPercent < 0) {
            hang->setControlMode(Hang::ControlMode::GOING_DOWN);
        }
    }

    // Elevator Manual Movement Code, re-implement if needed
    if (gamepiece->elevator && auxController.IsConnected() && manualMode) {
        double movementPercent = -auxController.GetLeftY();
        if (fabs(movementPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
            movementPercent = 0;
        gamepiece->elevator->manualMovement(movementPercent / 3);
    }

    
    // Wrist Manual Movement Code, re-implement if needed
    if (gamepiece->wrist && auxController.IsConnected() && manualMode) {
        double movementPercent = -auxController.GetRightY();
        if (fabs(movementPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
            movementPercent = 0;
        gamepiece->wrist->manualMovement(units::degree_t(movementPercent / 3));
    }
}

void Controls::sendFeedback() {
    Alert::sendControllerDisableAndDisconnectedAlerts(auxController.IsConnected(), driveController.IsConnected());
    frc::SmartDashboard::PutBoolean("Controls Manual Mode", manualMode);
    
}

void Controls::utilizeSwitchBoard() {
    if (!switchBoard.IsConnected()) {
        printf("Switch Board Not Connected to port 2\n");
        manualMode = false;
        return;
    }

    // bool disableLimelight = switchBoard.GetRawButton(1);
    // if (limelight != nullptr)
    //     limelight->setFunctioningState(disableLimelight);

    // bool wristMotorBroken = switchBoard.GetRawButton(2);
    // if (gamepiece->wrist != nullptr)
    //     gamepiece->wrist->setEncoderBroken(wristMotorBroken);
    
    manualMode = switchBoard.GetRawButton(3);
}

bool Controls::shouldPersistentConfig() {
    // utilizeSwitchBoard(); // This is run in disable, might want to rename function
    if (auxController.IsConnected()) {
        if (auxController.GetLeftBumperButton() &&
            auxController.GetRightBumperButton() &&
            auxController.GetStartButtonPressed()) {
            return true;
        }
    }
    return false;
}