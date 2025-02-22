#include "Controls.h"

Controls::Controls(Drive* drive_, Gamepiece* gamepiece_, Calgae* calgae_, Wrist* wrist_, Elevator* elevator_, BlinkyBlinky* blinkyBlinky_, Hang* hang_):
    drive(drive_),
    calgae(calgae_),
    wrist(wrist_),
    elevator(elevator_),
    gamepiece(gamepiece_),
    blinkyBlinky(blinkyBlinky_),
    hang(hang_)
{}
// #define TMP
void Controls::process() {
    // MARK: Drive
    if (drive != nullptr && driveController.IsConnected()) {
        // Drive limiting based on elevator position
        double speedReduction = 0.0;
        
        if (elevator != nullptr) {
            // Percentage from L3 -> NET
            double elevatorPercent = ((elevator->getPosition() - elevator->Position[Elevator::Preset::kL3]) / elevator->Position[Elevator::Preset::kNET]).value() - 1;

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

        // SWAP: 90_deg offset for drive
        double finalSpeedReduction = 0.3; //1 - speedReduction
        #ifdef TMP
        static double r = 0.5;
        if (driveController.GetPOV() == 0) {
            r += 0.05;
            printf("Drive: %lf\n", r);
        } else if (driveController.GetPOV() == 180) {
            r -= 0.05;
            printf("Drive: %lf\n", r);
        }
        finalSpeedReduction = r;
        #endif
        drive->driveFromPercents(yPercent * finalSpeedReduction, xPercent * finalSpeedReduction, rotPercent * finalSpeedReduction, flags);
    }

    // MARK: Aux
    if (gamepiece != nullptr && auxController.IsConnected()) {
        #ifndef TMP
        bool toGround = auxController.GetPOV() == 180;
        bool toProcessor = auxController.GetPOV(0) == 270;
        bool toCoralStation = auxController.GetPOV(0) == 90;
        bool toL1 = auxController.GetAButton();
        bool toL2 = auxController.GetBButton();
        bool toL3 = auxController.GetXButton();
        bool toL4 = auxController.GetYButton();
        bool toNet = auxController.GetPOV() == 0;
        bool toTransit = auxController.GetStartButton();
        if (blinkyBlinky != nullptr)
            blinkyBlinky->neuralyze = auxController.GetBackButton(); // Flash leds/signal light/limelight for Human Player attention acquisition

        // Prioritize Highest
        if (toNet)                 { gamepiece->moveToPreset(Gamepiece::Preset::kNET);
        } else if (toL4)           { gamepiece->moveToPreset(Gamepiece::Preset::kL4);
        } else if (toL3)           { gamepiece->moveToPreset(Gamepiece::Preset::kL3);
        } else if (toL2)           { gamepiece->moveToPreset(Gamepiece::Preset::kL2);
        } else if (toL1)           { gamepiece->moveToPreset(Gamepiece::Preset::kL1);
        } else if (toCoralStation) { gamepiece->moveToPreset(Gamepiece::Preset::kCORAL_STATION);
        } else if (toProcessor)    { gamepiece->moveToPreset(Gamepiece::Preset::kPROCESSOR);
        } else if (toGround)       { gamepiece->moveToPreset(Gamepiece::Preset::kGROUND); 
        } else if (toTransit)      { gamepiece->moveToPreset(Gamepiece::Preset::kTRANSIT); }
        #endif
    }

    // #define CALGAE_SENSOR_BROKEN false// Replace with switchboard?
    if (calgae != nullptr && auxController.IsConnected()) {
        bool coralIntake = fabs(auxController.GetRightTriggerAxis()) > CONTROLS_PREFERENCE.AXIS_DEADZONE; //fabs is extraneous but might as well 
        bool algaeIntake = fabs(auxController.GetLeftTriggerAxis()) > CONTROLS_PREFERENCE.AXIS_DEADZONE;
        bool shoot = auxController.GetRightBumperButton();
        bool resetHadGamepiece = auxController.GetLeftBumperButtonPressed();
        bool shootDone = auxController.GetRightBumperButtonReleased();

        if (coralIntake) {
            gamepiece->calgaeAutopilot = false;
            calgae->setMotorMode(Calgae::MotorModes::kCORAL_INTAKE);
        } else if (algaeIntake) {
            gamepiece->calgaeAutopilot = false;
            calgae->setMotorMode(Calgae::MotorModes::kALGAE_INTAKE);
        } else if (shoot) {
            gamepiece->calgaeAutopilot = false;
            #ifndef CALGAE_SENSOR_BROKEN 
                calgae->setMotorMode(Calgae::MotorModes::kSHOOT);
            #else
                calgae->setMotorMode(Calgae::MotorModes::kSHOOT_OVERRIDE);
            #endif
        } else if (shootDone) {
            gamepiece->calgaeAutopilot = false;
            calgae->setMotorMode(Calgae::MotorModes::kDONE_SHOOTING);
        } else {
            if (!gamepiece->calgaeAutopilot) {
                calgae->setMotorMode(Calgae::MotorModes::kSTOP);
            }
        }
        if (resetHadGamepiece) {
            gamepiece->calgaeAutopilot = false;
            calgae->resetHadGamepiece();
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
    // if (gamepiece->elevator && auxController.IsConnected()) {
    //     double movementPercent = -auxController.GetLeftY();
    //     if (fabs(movementPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
    //         movementPercent = 0;
    //     #ifdef TMP
    //     static double er = 0.5;
    //     if (auxController.GetPOV() == 0) {
    //         er += 0.05;
    //         printf("Elevator: %lf\n", er);
    //     } else if (auxController.GetPOV() == 180) {
    //         er -= 0.05;
    //         printf("Elevator: %lf\n", er);
    //     }
    //     movementPercent *= er;
    //     #endif
    //     gamepiece->elevator->manualMovement(movementPercent);
    // }

    
    // Wrist Manual Movement Code, re-implement if needed
    // if (gamepiece->wrist && auxController.IsConnected()) {
    //     double movementPercent = -auxController.GetRightY();
    //     if (fabs(movementPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
    //         movementPercent = 0;
    //     #ifdef TMP
    //     static double wr = 0.5;
    //     if (auxController.GetPOV() == 90) {
    //         wr += 0.05;
    //         printf("Wrist: %lf\n", wr);
    //     } else if (auxController.GetPOV() == 270) {
    //         wr -= 0.05;
    //         printf("Wrist: %lf\n", wr);
    //     }
    //     movementPercent *= wr;
    //     #endif
    //     gamepiece->wrist->manualMovement(movementPercent);
    // }
}

void Controls::sendFeedback() {
    Alert::sendControllerDisableAndDisconnectedAlerts(auxController.IsConnected(), driveController.IsConnected());
}

void Controls::utilizeSwitchBoard() {
    if (!switchBoard.IsConnected()) {
        printf("Switch Board Not Connected to port 2\n");
        return;
    }

    bool disableLimelight = switchBoard.GetRawButton(1);
    limelight->setFunctioningState(disableLimelight);

    bool wristMotorBroken = switchBoard.GetRawButton(2);
    wrist->setEncoderBroken(wristMotorBroken);
}

bool Controls::shouldPersistentConfig() {
    if (auxController.IsConnected()) {
        if (auxController.GetLeftBumperButton() &&
            auxController.GetRightBumperButton() &&
            auxController.GetStartButtonPressed()) {
            return true;
        }
    }
    return false;
}