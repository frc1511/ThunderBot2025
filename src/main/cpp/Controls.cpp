#include "Controls.h"

Controls::Controls(Drive* drive_, Gamepiece* gamepiece_, Calgae* calgae_, Wrist* wrist_, Elevator* elevator_, BlinkyBlinky* blinkyBlinky_):
    drive(drive_),
    calgae(calgae_),
    wrist(wrist_),
    elevator(elevator_),
    gamepiece(gamepiece_),
    blinkyBlinky(blinkyBlinky_)
{
    sendAlertsTimer.Start();
}

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
        bool persistentConfig = driveController.GetBButtonPressed();
        bool resetIMU = driveController.GetYButtonPressed();
        bool brickMode = driveController.GetAButton();
        if (persistentConfig) {
            drive->doPersistentConfiguration();
        }
        unsigned flags = 0;
        if (lockX)
            flags |= Drive::ControlFlag::LOCK_X;
        if (lockY)
            flags |= Drive::ControlFlag::LOCK_Y; 
        if (lockRot)
            flags |= Drive::ControlFlag::LOCK_ROT;
        if (brickMode)
            flags |= Drive::ControlFlag::BRICK;

        flags |= Drive::ControlFlag::FIELD_CENTRIC;

        if (resetIMU)
            drive->resetOdometry();

        // SWAP: 90_deg offset for drive
        float finalSpeedReduction = 1 - speedReduction;
        drive->driveFromPercents(yPercent * -finalSpeedReduction, xPercent * finalSpeedReduction, rotPercent * -finalSpeedReduction, flags);
    }

    // MARK: Aux
    if (gamepiece != nullptr && auxController.IsConnected()) {
        bool toGround = auxController.GetPOV() == 180;
        bool toProcessor = auxController.GetPOV(0) == 270;
        bool toCoralStation = auxController.GetPOV(0) == 90;
        bool toL1 = auxController.GetAButton();
        bool toL2 = auxController.GetBButton();
        bool toL3 = auxController.GetXButton();
        bool toL4 = auxController.GetYButton();
        bool toNet = auxController.GetPOV() == 0;
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
        } else if (toGround)       { gamepiece->moveToPreset(Gamepiece::Preset::kGROUND); }
    }

    #define CALGAE_SENSOR_BROKEN false// Replace with switchboard?
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
            if (!CALGAE_SENSOR_BROKEN) calgae->setMotorMode(Calgae::MotorModes::kSHOOT);
            else calgae->setMotorMode(Calgae::MotorModes::kSHOOT_OVERRIDE);
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
}

void Controls::sendFeedback() {
    static bool driveDisableAlertShown = false;
    static bool auxDisableAlertShown = false;
    static bool shouldShowDriveAlert = false;
    frc::SmartDashboard::PutNumber("Alert Timer", sendAlertsTimer.Get().value());
    if (!auxDisableAlertShown && (calgae == nullptr || !auxController.IsConnected())) {
        elastic::SendNotification(auxDisabledAlert);
        auxDisableAlertShown = true;
    }

    if (!driveDisableAlertShown && (drive == nullptr || !driveController.IsConnected())) {
        elastic::SendNotification(driveDisabledAlert);
        driveDisableAlertShown = true;
    }

    if (sendAlertsTimer.HasElapsed(0.8_s) && shouldShowDriveAlert) {
        driveDisableAlertShown = false;
        shouldShowDriveAlert = false;
    } else if (sendAlertsTimer.HasElapsed(1.6_s)) {
        auxDisableAlertShown = false; 
        shouldShowDriveAlert = true;
        sendAlertsTimer.Restart();
    }
}