#include "Controls.h"

Controls::Controls(Drive* drive_, Gamepiece* gamepiece_, Elevator* elevator_):
    drive(drive_),
    gamepiece(gamepiece_),
    elevator(elevator_)
{}

#define SPEED_REDUCTION 1

void Controls::sendFeedback() {
}

void Controls::process() {
    
    // MARK: Drive
    #ifndef DRIVE_DISABLED
        double xPercent = driveController.GetLeftX();
        if (fabs(xPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
            xPercent = 0;
        double yPercent = driveController.GetLeftY();
        if (fabs(yPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
            yPercent = 0;
        double rotPercent = driveController.GetRightX();
        if (fabs(rotPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
            rotPercent = 0;
        bool lockX = driveController.GetL2ButtonPressed();
        bool lockY = driveController.GetR2ButtonPressed();
        bool lockRot = driveController.GetR3ButtonPressed();
        bool persistentConfig = driveController.GetCircleButtonPressed();
        bool resetIMU = driveController.GetTriangleButtonPressed();
        bool brickMode = driveController.GetCrossButton();
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
        drive->driveFromPercents(yPercent * -SPEED_REDUCTION, xPercent * SPEED_REDUCTION, rotPercent * -SPEED_REDUCTION, flags);
    #endif

    // MARK: Aux
    #ifndef AUX_DISABLED
        // bool coralIntake = auxController.GetR1Button();
        // bool coralShoot = auxController.GetR2Button();
        // bool algaeIntake = auxController.GetL1Button();
        // bool algaeShoot = auxController.GetL2Button();
        bool elevatorUp = auxController.GetPOV(0) > 0;
        bool elevatorDown = auxController.GetPOV(180) > 0;
        bool elevatorMove = auxController.GetPOV(90) > 0;

        // if (coralIntake) {
        //     gamepiece->setMotorMode(Gamepiece::MotorModes::kCORAL_INTAKE);
        // } else if (coralShoot) {
        //     gamepiece->setMotorMode(Gamepiece::MotorModes::kCORAL_SHOOT);
        // } else if (algaeIntake) {
        //     gamepiece->setMotorMode(Gamepiece::MotorModes::kALGAE_INTAKE);
        // } else if (algaeShoot) {
        //     gamepiece->setMotorMode(Gamepiece::MotorModes::kALGAE_SHOOT);
        // } else {
        //     gamepiece->setMotorMode(Gamepiece::MotorModes::kNONE);
        // }

        if (elevatorUp) {
            elevator->incrementPositionIndex();
        } else if (elevatorDown) {
            elevator->decrementPositionIndex();
        }
        if (elevatorMove) {
            elevator->updateCurrentPreset();
        }
    #endif
}
