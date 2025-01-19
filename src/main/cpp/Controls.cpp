#include "Controls.h"

Controls::Controls(Drive* drive_, Gamepiece* gamepiece_):
    drive(drive_),
    gamepiece(gamepiece_)
{}

#define SPEED_REDUCTION 1

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
        bool coralIntake = auxController.GetR2Button();
        bool algaeIntake = auxController.GetR2Button();
        bool shoot = auxController.GetR1Button();
        if (coralIntake) {
            gamepiece->setMotorMode(Gamepiece::MotorModes::kCORAL_INTAKE);
        } else if (algaeIntake) {
            gamepiece->setMotorMode(Gamepiece::MotorModes::kALGAE_INTAKE);
        } else if (shoot) {
            gamepiece->setMotorMode(Gamepiece::MotorModes::kSHOOT);
        } else {
            gamepiece->setMotorMode(Gamepiece::MotorModes::kNONE);
        }
    #endif
}
