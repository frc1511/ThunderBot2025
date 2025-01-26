#include "Controls.h"

Controls::Controls(Drive* drive_)
: drive(drive_)
{}

#define SPEED_REDUCTION .5

void Controls::process() {
    #if 0
    wpi::array<SwerveModule*, 4>*  modules = drive->getSwerveModules();
    frc::SwerveModuleState state;
    state.speed = 0.5_mps;
    state.angle = 90_deg;

    for (SwerveModule* swerMod : *modules){
        swerMod->setState(state);
    }
    #else
    double controllerXPercent = driveController.GetLeftX();
    if (fabs(controllerXPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
        controllerXPercent = 0;
    double controllerYPercent = driveController.GetLeftY();
    if (fabs(controllerYPercent) < CONTROLS_PREFERENCE.AXIS_DEADZONE)
        controllerYPercent = 0;
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

    double robotXPercent = -controllerYPercent * SPEED_REDUCTION; // positive -> forward
    double robotYPercent = -controllerXPercent * SPEED_REDUCTION; // positive -> left
    double robotRotPercent = -rotPercent * SPEED_REDUCTION; // positive -> counter-clockwise

    // SWAP: 90_deg offset for drive
    drive->driveFromPercents(robotXPercent, robotYPercent, robotRotPercent, flags);
    #endif

}
