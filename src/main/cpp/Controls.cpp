#include "Controls.h"

Controls::Controls(Drive* drive_)
: drive(drive_)
{}

void Controls::process()
{
    double yPercent = driveController.GetRawAxis(1);
    yPercent = abs(yPercent) > CONTROLS_PREFERENCE.AXIS_DEADZONE ? yPercent : 0;
    double xPercent = driveController.GetRawAxis(0);
    xPercent = abs(xPercent) > CONTROLS_PREFERENCE.AXIS_DEADZONE ? xPercent : 0;
    double rotPercent = driveController.GetRawAxis(2);
    rotPercent = abs(rotPercent) > CONTROLS_PREFERENCE.AXIS_DEADZONE ? rotPercent : 0;
    // bool lockX = driveController.GetL2ButtonPressed();
    // bool lockY = driveController.GetR2ButtonPressed();
    // bool lockRot = driveController.GetR3ButtonPressed();
    // bool persistentConfig = driveController.GetCircleButtonPressed();
    // if (persistentConfig) {
    //     drive->doPersistentConfiguration();
    // }
    unsigned flags = 0;
    // if (lockX) {
    //     flags |= Drive::ControlFlag::LOCK_X;
    // }
    // if (lockY) {
    //     flags |= Drive::ControlFlag::LOCK_Y; 
    // }
    // if (lockRot) {
    //     flags |= Drive::ControlFlag::LOCK_ROT;
    // }
    
    drive->driveFromPercents(xPercent * .5, yPercent * .5, rotPercent * .5, flags);
}
