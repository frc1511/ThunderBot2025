#include "Controls.h"

Controls::Controls(Drive* drive_)
: drive(drive_)
{}

void Controls::process()
{
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
    if (persistentConfig) {
        drive->doPersistentConfiguration();
    }
    unsigned flags = 0;
    if (lockX) {
        flags |= Drive::ControlFlag::LOCK_X;
    }
    if (lockY) {
        flags |= Drive::ControlFlag::LOCK_Y; 
    }
    if (lockRot) {
        flags |= Drive::ControlFlag::LOCK_ROT;
    }
    
    // SWAP: 90_deg offset for drive
    drive->driveFromPercents(yPercent * -1, xPercent * 1, rotPercent * -1, flags);
}
