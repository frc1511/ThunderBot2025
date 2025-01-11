#include "Controls.h"

Controls::Controls(Drive* drive_)
: drive(drive_)
{}

void Controls::process()
{
    double yPercent = driveController.GetLeftY()*.2;
    double xPercent = driveController.GetLeftX()*.2;
    double rotPercent = driveController.GetRightX()*.2;
    bool lockX = driveController.GetL2ButtonPressed();
    bool lockY = driveController.GetR2ButtonPressed();
    bool lockRot = driveController.GetR3ButtonPressed();
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
    drive->driveFromPercents(xPercent, yPercent, rotPercent, flags);
}
