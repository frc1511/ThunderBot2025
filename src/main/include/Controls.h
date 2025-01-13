#pragma once

#include "Drive/Drive.h"
#include <frc/GenericHID.h>

class Controls
{
public:
    Controls(Drive* drive_);

    void process();
private:
    Drive* drive;

    frc::GenericHID driveController {0};
};