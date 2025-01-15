#pragma once

#include "Drive/Drive.h"
#include <frc/PS4Controller.h>

class Controls
{
public:
    Controls(Drive* drive_);

    void process();
private:
    Drive* drive;

    frc::PS4Controller driveController {0};
};