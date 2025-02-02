#pragma once

#include <frc/PS4Controller.h>
#include <frc/GenericHID.h>

#include "Basic/Component.h"
#include "Drive/Drive.h"
#include "ControlMap.h"

class Controls : public Component {
  public:
    Controls(Drive* drive_);

    void process();

    void utilizeSwitchBoard();
private:
    Drive* drive;
    Limelight* limelight;

    frc::PS4Controller driveController {0};
    frc::GenericHID switchBoard {2};
};