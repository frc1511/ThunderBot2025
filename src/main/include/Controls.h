#pragma once

#include <frc/PS4Controller.h>

#include "Basic/Component.h"
#include "Drive/Drive.h"
#include "GamEpiece.h"

class Controls : public Component {
  public:
    Controls(Drive* drive_, Gamepiece* gamepiece_);

    void process();
 private:
    Drive* drive;
    Gamepiece* gamepiece;

    frc::PS4Controller driveController {0};
    frc::PS4Controller auxController {1};
};