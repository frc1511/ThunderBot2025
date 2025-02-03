#pragma once

#include <frc/PS4Controller.h>
#include "Basic/Component.h"
#include "Drive/Drive.h"

class Controls : public Component {
  public:
    Controls(Drive* drive_);

    virtual void process();
private:
    Drive* drive;

    frc::PS4Controller driveController {0};
};