#pragma once

#include <frc/PS4Controller.h>

#include "Basic/Component.h"
#include "Drive/Drive.h"
#include "GamEpiece/Calgae.h"
#include "GamEpiece/Wrist.h"
#include "GamEpiece/Gamepiece.h"

class Controls : public Component {
  public:
    Controls(Drive* drive_, Gamepiece* gamepiece_, Calgae* calgae_, Wrist* wrist_);

    void process();
 private:
    Drive* drive;
    Calgae* calgae;
    Wrist* wrist;
    Gamepiece* gamepiece;

    frc::PS4Controller driveController {0};
    frc::PS4Controller auxController {1};
};