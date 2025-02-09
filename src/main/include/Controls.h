#pragma once

#include <frc/XboxController.h> // For Logitech Gamepad F310

#include "Basic/Component.h"
#include "Drive/Drive.h"
#include "GamEpiece/Calgae.h"
#include "GamEpiece/Wrist.h"
#include "Elevator.h"
#include "GamEpiece/Gamepiece.h"

class Controls : public Component {
  public:
    Controls(Drive* drive_, Gamepiece* gamepiece_, Calgae* calgae_, Wrist* wrist_, Elevator* elevator_);

    void process();
 private:
    float speedReduction = 0;

    Drive* drive;
    Calgae* calgae;
    Wrist* wrist;
    Elevator* elevator;
    Gamepiece* gamepiece;

    frc::XboxController driveController {0};
    frc::XboxController auxController {1};
};