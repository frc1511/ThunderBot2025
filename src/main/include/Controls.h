#pragma once

#include <frc/XboxController.h> // For Logitech Gamepad F310
#include <frc/GenericHID.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Basic/Component.h"
#include "Drive/Drive.h"
#include "GamEpiece/Calgae.h"
#include "GamEpiece/Wrist.h"
#include "Elevator.h"
#include "GamEpiece/Gamepiece.h"
#include "BlinkyBlinky.h"
#include "Hang.h"
#include "Libraries/elasticlib.h"
#include "Alerts.h"

class Controls : public Component {
  public:
    Controls(Drive* drive_, Gamepiece* gamepiece_, Calgae* calgae_, Wrist* wrist_, Elevator* elevator_, BlinkyBlinky* blinkyBlinky_, Hang* hang_);

    void process();
    void sendFeedback();
    void utilizeSwitchBoard();
    bool shouldPersistentConfig();
 private:

    Drive* drive;
    Calgae* calgae;
    Wrist* wrist;
    Elevator* elevator;
    Gamepiece* gamepiece;
    BlinkyBlinky* blinkyBlinky;
    Hang* hang;
    Limelight* limelight;

    frc::XboxController driveController {0};
    frc::XboxController auxController {1};
    frc::GenericHID switchBoard {2};
};