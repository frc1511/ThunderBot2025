#pragma once

#include <frc/XboxController.h> // For Logitech Gamepad F310
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Basic/Component.h"
#include "Drive/Drive.h"
#include "GamEpiece/Calgae.h"
#include "GamEpiece/Wrist.h"
#include "Elevator.h"
#include "GamEpiece/Gamepiece.h"
#include "Libraries/elasticlib.h"
#include "BlinkyBlinky.h"

class Controls : public Component {
  public:
    Controls(Drive* drive_, Gamepiece* gamepiece_, Calgae* calgae_, Wrist* wrist_, Elevator* elevator_, BlinkyBlinky* blinkyBlinky_);

    void process();
    void sendFeedback();
 private:

    Drive* drive;
    Calgae* calgae;
    Wrist* wrist;
    Elevator* elevator;
    Gamepiece* gamepiece;
    BlinkyBlinky* blinkyBlinky;

    elastic::Notification driveDisabledAlert = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Drive Disabled",
                                                .description = "Drive Disabled or Drive Controller Disconnected",
                                                .displayTime = 1.5_s,};
    elastic::Notification auxDisabledAlert   = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Aux Disabled",
                                                .description = "Aux Disabled or Aux Controller Disconnected",
                                                .displayTime = 1.5_s,};

    frc::Timer sendAlertsTimer {}; 

    frc::XboxController driveController {0};
    frc::XboxController auxController {1};
};