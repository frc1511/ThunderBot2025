#pragma once

#include "Basic/Component.h"
#include "Elevator.h"
#include "Calgae.h"
#include "Wrist.h"

#include <frc/smartdashboard/SmartDashboard.h>

class Gamepiece : public Component {
  public:
    Gamepiece(Calgae* calgae_, Wrist* wrist_, Elevator* elevator_);

    void process();
    void doConfiguration(bool persist);
    void sendFeedback();

    enum Preset {
        kSTOP,
        kGROUND,
        kPROCESSOR,
        kTRANSIT,
        kL1,
        kL2,
        kREEF_LOW,
        kCORAL_STATION_LOW,
        kCORAL_STATION,
        kL3,
        kREEF_HIGH,
        kL4,
        kNET,
        _enum_MAX,
    };

    void moveToPreset(Preset preset);

    bool isAtPreset();

    bool hasGamepiece();

    bool calgaeAutopilot = false;
    bool wristAutopilot = false;
    bool elevatorAutopilot = false;

    Calgae* calgae;
    Wrist* wrist;
    Elevator* elevator;
    
    bool elevatorDisable = false;
    bool wristDisable = false;
  private:
    std::string targetPresetAsString();

    Preset targetPreset = Preset::kSTOP;

    void moveToTarget();

    bool isMovingDown = false;
    bool wristMoveDone = false;
    bool elevatorMoveDone = false;
};