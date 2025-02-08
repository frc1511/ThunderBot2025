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
    void doPersistentConfiguration();
    void sendFeedback();

    enum Preset {
        kSTOP,
        kGROUND,
        kPROCESSOR,
        kCORAL_STATION,
        kL1,
        kL2,
        kL3,
        kL4,
        _enum_MAX,
    };

    void moveToPreset(Preset preset);

    bool isAtPreset();

    bool hasGamepiece();

    bool calgaeAutopilot = false;
    bool wristAutopilot = false;
    bool elevatorAutopilot = false;
  private:
    std::string targetPresetAsString();

    Calgae* calgae;
    Wrist* wrist;
    Elevator* elevator;

    Preset targetPreset = Preset::kSTOP;
};