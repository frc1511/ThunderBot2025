#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PWM.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <units/angle.h>

class Wrist : public Component {
  public:
    Wrist();

    void process();

    void doPersistentConfiguration();

    void sendFeedback();

    enum Preset {
        kGROUND,
        kSTATION,
        kTROUGH,
        kBRANCH2_3,
        kBRANCH4,
        kPROCESSOR,
        _enum_MAX
    };

    void toPreset(Preset preset);

    bool atPreset();

    void setEncoderBroken(bool isBroken);

  private:
    Preset currentPreset = Preset::kGROUND;

    units::degree_t Positions[Preset::_enum_MAX] = {
        0_deg,   // Ground
        35_deg,  // Station
        35_deg,  // Trough
        55_deg,  // Branch 2 & 3
        75_deg,  // Branch 4
        0_deg,   // Processor
    };

    double getRawEncoder();

    units::degree_t getEncoderDegrees();

    void setTarget(Preset preset);

    void setSpeed(double speed);

    std::string presetAsString();

    frc::PWM motor {PWM_WRIST}; // The wrist motor

    frc::DutyCycleEncoder encoder {DIO_WRIST_ENCODER}; // The through bore encoder
    bool encoderBroken = false;

    frc::ProfiledPIDController<units::degrees> PIDController {
        WRIST_PREFERENCE.PID.Kp, WRIST_PREFERENCE.PID.Ki, WRIST_PREFERENCE.PID.Kd,
        frc::TrapezoidProfile<units::degrees>::Constraints(WRIST_PREFERENCE.PID.MaxVel, WRIST_PREFERENCE.PID.MaxAccel)
    };
};