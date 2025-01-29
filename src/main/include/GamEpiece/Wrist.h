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
    void process();

    void doPersistentConfiguration();

    void sendFeedback();

  private:
    enum Preset {
        kLOWEST,
        kINTAKE,
        kSHOOT_CORAL,
        kSHOOT_ALGAE,
        kHIGHEST,
        kMAX
    };
    Preset currentPreset = Preset::kLOWEST;

    units::degree_t Positions[Preset::kMAX] = {
        0_deg,  // Lowest  position
        25_deg, // Intake  position
        40_deg, // Shoot Coral
        55_deg, // Shoot Algae
        100_deg // Highest position
    };

    double getRawEncoder();

    units::degree_t getEncoderDegrees();

    void setTarget(Preset preset);

    void setSpeed(double speed);

    std::string presetAsString();

    frc::PWM motor {PWM_WRIST}; // The wrist motor

    frc::DutyCycleEncoder encoder {DIO_WRIST_ENCODER}; // The through bore encoder

    frc::ProfiledPIDController<units::degrees> PIDController {
        WRIST_PREFERENCE.PID.Kp, WRIST_PREFERENCE.PID.Ki, WRIST_PREFERENCE.PID.Kd,
        frc::TrapezoidProfile<units::degrees>::Constraints(WRIST_PREFERENCE.PID.MaxVel, WRIST_PREFERENCE.PID.MaxAccel)
    };
};