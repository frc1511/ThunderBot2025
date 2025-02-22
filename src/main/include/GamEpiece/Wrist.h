#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PWM.h>
#include <frc/DutyCycleEncoder.h>
#include <units/angle.h>

class Wrist : public Component {
  public:
    Wrist();

    void process();

    void doConfiguration(bool persist);

    void sendFeedback();

    enum Preset {
        kGROUND,
        kSTATION,
        kTROUGH,
        kBRANCH2_3,
        kBRANCH4,
        kPROCESSOR,
        kTRANSIT,
        kREEF,
        _enum_MAX
    };

    bool withinEncoderSafeZone();

    void toPreset(Preset preset);

    bool atPreset();

    void setEncoderBroken(bool isBroken);

    void manualMovement(double speed);

    bool wristIsUnsafe();

  private:
    Preset currentPreset = Preset::kTRANSIT;

    units::degree_t Positions[Preset::_enum_MAX] = {
         63_deg,  // Ground
        -37_deg,  // Station, Max back angle
        -19_deg,  // Trough
        -19_deg,  // Branch 2 & 3
         100_deg, // Branch 4
         25_deg,  // Processor
        -37_deg,  // Transit
        -35_deg,  // Reef
    };

    double getRawEncoder();

    units::degree_t getEncoderDegrees();

    void setTarget(Preset preset);

    double feedForwardPower();

    void setSpeed(double speed);

    std::string presetAsString();

    bool manual = false;
    double manualSpeed = 0.0;
    units::degree_t startPosition = 0_deg;

    frc::PWM motor {PWM_WRIST}; // The wrist motor

    frc::DutyCycleEncoder encoder {DIO_WRIST_ENCODER}; // The through bore encoder
    bool encoderBroken = false;
};