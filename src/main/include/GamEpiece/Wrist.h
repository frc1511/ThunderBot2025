#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PWM.h>
#include <frc/DutyCycleEncoder.h>
#include <units/angle.h>

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

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
        kCORAL_STATION_LOW,
        _enum_MAX
    };

    bool withinEncoderSafeZone();

    void toPreset(Preset preset);

    bool atPreset();

    void setEncoderBroken(bool isBroken);

    void manualMovement(units::degree_t speed);

    bool wristIsUnsafe();

  private:
    Preset currentPreset = Preset::kTRANSIT;

    units::degree_t Positions[Preset::_enum_MAX] = {
         63_deg,   // Ground
        -35_deg,   // Coral Station
        -19_deg,   // Trough
        -0_deg,    // Branch 2 & 3
         53.1_deg, // Branch 4
         25_deg,   // Processor
        -37_deg,   // Transit
         0_deg,    // Reef
        -37_deg,   // Coral Station Low
    };

    double getRawEncoder();

    units::degree_t getEncoderDegrees();

    double feedForwardPower();

    void setSpeed(double speed);

    std::string presetAsString();

    bool manual = false;

    units::degree_t manualAngle = 0_deg;
    units::degree_t startPosition = 0_deg;

    frc::PWM motor {PWM_WRIST}; // The wrist motor

    frc::DutyCycleEncoder encoder {DIO_WRIST_ENCODER}; // The through bore encoder
    bool encoderBroken = false;

    friend class Gamepiece;
};