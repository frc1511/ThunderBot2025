#include "GamEpiece/Wrist.h"

Wrist::Wrist() {
    doConfiguration(false);
    encoder.SetAssumedFrequency(975.6_Hz); // From REV specs
    encoder.SetConnectedFrequencyThreshold(100); // NOTE: We should ensure that this one is correct
}

void Wrist::process() {
    if (!encoder.IsConnected()) {
        encoderBroken = true;
    }
    if (!encoderBroken && withinEncoderSafeZone()) {
        units::degree_t targetPosition = Positions[currentPreset];
        if (!manual) {
            if (atPreset()) {
                setSpeed(0);
                return;
            }
        } else {
            targetPosition = manualAngle;
        }
        units::degree_t difference = targetPosition - getEncoderDegrees();

        double speedFactor = std::clamp(difference.value() * 0.1, -1.0, 1.0);
        // speedFactor *= std::clamp(fabs(difference.value()) * 0.05, 0.3, 1.0);

        setSpeed(WRIST_PREFERENCE.MAX_SPEED * speedFactor);
    } else {
        setSpeed(0);
    }
}

void Wrist::doConfiguration(bool persist) {}

void Wrist::sendFeedback() {
    frc::SmartDashboard::PutBoolean("Wrist Manual",          manual);
    frc::SmartDashboard::PutNumber ("Wrist Manual Angle",    manualAngle.value());
    frc::SmartDashboard::PutNumber ("Wrist Encoder Raw",     getRawEncoder());
    frc::SmartDashboard::PutNumber ("Wrist Degrees",         getEncoderDegrees().value());
    frc::SmartDashboard::PutString ("Wrist Target Preset",   presetAsString());
    frc::SmartDashboard::PutNumber ("Wrist Target Position", Positions[currentPreset].value());
    frc::SmartDashboard::PutNumber ("Wrist Motor Speed",     motor.GetSpeed());
    frc::SmartDashboard::PutBoolean("Wrist At Goal",         atPreset());
}

bool Wrist::withinEncoderSafeZone() {
    if (getEncoderDegrees() < WRIST_PREFERENCE.LOWEST_ANGLE - WRIST_PREFERENCE.ENCODER_FAILURE_OUTBOUND) {
        printf("WRIST ENCODER FAILURE\n");
        return false;
    }
    if (getEncoderDegrees() > WRIST_PREFERENCE.HIGHEST_ANGLE + WRIST_PREFERENCE.ENCODER_FAILURE_OUTBOUND) {
        printf("WRIST ENCODER FAILURE\n");
        return false;
    }
    return true;
}

void Wrist::toPreset(Wrist::Preset preset) {
    currentPreset = preset;
    startPosition = getEncoderDegrees();
    manualAngle = 0_deg;
    manual = false;
}

bool Wrist::atPreset() {
    units::degree_t targetPosition = Positions[currentPreset];
    units::degree_t difference = targetPosition - getEncoderDegrees();

    return fabs(difference.value()) < WRIST_PREFERENCE.ANGLE_TOLERANCE.value();
}

void Wrist::setEncoderBroken(bool isBroken) {
    if (encoder.IsConnected() && !isBroken) {
        encoderBroken = false;
    } else {
        encoderBroken = true;
    }
}

void Wrist::manualMovement(units::degree_t angle) {
    manual = true;
    manualAngle = std::clamp(manualAngle + angle, WRIST_PREFERENCE.LOWEST_ANGLE, WRIST_PREFERENCE.HIGHEST_ANGLE);
}

bool Wrist::wristIsUnsafe() {
    if (getEncoderDegrees() > WRIST_PREFERENCE.UNSAFE_MIN && !encoderBroken)
        return true;
    return false;
}

double Wrist::getRawEncoder() {
    return encoder.Get(); 
}

units::degree_t Wrist::getEncoderDegrees() {
    return units::degree_t(getRawEncoder() * 360) - WRIST_PREFERENCE.UP_ZERO; // 0-1 -> 0-360
}

double Wrist::feedForwardPower() {
    double radFromUp = getEncoderDegrees().value() * (std::numbers::pi / 180);
    frc::SmartDashboard::PutNumber("Wrist Rad From Up", radFromUp);

    if (getEncoderDegrees() > 45_deg) {
        return sin(radFromUp) * -WRIST_PREFERENCE.MAX_FEED_FORWARD_POWER_HIGH_ANGLE;
    }

    return sin(radFromUp) * -WRIST_PREFERENCE.MAX_FEED_FORWARD_POWER_LOW_ANGLE;
}

void Wrist::setSpeed(double speed) {
    speed = std::clamp(speed, -WRIST_PREFERENCE.MAX_SPEED, WRIST_PREFERENCE.MAX_SPEED);
    if (speed < 0 && getEncoderDegrees() < WRIST_PREFERENCE.LOWEST_ANGLE) {
        speed = 0;
    }
    if (speed > 0 && getEncoderDegrees() > WRIST_PREFERENCE.HIGHEST_ANGLE) {
        speed = 0;
    }
    speed += feedForwardPower();
    frc::SmartDashboard::PutNumber("Wrist Speed", speed);
    motor.SetSpeed(speed);
}

std::string Wrist::presetAsString() {
    switch (currentPreset) {
    case Preset::kGROUND:
        return "Ground";
    case Preset::kSTATION:
        return "Station";
    case Preset::kTROUGH:
        return "Trough";
    case Preset::kBRANCH2_3:
        return "Branch 2&3";
    case Preset::kBRANCH4:
        return "Branch 4";
    case Preset::kPROCESSOR:
        return "Processor";
    case Preset::kTRANSIT:
        return "Transit";
    default:
       return "ERROR: Unknown/Incorrect";
    }
}