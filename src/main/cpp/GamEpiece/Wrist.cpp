#include "GamEpiece/Wrist.h"

Wrist::Wrist() {
    PIDController.Reset(getEncoderDegrees());
    PIDController.DisableContinuousInput(); // No 360 rotate. :(
    PIDController.SetTolerance(WRIST_PREFERENCE.ANGLE_TOLERANCE);

    encoder.SetAssumedFrequency(975.6_Hz); // From REV specs
    encoder.SetConnectedFrequencyThreshold(100); // NOTE: We should ensure that this one is correct
}

void Wrist::process() {
    // TODO: Build in controller override to use encoderBroken
    units::degree_t degrees = getEncoderDegrees();
    double speed = PIDController.Calculate(degrees);
    setSpeed(speed);
}

void Wrist::doPersistentConfiguration() {}

void Wrist::sendFeedback() {
    frc::SmartDashboard::PutNumber ("Wrist Encoder Raw",     getRawEncoder());
    frc::SmartDashboard::PutNumber ("Wrist Degrees",         getEncoderDegrees().value());
    frc::SmartDashboard::PutString ("Wrist Target Preset",   presetAsString());
    frc::SmartDashboard::PutNumber ("Wrist Target Position", Positions[currentPreset].value());
    frc::SmartDashboard::PutNumber ("Wrist Motor Speed",     motor.GetSpeed());
    frc::SmartDashboard::PutBoolean("Wrist At Goal",         PIDController.AtGoal());
}

void Wrist::toPreset(Wrist::Preset preset) {
    currentPreset = preset;
}

bool Wrist::atPreset() {
    return PIDController.AtSetpoint();
}

void Wrist::setEncoderBroken(bool isBroken) {
    if (encoder.IsConnected() && !isBroken) {
        encoderBroken = false;
    } else {
        encoderBroken = true;
    }
}

double Wrist::getRawEncoder() {
    return encoder.Get() * 360; // 0-1 -> 0-360
}

units::degree_t Wrist::getEncoderDegrees() {
    return units::degree_t(getRawEncoder());
}

void Wrist::setTarget(Preset preset) {
    currentPreset = preset;
    PIDController.SetGoal(Positions[preset]);
}

void Wrist::setSpeed(double speed) {
    speed = std::clamp(speed, -WRIST_PREFERENCE.MAX_SPEED, WRIST_PREFERENCE.MAX_SPEED);
    if (speed < 0 && getEncoderDegrees() < WRIST_PREFERENCE.LOWEST_ANGLE) {
        speed = 0;
    }
    if (speed > 0 && getEncoderDegrees() > WRIST_PREFERENCE.HIGHEST_ANGLE) {
        speed = 0;
    }
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
    default:
       return "ERROR: Unknown/Incorrect";
    }
}