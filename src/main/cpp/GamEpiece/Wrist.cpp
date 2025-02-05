#include "GamEpiece/Wrist.h"

void Wrist::process() {
    units::degree_t degrees = getEncoderDegrees();
    double speed = PIDController.Calculate(degrees);
    setSpeed(speed);
}

void Wrist::doPersistentConfiguration() {
    PIDController.Reset(getEncoderDegrees());
    PIDController.DisableContinuousInput(); // No 360 rotate. :(
}

void Wrist::sendFeedback() {
    frc::SmartDashboard::PutNumber ("Wrist Encoder Raw",     getRawEncoder());
    frc::SmartDashboard::PutNumber ("Wrist Degrees",         getEncoderDegrees().value());
    frc::SmartDashboard::PutString ("Wrist Target Preset",   presetAsString());
    frc::SmartDashboard::PutNumber ("Wrist Target Position", Positions[currentPreset].value());
    frc::SmartDashboard::PutNumber ("Wrist Motor Speed",     motor.GetSpeed());
    frc::SmartDashboard::PutBoolean("Wrist At Goal",         PIDController.AtGoal());
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
    if (speed < 0 && getEncoderDegrees() < Positions[Preset::kLOWEST]) {
        speed = 0;
    }
    if (speed > 0 && getEncoderDegrees() > Positions[Preset::kHIGHEST]) {
        speed = 0;
    }
    motor.SetSpeed(speed);
}

std::string Wrist::presetAsString() {
    switch (currentPreset) {
    case Preset::kLOWEST:
        return "Lowest";
    case Preset::kSTATION:
        return "Station";
    case Preset::kTROUGH:
        return "Trough";
    case Preset::kBRANCH2_3:
        return "Branch 2&3";
    case Preset::kBRANCH4:
        return "Branch 4";
    case Preset::kHIGHEST:
        return "Highest";
    default:
       return "ERROR: Unknown/Incorrect";
    }
}