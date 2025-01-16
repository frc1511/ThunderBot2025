#include <Basic/Component.h>

Component::MatchMode Component::getMatchMode() {
    if(frc::DriverStation::IsDisabled()) {
        return MatchMode::DISABLED;
    } else if(frc::DriverStation::IsTeleop()) {
        return MatchMode::TELEOP;
    } else if(frc::DriverStation::IsAutonomous()) {
        return MatchMode::AUTO;
    } else if(frc::DriverStation::IsTest()) {
        return MatchMode::TEST;
    } else {
        return MatchMode::DISABLED;
    }
}

Component::MatchMode Component::getLastMode() {
    return lastMode;
}

void Component::callResetToMode(MatchMode _lastMode) {
    lastMode = _lastMode;
    resetToMatchMode(getMatchMode());
}

Settings Component::settings {};