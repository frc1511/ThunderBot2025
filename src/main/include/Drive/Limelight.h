#pragma once

#include "LimelightHelpers.h"
#include "Preferences.h"

#include <frc/DriverStation.h>
#include <units/Time.h>
#include <optional>

class Limelight {
  public:
    std::optional<std::pair<bool, LimelightHelpers::PoseEstimate>> getEstimatedBotPose();

    bool isFunctioning = true;

    void setFunctioningState(bool isFunctioning_);
  private:
    LimelightHelpers::PoseEstimate limelightMeasurement;
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance().value();
};