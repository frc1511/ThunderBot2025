#pragma once

#include "LimelightHelpers.h"
#include "Preferences.h"

#include <frc/DriverStation.h>
#include <units/Time.h>
#include <optional>
#include <map>

class Limelight {
  public:
    // Exists, <Is Reliable, Measurement>[]
    std::optional<std::map<bool,LimelightHelpers::PoseEstimate>> getEstimatedBotPose();

    std::pair<bool,LimelightHelpers::PoseEstimate> getLimelightPose(std::string name);

    bool isFunctioning = true;

    void setFunctioningState(bool isFunctioning_);
  private:
    LimelightHelpers::PoseEstimate limelightMeasurement;
};