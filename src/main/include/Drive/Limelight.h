#pragma once

#include "LimelightHelpers.h"
#include "Preferences.h"

#include <frc/DriverStation.h>
#include <units/Time.h>

class Limelight{
  public:
    std::pair<bool, LimelightHelpers::PoseEstimate> getEstimatedBotPose();

    void setFunctioningState(bool isFunctioning_);
  private:
    LimelightHelpers::PoseEstimate limelightMeasurement;
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance().value();
    
    bool isFunctioning = true;
};