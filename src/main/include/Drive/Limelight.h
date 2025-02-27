#pragma once

#include "LimelightHelpers.h"
#include "Preferences.h"

#include <frc/DriverStation.h>
#include <units/Time.h>

class Limelight{
  public:
    bool getEstimatedBotPose();

    void setFunctioningState(bool isFunctioning_);
  private:
    nullptr_t limelightMeasurement;
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance().value();
    
    bool isFunctioning = true;
};