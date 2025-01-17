#pragma once

#include "LimelightHelpers.h"
#include "Basic/Component.h"

#include <frc/DriverStation.h>
#include <units/Time.h>

class Limelight : public Component {
  public:
    Limelight();
    ~Limelight();

    void process();
    void doPersistentConfiguration();
    void resetToMatchMode(MatchMode mode);
    void sendFeedback();

    LimelightHelpers::PoseEstimate getEstimatedBotPose();
  private:
    LimelightHelpers::PoseEstimate limelightMeasurement;
    frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance().value();
};