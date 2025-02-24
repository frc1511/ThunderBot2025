#include "Drive/Limelight.h"

#include <frc/smartdashboard/SmartDashboard.h>

std::pair<bool, LimelightHelpers::PoseEstimate> Limelight::getEstimatedBotPose() {
    if (!isFunctioning) {
        // If it's not functioning, don't use it's values!
        return std::make_pair(false, LimelightHelpers::PoseEstimate());
    }

    LimelightHelpers::PoseEstimate mt1 = LimelightHelpers::getBotPoseEstimate_wpiBlue(LIMELIGHT_PREFERENCE.LIMELIGHT_NAME);
    if (allianceColor == frc::DriverStation::Alliance::kRed) 
        LimelightHelpers::PoseEstimate mt1 = LimelightHelpers::getBotPoseEstimate_wpiRed(LIMELIGHT_PREFERENCE.LIMELIGHT_NAME);
    
    bool shouldUpdate = true;
    if (mt1.tagCount == 0)
        shouldUpdate = false;
    if (mt1.tagCount == 1 && mt1.rawFiducials.size() == 1) {
        if (mt1.rawFiducials[0].ambiguity > .8) {
            shouldUpdate = false;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) { // distToCamera is in meters
            shouldUpdate = false;
        }
    }

    for (auto fiducial : mt1.rawFiducials) {
        if (fiducial.ambiguity > .9) {
            shouldUpdate = false;
        }
    }
        
    return std::make_pair(shouldUpdate, mt1);
}

void Limelight::setFunctioningState(bool isFunctioning_) {
    isFunctioning = isFunctioning_;
}