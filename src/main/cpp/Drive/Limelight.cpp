#include "Drive/Limelight.h"

#include <frc/smartdashboard/SmartDashboard.h>

std::optional<std::map<bool, LimelightHelpers::PoseEstimate>> Limelight::getEstimatedBotPose() {
    if (!isFunctioning) {
        // If it's not functioning, don't use it's values!
        return std::nullopt;
    }

    std::map<bool, LimelightHelpers::PoseEstimate> poses = {
        getLimelightPose(PreferencesLimelight::LIMELIGHT_FRONT),
        getLimelightPose(PreferencesLimelight::LIMELIGHT_BACK),
    };

    return poses;
}

std::pair<bool,LimelightHelpers::PoseEstimate> Limelight::getLimelightPose(std::string name) {
    LimelightHelpers::PoseEstimate mt1 = LimelightHelpers::getBotPoseEstimate_wpiBlue(name);
    if (allianceColor == frc::DriverStation::Alliance::kRed) 
        LimelightHelpers::PoseEstimate mt1 = LimelightHelpers::getBotPoseEstimate_wpiRed(name);

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