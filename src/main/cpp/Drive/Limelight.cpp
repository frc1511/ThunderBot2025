#include "Drive/Limelight.h"

Limelight::Limelight() {

}

Limelight::~Limelight() {
}

void Limelight::process() {

}

void Limelight::doPersistentConfiguration() {

}

void Limelight::resetToMatchMode(MatchMode mode) {

}

void Limelight::sendFeedback() {

}

LimelightHelpers::PoseEstimate Limelight::getEstimatedBotPose() {
    if (allianceColor == frc::DriverStation::Alliance::kRed) {
        LimelightHelpers::PoseEstimate limelightMeasurement = LimelightHelpers::getBotPoseEstimate_wpiRed("limelight");
    } else {
        LimelightHelpers::PoseEstimate limelightMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight");
    }
    return limelightMeasurement;
}