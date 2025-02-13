#include "Drive/Limelight.h"

#include <frc/smartdashboard/SmartDashboard.h>

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
    LimelightHelpers::PoseEstimate limelightMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue();
    return limelightMeasurement;
}