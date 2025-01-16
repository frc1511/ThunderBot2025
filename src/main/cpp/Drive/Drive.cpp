#include "Drive/drive.h"

Drive::Drive():
driveController(
    [&]() -> frc::HolonomicDriveController {
        // Set the angular PID controller range from -180 to 180 degrees.
        trajectoryThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
        // Setup the drive controller with the individual axis PID controllers.
        return frc::HolonomicDriveController(xPIDController, yPIDController, trajectoryThetaPIDController);
    } ()) {
    manualThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));

    // Enable the trajectory drive controller.
    driveController.SetEnabled(true);
}

Drive::~Drive() 
{
    for (SwerveModule* module : swerveModules) {
       delete module;
    }
}

void Drive::resetToMode(/*MatchMode mode*/) {
    resetPIDControllers();

    driveMode = DriveMode::STOPPED;

    // Reset the manual control data.
    VelocityControlData lastControlData(controlData);
    controlData = { 0_mps, 0_mps, 0_rad_per_s, ControlFlag::NONE };

    trajectoryThetaPIDController.Reset(getRotation().Radians());

    // This seems to be necessary. Don't ask me why.
    for (SwerveModule* module : swerveModules) {
        module->stop();
    }

    // if (mode == MatchMode::DISABLED) {
    //     /**
    //      * Coast all motors in disabled (good for transportation, however can
    //      * lead to some runaway robots).
    //      */
    //     setIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    //     trajectoryTimer.Stop();
    // }
    // else {
    //     // Brake all motors when enabled to help counteract pushing.
    //     setIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        /**
         * Calibrate the IMU if not already calibrated. This will cause the
         * robot to pause for 4 seconds while it waits for it to calibrate, so
         * the IMU should always be calibrated before the match begins.
         */
        if (!isIMUCalibrated()) {
            calibrateIMU();
        }

        // Reset the trajectory timer.
    //     trajectoryTimer.Reset();

    //     if (mode == MatchMode::AUTO) {
    //         trajectoryTimer.Start();
    //     }
    // }

    // static bool wasAuto = false;

    // // Going from Auto to Disabled to Teleop.
    // if (wasAuto && mode == Mechanism::MatchMode::TELEOP) {
    //     wasAuto = false;
    //     frc::Pose2d currPose(getEstimatedPose());
    //     resetOdometry(frc::Pose2d(currPose.X(), currPose.Y(), currPose.Rotation().Degrees() + 90_deg + (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)));
    // }
    // else {
    //     // Check if going from Auto to Disabled.
    //     wasAuto = getLastMode() == Mechanism::MatchMode::AUTO && mode == Mechanism::MatchMode::DISABLED;

    //     // Doing something else.
    //     if (!wasAuto && mode != Mechanism::MatchMode::DISABLED) {
    //         // Stuff to reset normally.
    //     }
    // }
}

void Drive::process()
{
    updateOdometry();

    switch (driveMode) {
        case DriveMode::STOPPED:
            stop();
            break;
        case DriveMode::VELOCITY:
            executeVelocityData();
            break;
        case DriveMode::TRAJECTORY:
            //execTrajectory();
            break;
    }
}


void Drive::doPersistentConfiguration()
{
    for (SwerveModule* module : swerveModules) {
        module->doPersistentConfiguration();
    }
}

wpi::array<SwerveModule*,4>* Drive::getSwerveModules()
{
    return &swerveModules;
}
void Drive::driveFromPercents(double xPct, double yPct, double rotPct, unsigned flags)
{
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel    = xPct * DRIVE_PREFERENCES.DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel    = yPct * DRIVE_PREFERENCES.DRIVE_MANUAL_MAX_VEL;
    units::radians_per_second_t rotVel = rotPct * DRIVE_PREFERENCES.DRIVE_MANUAL_MAX_ANG_VEL;

    // Pass the velocities to the velocity control function.
    driveWithVelocities(xVel, yVel, rotVel, flags);
}

void Drive::driveWithVelocities(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t angVel, unsigned flags)
{
    units::meters_per_second_t newXVel    = xVel;
    units::meters_per_second_t newYVel    = yVel;
    units::radians_per_second_t newAngVel = angVel;
    
    // Apply the locked axis flags.
    if (flags & ControlFlag::LOCK_X) newXVel = 0_mps;
    if (flags & ControlFlag::LOCK_Y) newYVel = 0_mps;
    if (flags & ControlFlag::LOCK_ROT) newAngVel = 0_rad_per_s;

    // Stop the robot in brick mode no matter what.
    if (flags & ControlFlag::BRICK) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot isn't being told to do move, sooo.... stop??
    else if (!newXVel && !newYVel && !newAngVel) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot is being told to do stuff, so start doing stuff.
    else {
        driveMode = DriveMode::VELOCITY;
    }

    controlData = { newXVel, newYVel, newAngVel, flags };
}

void Drive::executeVelocityData()
{
    if (controlData.flags & ControlFlag::BRICK) {
        makeBrick();
        return;
    }

    frc::Pose2d currPose(getEstimatedPose());
    
    frc::ChassisSpeeds velocities;

    // Generate chassis speeds depending on the control mode.
    if (controlData.flags & ControlFlag::FIELD_CENTRIC) {
        // Generate chassis speeds based on the rotation of the robot relative to the field.
        velocities = frc::ChassisSpeeds::FromFieldRelativeSpeeds(controlData.xVel, controlData.yVel, controlData.angVel, currPose.Rotation());//whooshWhoosh->getHeadingAngle());// currPose.Rotation());
    }
    else {
        // Chassis speeds are robot-centric.
        velocities = { controlData.xVel, controlData.yVel, controlData.angVel };
    }

    // Set the modules to drive at the given velocities.
    setModuleStates(velocities);
}

void Drive::setModuleStates(frc::ChassisSpeeds speeds)
{

    /// NOTE: If auto is 90_deg off swap X and Y in chassis speeds here, its swaped in controls.

    // Store velocities for feedback.
    chassisSpeeds = speeds;

    // Generate individual module states using the chassis velocities.
    wpi::array<frc::SwerveModuleState, 4> moduleStates(kinematics.ToSwerveModuleStates(speeds));
    
    kinematics.DesaturateWheelSpeeds(&moduleStates, DRIVE_PREFERENCES.DRIVE_MANUAL_MAX_VEL);

    // Set the states of the individual modules.
    for(std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->setState(moduleStates.at(i));
    }
}

void Drive::stop()
{
    /// TODO: Re-implement
    // Set the speeds to 0.
    setModuleStates({ 0_mps, 0_mps, 0_deg_per_s });

    // Just for feedback.
    //targetPose = getEstimatedPose();

    // Put the drivetrain into brick mode if the flag is set.
    if (controlData.flags & ControlFlag::BRICK) {
        makeBrick();
    }
}

void Drive::sendDebugInfo()
{
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->sendDebugInfo(i);
    }
}

bool Drive::isFinished() const 
{
    return driveMode == DriveMode::STOPPED;
}

/// MARK: Field Centric

void Drive::calibrateIMU() {
    pigeon.Reset();
    
    imuCalibrated = true;

    resetOdometry();
}

bool Drive::isIMUCalibrated() {
    return imuCalibrated;
}

void Drive::resetOdometry(frc::Pose2d pose) {
    /**
     * Resets the position and rotation of the robot to a given pose
     * while ofsetting for the IMU's recorded rotation.
     */

    printf("Reset IMU\n");

    poseEstimator.ResetPosition(getRotation(), getModulePositions(), pose);

    pigeon.SetYaw(0_deg);

    for (SwerveModule* module : swerveModules) {
        module->zeroDriveEncoder();
    }
}


frc::Pose2d Drive::getEstimatedPose() {
    return poseEstimator.GetEstimatedPosition();
}

frc::Rotation2d Drive::getRotation() {
    // The raw rotation from the IMU.
    return frc::Rotation2d(-pigeon.GetYaw().GetValue() - 90_deg);
}

void Drive::resetPIDControllers() {
    xPIDController.Reset();
    yPIDController.Reset();

    frc::Pose2d currPose(getEstimatedPose());
    units::degree_t rotation(currPose.Rotation().Degrees());

    manualThetaPIDController.Reset(rotation);
    trajectoryThetaPIDController.Reset(rotation);

}


void Drive::updateOdometry() {
    /**
     * Update the pose estimator with encoder measurements from
     * the swerve modules.
     */
    poseEstimator.Update(getRotation(), getModulePositions());
}


wpi::array<frc::SwerveModuleState, 4> Drive::getModuleStates() {
    return { swerveModules.at(0)->getState(), swerveModules.at(1)->getState(),
             swerveModules.at(2)->getState(), swerveModules.at(3)->getState() };
}

wpi::array<frc::SwerveModulePosition, 4> Drive::getModulePositions() {
    return { swerveModules.at(0)->getPosition(), swerveModules.at(1)->getPosition(),
             swerveModules.at(2)->getPosition(), swerveModules.at(3)->getPosition() };
}


void Drive::makeBrick() {
    swerveModules.at(0)->setTurningMotor(-45_deg);
    swerveModules.at(1)->setTurningMotor(45_deg);
    swerveModules.at(2)->setTurningMotor(45_deg);
    swerveModules.at(3)->setTurningMotor(-45_deg);
}