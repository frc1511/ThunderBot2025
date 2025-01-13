#include "Drive/drive.h"

Drive::Drive() 
{
}

Drive::~Drive() 
{
    for (SwerveModule* module : swerveModules) {
       delete module;
    }
}

void Drive::process()
{
    //updateOdometry();

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

void Drive::driveFromPercents(double xPct, double yPct, double rotPct, unsigned flags)
{
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel    = xPct * DRIVE_PREFERENCES.DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel    = yPct * DRIVE_PREFERENCES.DRIVE_MANUAL_MAX_VEL;
    units::radians_per_second_t rotVel = rotPct * DRIVE_PREFERENCES.DRIVE_MANUAL_MAX_ANG_VEL;
    printf("EXEC | X: %lf, Y: %lf, OMEGA:%lf\n", xVel.value(), yVel.value(), rotVel.value());

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
    /// MARK: Field Centric Needed
    // frc::Pose2d currPose(getEstimatedPose());
    
    frc::ChassisSpeeds velocities;

    // Generate chassis speeds depending on the control mode.
    if (controlData.flags & ControlFlag::FIELD_CENTRIC) {
        // Generate chassis speeds based on the rotation of the robot relative to the field.
        // velocities = frc::ChassisSpeeds::FromFieldRelativeSpeeds(controlData.xVel, controlData.yVel, controlData.angVel, currPose.Rotation());//whooshWhoosh->getHeadingAngle());// currPose.Rotation());
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
        //makeBrick();
    }
}

void Drive::sendDebugInfo()
{
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->sendDebugInfo(i);
    }
}