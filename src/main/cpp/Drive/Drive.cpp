#include "Drive/drive.h"

Drive::Drive(Limelight* _limelight):
driveController(
    [&]() -> frc::HolonomicDriveController {
        // Set the angular PID controller range from -180 to 180 degrees.
        trajectoryThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
        // Setup the drive controller with the individual axis PID controllers.
        return frc::HolonomicDriveController(xPIDController, yPIDController, trajectoryThetaPIDController);
    } ()),
limelight(_limelight),
driveLineupController(
    [&]() -> frc::HolonomicDriveController {
        // Set the angular PID controller range from -180 to 180 degrees.
        lineupThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
        // Setup the drive controller with the individual axis PID controllers.
        return frc::HolonomicDriveController(lineupXPIDController, lineupYPIDController, lineupThetaPIDController);
    } ())
{
    manualThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));

    // Enable the trajectory/lineup drive controller.
    driveController.SetEnabled(true);
    driveLineupController.SetEnabled(true);

    //Initialize the field widget
    //// frc::SmartDashboard::PutData("Field", &m_field);

    //Initialize the orchestra
    for (SwerveModule* module : swerveModules) {
        swerveOrchestra.AddInstrument(module->driveMotor);
        swerveOrchestra.AddInstrument(module->turningMotor);
    }
}

Drive::~Drive() {
    for (SwerveModule* module : swerveModules) {
       delete module;
    }
}

void Drive::resetToMatchMode(MatchMode priorMode, MatchMode mode) {
    resetPIDControllers();

    driveMode = DriveMode::STOPPED;

    // Reset the manual control data.
    VelocityControlData lastControlData(controlData);
    controlData = { 0_mps, 0_mps, 0_rad_per_s, ControlFlag::NONE };

    trajectoryThetaPIDController.Reset(getRotation().Radians());
    lineupThetaPIDController.Reset(getRotation().Radians());

    // This seems to be necessary. Don't ask me why.
    for (SwerveModule* module : swerveModules) {
        module->stop();
    }

    if (mode == MatchMode::DISABLED) {
        /**
         * Coast all motors in disabled (good for transportation, however can
         * lead to some runaway robots).
         */
        ////setIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        trajectoryTimer.Stop();
    } else {
        // Brake all motors when enabled to help counteract pushing.
        ////setIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        /**
         * Calibrate the IMU if not already calibrated. This will cause the
         * robot to pause for 4 seconds while it waits for it to calibrate, so
         * the IMU should always be calibrated before the match begins.
         */
        if (!isIMUCalibrated()) {
            calibrateIMU();
        }

        // Reset the trajectory timer.
        trajectoryTimer.Reset();

        if (mode == MatchMode::AUTO) {
            trajectoryTimer.Start();
        }
    }

    if (mode == MatchMode::AUTO) {
        poseEstimator.SetVisionMeasurementStdDevs({.1, .1, .7});
        isAuto = true;
    } else {
        poseEstimator.SetVisionMeasurementStdDevs({.4, .4, .9});
        isAuto = false;
    }

    static bool wasAuto = false;

    // Going from Auto to Disabled to Teleop.
    if (wasAuto && mode == Component::MatchMode::TELEOP) {
        wasAuto = false;
        frc::Pose2d currPose(getEstimatedPose());
        // resetOdometry(frc::Pose2d(currPose.X(), currPose.Y(), currPose.Rotation().Degrees() + 90_deg + (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)));
    } else {
        // Check if going from Auto to Disabled.
        wasAuto = priorMode == Component::MatchMode::AUTO && mode == Component::MatchMode::DISABLED;

        // Doing something else.
        if (!wasAuto && mode != Component::MatchMode::DISABLED) {
            // Stuff to reset normally.
        }
    }
}

// MARK: Process

void Drive::process() {
    updateOdometry();

    switch (driveMode) {
        case DriveMode::STOPPED:
            stop();
            break;
        case DriveMode::VELOCITY:
            executeVelocityData();
            break;
        case DriveMode::TRAJECTORY:
            execTrajectory();
            break;
        case DriveMode::LINEUP:
            execLineup();
            break;
        case DriveMode::ORCHESTRA:
            orchestrate("homedepot.chrp");
            break;
    }
}

void Drive::doConfiguration(bool persist) {
    for (SwerveModule* module : swerveModules) {
        module->doConfiguration(persist);
    }
}

void Drive::driveFromPercents(double xPct, double yPct, double rotPct, unsigned flags) {
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel    = xPct * PreferencesDrive::DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel    = yPct * PreferencesDrive::DRIVE_MANUAL_MAX_VEL;
    units::radians_per_second_t rotVel = rotPct * PreferencesDrive::DRIVE_MANUAL_MAX_ANG_VEL;

    // Pass the velocities to the velocity control function.
    driveWithVelocities(xVel, yVel, rotVel, flags);
}

void Drive::driveWithVelocities(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t angVel, unsigned flags) {
    units::meters_per_second_t newXVel    = xVel;
    units::meters_per_second_t newYVel    = yVel;
    units::radians_per_second_t newAngVel = angVel;

    // Apply the locked axis flags.
    if (flags & ControlFlag::LOCK_X) newXVel = 0_mps;
    if (flags & ControlFlag::LOCK_Y) newYVel = 0_mps;
    if (flags & ControlFlag::LOCK_ROT) newAngVel = 0_rad_per_s;

    newXVel   *= speedLimiting;
    newYVel   *= speedLimiting;
    //TODO: Change this up a bit (ask Ben D)
    newAngVel *= speedLimiting;

    // Stop the robot in brick mode no matter what.
    if (flags & ControlFlag::BRICK) {
        driveMode = DriveMode::STOPPED;
    } else if (!newXVel && !newYVel && !newAngVel) { // The robot isn't being told to do move, sooo.... stop??
        driveMode = DriveMode::STOPPED;
    } else { // The robot is being told to do stuff, so start doing stuff.
        driveMode = DriveMode::VELOCITY;
    }

    controlData = { newXVel, newYVel, newAngVel, flags };
}

void Drive::executeVelocityData() {
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
    } else {
        // Chassis speeds are robot-centric.
        velocities = { controlData.xVel, controlData.yVel, controlData.angVel };
    }

    // Set the modules to drive at the given velocities.
    setModuleStates(velocities);
}

void Drive::setModuleStates(frc::ChassisSpeeds speeds) {
    /// NOTE: If auto is 90_deg off swap X and Y in chassis speeds here, its swaped in controls.

    // Store velocities for feedback.
    chassisSpeeds = speeds;

    // Generate individual module states using the chassis velocities.
    wpi::array<frc::SwerveModuleState, 4> moduleStates(kinematics.ToSwerveModuleStates(speeds));

    kinematics.DesaturateWheelSpeeds(&moduleStates, PreferencesDrive::DRIVE_MANUAL_MAX_VEL);

    // Set the states of the individual modules.
    for(std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->setState(moduleStates.at(i));
    }
}

void Drive::stop() {
    // Set the speeds to 0.
    setModuleStates({ 0_mps, 0_mps, 0_deg_per_s });

    // Just for feedback.
    targetPose = getEstimatedPose();

    // Stop Orchestrating
    unOrchestrate();

    // Put the drivetrain into brick mode if the flag is set.
    if (controlData.flags & ControlFlag::BRICK) {
        makeBrick();
    }
}

// MARK: Feedback

void Drive::sendFeedback() {
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->sendDebugInfo(i);
    }

    frc::Pose2d pose(getEstimatedPose());

    static auto driveXController = driveController.GetXController();
    static auto driveYController = driveController.GetYController();
    static auto driveThetaController = driveController.GetThetaController();

    frc::SmartDashboard::PutData("Drive X PID Controller", &driveXController);
    frc::SmartDashboard::PutData("Drive Y PID Controller", &driveYController);
    frc::SmartDashboard::PutData("Drive Theta PID Controller", &driveThetaController);

    swerveFeedback.robotRotation = getRotation();
    frc::SmartDashboard::PutData("Swerve_Feedback", &swerveFeedback);

    feedbackField.SetRobotPose(pose);
    frc::SmartDashboard::PutData("Field", &feedbackField);
    frc::SmartDashboard::PutData("debug_TrajectoryTargetField", &trajectoryField);
    frc::SmartDashboard::PutData("debug_lineupField", &lineupField);

    frc::SmartDashboard::PutBoolean("Limelight_Reliable", limelightReliable);

    // Drive feedback.
    frc::SmartDashboard::PutNumber("Drive_SpeedLimiting",           speedLimiting);
    frc::SmartDashboard::PutNumber("Drive_PoseX_m",                 pose.X().value());
    frc::SmartDashboard::PutNumber("Drive_PoseY_m",                 pose.Y().value());
    frc::SmartDashboard::PutNumber("Drive_PoseRot_deg",             getRotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelX_mps",         controlData.xVel.value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelY_mps",         controlData.yVel.value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelRot_rad_per_s", controlData.angVel.value());
    frc::SmartDashboard::PutBoolean("Drive_FieldCentric",           controlData.flags & ControlFlag::FIELD_CENTRIC);
    frc::SmartDashboard::PutBoolean("Drive_Brick",                  controlData.flags & ControlFlag::BRICK);
    frc::SmartDashboard::PutBoolean("Drive_LockX",                  controlData.flags & ControlFlag::LOCK_X);
    frc::SmartDashboard::PutBoolean("Drive_LockY",                  controlData.flags & ControlFlag::LOCK_Y);
    frc::SmartDashboard::PutBoolean("Drive_LockRot",                controlData.flags & ControlFlag::LOCK_ROT);
    frc::SmartDashboard::PutNumber("Drive_Mode_Enum_Index",         (int)driveMode);

    // Nyoom

    lastQuadrant = currentQuadrant;
    currentQuadrant = getCurrentQuadrant();
    frc::SmartDashboard::PutString("Drive Current Quadrant", getCurrentQuadrantAsString());

    bool playNyoom = false;
    if (lastQuadrant != currentQuadrant && currentQuadrant != Quadrant::kNONE && lastQuadrant != Quadrant::kNONE) {
        playNyoom = true;
    }

    frc::SmartDashboard::PutBoolean("Drive_Play_Nyoom", playNyoom);
}

bool Drive::isFinished() const { // Can this const be moved to the beginning of the line? I think it would be easier to read
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
    return frc::Rotation2d(pigeon.GetYaw().GetValue());
}

void Drive::resetPIDControllers() {
    xPIDController.Reset();
    yPIDController.Reset();

    frc::Pose2d currPose(getEstimatedPose());

    lineupXPIDController.Reset();
    lineupYPIDController.Reset();

    units::degree_t rotation(currPose.Rotation().Degrees());

    manualThetaPIDController.Reset(rotation);
    trajectoryThetaPIDController.Reset(rotation);
    lineupThetaPIDController.Reset(rotation);
}

void Drive::updateOdometry() {
    /**
     * Update the pose estimator with encoder measurements from
     * the swerve modules.
     */
    poseEstimator.Update(getRotation(), getModulePositions());

    LimelightHelpers::SetRobotOrientation(PreferencesLimelight::LIMELIGHT_FRONT, pigeon.GetYaw().GetValue().value(),
                                                                                 pigeon.GetAngularVelocityZWorld().GetValue().value(),
                                                                                 pigeon.GetPitch().GetValue().value(),
                                                                                 pigeon.GetAngularVelocityYWorld().GetValue().value(),
                                                                                 pigeon.GetRoll().GetValue().value(),
                                                                                 pigeon.GetAngularVelocityXWorld().GetValue().value());
    LimelightHelpers::SetRobotOrientation(PreferencesLimelight::LIMELIGHT_BACK, pigeon.GetYaw().GetValue().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
    auto estimatedBotPose = limelight->getEstimatedBotPose();
    if (estimatedBotPose) {
        for (auto pose : *estimatedBotPose) {
            limelightReliable = pose.first;
            LimelightHelpers::PoseEstimate mt1 = pose.second;

            if (!limelightReliable) continue;
            if (distToLineupPose() < PreferencesDrive::LINEUP_LIMELIGHT_DEADZONE) {
                continue;
            }
            poseEstimator.AddVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds
            );
        }
    }
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
    swerveModules.at(0)->setTurningMotor(45_deg);
    swerveModules.at(1)->setTurningMotor(-45_deg);
    swerveModules.at(2)->setTurningMotor(45_deg);
    swerveModules.at(3)->setTurningMotor(-45_deg);
}


/// MARK: Trajectory


void Drive::runTrajectory(const CSVTrajectory* _trajectory, const std::map<u_int32_t, Action*>& actionMap) {
    driveMode = DriveMode::TRAJECTORY;

    // Set the trajectory.
    trajectory = _trajectory;

    // Set the initial action.
    trajectoryActionIter = trajectory->getActions().cbegin();

    trajectoryActions = &actionMap;

    // Reset done trajectory actions.
    doneTrajectoryActions.clear();

    // Reset the trajectory timer.
    trajectoryTimer.Reset();
    trajectoryTimer.Start();
}

void Drive::setupInitialTrajectoryPosition(const CSVTrajectory *trajectory) {
    frc::Pose2d initPose(trajectory->getInitialPose());
    resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees()));
}

void Drive::setAccelerationReduction(double reduction) {
    for (SwerveModule* module : swerveModules) {
        module->setAccelerationReduction(reduction);
    }
}

void Drive::execTrajectory() {
    units::second_t time(trajectoryTimer.Get());

    actionExecuting = false;
    bool actionExecuted = false;

    // If we've got another action to go.
    if (trajectoryActionIter != trajectory->getActions().cend()) {
        const auto& [action_time, actions] = *trajectoryActionIter;
        // Check if it's time to execute the action.
        if (time >= action_time) {
            actionExecuted = true;
            // Loop through the available actions.
            for (auto it(trajectoryActions->cbegin()); it != trajectoryActions->cend(); ++it) {
                const auto& [id, action] = *it;
                // Narrow the list down to only actions that have not been completed yet.
                if (std::find(doneTrajectoryActions.cbegin(), doneTrajectoryActions.cend(), id) == doneTrajectoryActions.cend()) {
                    // If the action's bit is set in the bit field.
                    if (actions & id) {
                        // Execute the action.
                        Action::Result res = action->process();
                        // If the action has completed.
                        if (res == Action::Result::DONE || res == Action::Result::DONE_BUT_LINEUP_STILL_NEEDS_TO_HAPPEN) {
                            if (res == Action::Result::DONE_BUT_LINEUP_STILL_NEEDS_TO_HAPPEN) {
                                actionExecutingButLineup = true;
                            }
                            // Remember that it's done.
                            doneTrajectoryActions.push_back(id);
                        }

                        actionExecuting = res == Action::Result::WORKING;

                        if (actionExecuting)
                            break;
                    }
                }
            }
        }
    }
    // Stop the trajectory because an action is still running.
    if (actionExecuting) {
        trajectoryTimer.Stop();
    }
    // Continue/Resume the trajectory because the actions are done.
    else {
        // Increment the action if an action was just finished.
        if (actionExecuted) {
            ++trajectoryActionIter;
            doneTrajectoryActions.clear();
        }
        trajectoryTimer.Start();
        lineUpDone = false;
    }

    // If the trajectory is done, then stop it.
    if (time > trajectory->getDuration() && !actionExecuting) {//// && driveController.AtReference()) { 
        driveMode = DriveMode::STOPPED;
        printf("Done\n");
        return;
    }

    // Sample the trajectory at the current time for the desired state of the robot.
    CSVTrajectory::State state(trajectory->sample(time));

    // Don't be moving if an action is being worked on.
    if (actionExecuting) {
        if (actionExecutingButLineup) {
            state.pose = lineupPose;
        }
        state.velocity = 0_mps;
    } else {
        actionExecutingButLineup = false;
    }

    // We should *NEVER* be moving to 0, 0
    if (state.pose.X() != 0_m && state.pose.Y() != 0_m) {
        driveToState(state, false);
    }
}

void Drive::driveToState(CSVTrajectory::State state, bool isLineup) {

    // Adjust the rotation because everything about this robot is 90 degrees off D:
    //// state.pose = frc::Pose2d(state.pose.Translation(), frc::Rotation2d(state.pose.Rotation() - 90_deg));

    // The current pose of the robot.
    frc::Pose2d currentPose(getEstimatedPose());

    // The desired change in position.
    frc::Twist2d twist(currentPose.Log(state.pose));

    // The angle at which the robot should be driving at.
    frc::Rotation2d heading;
    if (auto ally = frc::DriverStation::GetAlliance()) {
        if (ally == frc::DriverStation::Alliance::kRed) {
            heading = frc::Rotation2d(twist.dtheta + 180_deg);
        } else {
            heading = frc::Rotation2d(twist.dtheta);
        }
    }

    //// printf("Speed: %lf, X: %lf, Y: %lf, stateRot: %lf, heading: %lf\n", state.velocity.value(), state.pose.X().value(), state.pose.Y().value(), state.pose.Rotation().Degrees().value(), heading.Degrees().value());
    //// printf("(Pose) X: %lf, Y: %lf, Rot: %lf\n", currentPose.X().value(), currentPose.Y().value(), currentPose.Rotation().Degrees().value());

    /**
     * Calculate the chassis velocities based on the error between the current
     * pose and the desired pose.
     */

    frc::ChassisSpeeds trajectoryVelocities(
        driveController.Calculate(
            currentPose,
            frc::Pose2d(state.pose.X(), state.pose.Y(), heading),
            state.velocity,
            state.pose.Rotation()
        )
    );
    frc::ChassisSpeeds lineupVelocities(
        driveLineupController.Calculate(
            currentPose,
            frc::Pose2d(state.pose.X(), state.pose.Y(), heading),
            state.velocity,
            state.pose.Rotation()
        )
    );

    frc::ChassisSpeeds velocities = trajectoryVelocities; 
    if (isLineup)
        velocities = lineupVelocities;

    trajectoryField.SetRobotPose(state.pose);

    frc::SmartDashboard::PutNumber("debug_driveHeading_deg", heading.Degrees().value());
    frc::SmartDashboard::PutNumber("debug_driveTwist_thetadeg", units::degree_t(twist.dtheta).value());
    frc::SmartDashboard::PutNumber("debug_driveXVel", velocities.vx.value());
    frc::SmartDashboard::PutNumber("debug_driveYVel", velocities.vy.value());
    frc::SmartDashboard::PutNumber("debug_driveOMEGAVel", velocities.omega.value());
    // Keep target pose for feedback.
    targetPose = state.pose;

    velocities.vx    *= speedLimiting;
    velocities.vy    *= speedLimiting;
    velocities.omega *= speedLimiting;

    // Make the robot go vroom :D
    setModuleStates(velocities);
} 

void Drive::slowYourRoll() {
    speedLimiting = std::clamp(speedLimiting - .1, 0.0, 1.0);
}

void Drive::unslowYourRoll() {
    speedLimiting = std::clamp(speedLimiting + .1, 0.0, 1.0);
}

SwerveFeedback::SwerveFeedback(wpi::array<SwerveModule*, 4>* _swerveModules):
 swerveModules(_swerveModules) {
}

void SwerveFeedback::InitSendable(wpi::SendableBuilder &builder) {
    builder.SetSmartDashboardType("SwerveDrive");

    builder.AddDoubleProperty("Front Left Angle", [this] () {
        return swerveModules->at(0)->getState().angle.Radians().value();
    }, [this] (double _) {} );
    builder.AddDoubleProperty("Front Left Velocity", [this] () {
        return swerveModules->at(0)->getState().speed.value();
    }, [this] (double _) {});

    builder.AddDoubleProperty("Front Right Angle", [this] () {
        return swerveModules->at(3)->getState().angle.Radians().value();
    }, [this] (double _) {});
    builder.AddDoubleProperty("Front Right Velocity", [this] () {
        return swerveModules->at(3)->getState().speed.value();
    }, [this] (double _) {});

    builder.AddDoubleProperty("Back Left Angle", [this] () {
        return swerveModules->at(1)->getState().angle.Radians().value();
    }, [this] (double _) {});
    builder.AddDoubleProperty("Back Left Velocity",  [this] () {
        return swerveModules->at(1)->getState().speed.value();
    }, [this] (double _) {});

    builder.AddDoubleProperty("Back Right Angle", [this] () {
        return swerveModules->at(2)->getState().angle.Radians().value();
    }, [this] (double _) {});
    builder.AddDoubleProperty("Back Right Velocity",  [this] () {
        return swerveModules->at(2)->getState().speed.value();
    }, [this] (double _) {});

    builder.AddDoubleProperty("Robot Angle", [this] () {
        return robotRotation.Radians().value();
    }, [this] (double _) {});
}

// MARK: Lineup

frc::Pose2d Drive::calculateFinalLineupPose(int posId, LineupHorizontal lineupHorizontal, bool isL4) {
    //------------------------- Rotate around reef center
    units::radian_t rotRads = units::radian_t(posId * (1.0/6.0) * std::numbers::pi * 2);
    /// Make the reef the origin
    units::meter_t reefRelX = PreferencesDrive::MASTER_LINEUP_POSE.X() - PreferencesDrive::REEF_POSE.X();
    units::meter_t reefRelY = PreferencesDrive::MASTER_LINEUP_POSE.Y() - PreferencesDrive::REEF_POSE.Y();
    /// Rotate the pose
    units::meter_t reefRelXPrime = (reefRelX * cosf((double)rotRads)) - (reefRelY * sinf((double)rotRads));
    units::meter_t reefRelYPrime = (reefRelX * sinf((double)rotRads)) + (reefRelY * sinf((double)rotRads));
    units::radian_t newRot = rotRads; // Assuming that the masterLineupPosition is at 0_rad
    /// Reset to the field origin
    units::meter_t rotatedX = reefRelXPrime + PreferencesDrive::REEF_POSE.X();
    units::meter_t rotatedY = reefRelYPrime + PreferencesDrive::REEF_POSE.Y();

    //------------------------- Translate for branch
    units::meter_t deltaX = 0_m;
    units::meter_t deltaY = 0_m;
    if (lineupHorizontal != LineupHorizontal::kCENTER) {
        units::meter_t moveMagnitude = PreferencesDrive::HORIZONTAL_REEF_MOVE;
        if (lineupHorizontal == LineupHorizontal::kRIGHT) {
            moveMagnitude *= -1; // Move + for left, - for right
        }

        deltaX = cosf((double)(newRot + 90_deg)) * moveMagnitude;
        deltaY = sinf((double)(newRot + 90_deg)) * moveMagnitude;
    }


    // Add to the rotated X&Y the move we need to do
    units::meter_t finalX = rotatedX + deltaX;
    units::meter_t finalY = rotatedY + deltaY;

    //------------------------- Translate for L4
    // Move back for L4
    if (isL4) {
        units::meter_t moveBackMagnitude = PreferencesDrive::VERTICAL_REEF_MOVE;
        units::meter_t deltaX = -cosf(double(newRot)) * moveBackMagnitude;
        units::meter_t deltaY = -sinf(double(newRot)) * moveBackMagnitude;

        // Add to the rotated X&Y the move we need to do
        finalX += deltaX;
        finalY += deltaY;
    }

    frc::Rotation2d finalRot = frc::Rotation2d(newRot);

    //------------------------- Flip over field for Red
    if (auto ally = frc::DriverStation::GetAlliance()) {
        if (ally == frc::DriverStation::Alliance::kRed) {
            
            finalX = PreferencesTrajectory::FIELD_X - finalX;
            finalY = PreferencesTrajectory::FIELD_Y - finalY;
            finalRot = frc::Rotation2d(finalRot.Degrees() + 180_deg);
        }
    }

    return frc::Pose2d(finalX, finalY, finalRot);
}

double dist(frc::Pose2d p1, frc::Pose2d p2) {
    // sqrt((X2 - X1)^2 + (Y2 - Y1)^2)
    return std::sqrt(powf(double(p2.X() - p1.X()), 2) + powf(double(p2.Y() - p1.Y()), 2));
}

void Drive::beginLineup(LineupHorizontal lineupHorizontal, bool L4) {
    driveMode = DriveMode::LINEUP;
    lineUpDone = false;

    frc::Pose2d currentPose(getEstimatedPose());

    frc::Pose2d closestPose = calculateFinalLineupPose(0, lineupHorizontal, L4);
    double lowestDist = fabs(dist(currentPose, closestPose));

    for (int i = 1; i < 6; i++) {
        frc::Pose2d currentLineupPose = calculateFinalLineupPose(i, lineupHorizontal, L4);

        double currentLineupDist = fabs(dist(currentPose, currentLineupPose));

        if (currentLineupDist < lowestDist) {
            closestPose = currentLineupPose;
            lowestDist = currentLineupDist;
        }
    }
    frc::SmartDashboard::PutNumber("Lineup lowest distance", lowestDist);

    lineupPose = closestPose;

    lineupField.SetRobotPose(lineupPose);
}


void Drive::execLineup() {
    CSVTrajectory::State targetState = {};

    targetState.pose = lineupPose;

    targetState.velocity = 0_mps; // Don't be moving if we are at the goal (in theory)

    //// double acceleration = 1;
    //// double maxVel = 2;

    //// double startRamp = acceleration * dist(lineupStartPose, currentPose);
    //// double endRamp = acceleration * dist(targetState.pose, currentPose);

    //// double finalVelocity = std::clamp(startRamp, 0.0, maxVel) - std::clamp(endRamp, 0.0, maxVel);
    //// targetState.velocity = (units::meters_per_second_t)std::clamp(finalVelocity, 0.0, maxVel); 

    driveToState(targetState, true);

    if (distToLineupPose() < PreferencesDrive::LINEUP_POSE_TOLERANCE) {
        lineUpDone = true;
        if (isAuto) {
            driveMode = DriveMode::TRAJECTORY;
        } else {
            driveMode = DriveMode::STOPPED;
        }
    }
}

double Drive::distToLineupPose() {
    return std::fabs(dist(getEstimatedPose(), lineupPose));
}

bool Drive::isLineUpDone() {
    return lineUpDone;
}

// MARK: Nyooooooooooom

Drive::Quadrant Drive::getCurrentQuadrant() {
    auto ally = frc::DriverStation::GetAlliance();
    frc::Pose2d pose(getEstimatedPose());
    units::meter_t robotX = pose.X();
    units::meter_t robotY = pose.Y();

    if (ally) {
        if (ally.value() == frc::DriverStation::Alliance::kRed) {
            robotX = PreferencesTrajectory::FIELD_X - robotX;
            robotY = PreferencesTrajectory::FIELD_Y - robotY;
        }

        if (robotX > PreferencesDrive::QUADRANT_LEFT[0].x &&
            robotY > PreferencesDrive::QUADRANT_LEFT[0].y &&
            robotX < PreferencesDrive::QUADRANT_LEFT[1].x &&
            robotY < PreferencesDrive::QUADRANT_LEFT[1].y
        ) { // Left side of DS
            return Quadrant::kLEFT;
        } else if (robotX > PreferencesDrive::QUADRANT_RIGHT[0].x &&
                robotY > PreferencesDrive::QUADRANT_RIGHT[0].y &&
                robotX < PreferencesDrive::QUADRANT_RIGHT[1].x &&
                robotY < PreferencesDrive::QUADRANT_RIGHT[1].y
        ) { // Right side of DS
            return Quadrant::kRIGHT;
        } else { // Too far away
            return Quadrant::kNONE;
        }
    } else {
        return Quadrant::kNONE;
    }
}

std::string Drive::getCurrentQuadrantAsString() {
    switch (currentQuadrant) {
        case Quadrant::kNONE:  return "None";
        case Quadrant::kLEFT:  return "Left";
        case Quadrant::kRIGHT: return "Right";
        default: return "Invalid Quadrant";
    }
}

// MARK: Orchestra

void Drive::orchestrate(std::string fp) {
    auto status = swerveOrchestra.LoadMusic(((std::string)"/home/lvuser/deploy/"+fp).c_str());
    if (status.IsOK() && !isPlaying) {
        isPlaying = true;
        swerveOrchestra.Play();
    }

}

void Drive::unOrchestrate() {
    if (isPlaying) {
        isPlaying = false;
        swerveOrchestra.Stop();
    }
}