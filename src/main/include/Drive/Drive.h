#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/math.h>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include "iomap.h"
#include "preferences.h"
#include "swerveModule.h"

class Drive {
public:
    Drive();
    ~Drive();

    void process();

    enum ControlFlag {
        NONE          = 0,
        FIELD_CENTRIC = 1 << 0, // Field-relative control (forward is always field-forward).
        BRICK         = 1 << 1, // All modules pointed towards the center.
        LOCK_X        = 1 << 3, // Lock X-axis drivetrain movement.
        LOCK_Y        = 1 << 4, // Lock Y-axis drivetrain movement.
        LOCK_ROT      = 1 << 5, // Lock rotation drivetrain movement.
        ABSOLUTE_ROT  = 1 << 6, // Drive with absolute rotation.
    };

    void driveFromPercents(double xPct, double yPct, double rotPct, unsigned flags);
    void driveWithVelocities(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t angVel, unsigned flags);
    
    void sendDebugInfo();
    void doPersistentConfiguration();

private:
    void executeVelocityData();
    void setModuleStates(frc::ChassisSpeeds speeds);

    void stop();

    struct VelocityControlData {
        units::meters_per_second_t xVel;
        units::meters_per_second_t yVel;
        units::radians_per_second_t angVel;
        unsigned flags = ControlFlag::NONE;
    };

    // The data concerning velocity control.
    VelocityControlData controlData {};

    // Feedback variables.
    frc::ChassisSpeeds chassisSpeeds { 0_mps, 0_mps, 0_rad_per_s };

    enum class DriveMode {
        STOPPED,
        VELOCITY,
        TRAJECTORY,
    };

    DriveMode driveMode = DriveMode::STOPPED;

    /*
        Swerve modules
     */



    wpi::array<frc::Translation2d, 4> locations {
        frc::Translation2d(-DRIVE_PREFERENCES.ROBOT_WIDTH/2, -DRIVE_PREFERENCES.ROBOT_LENGTH/2), // Back left.
        frc::Translation2d(+DRIVE_PREFERENCES.ROBOT_WIDTH/2, -DRIVE_PREFERENCES.ROBOT_LENGTH/2), // Back right.
        frc::Translation2d(+DRIVE_PREFERENCES.ROBOT_WIDTH/2, +DRIVE_PREFERENCES.ROBOT_LENGTH/2), // Front right.
        frc::Translation2d(-DRIVE_PREFERENCES.ROBOT_WIDTH/2, +DRIVE_PREFERENCES.ROBOT_LENGTH/2), // Front left.
    };
    /**
     * The helper class that it used to convert chassis speeds into swerve
     * module states.
     */
    frc::SwerveDriveKinematics<4> kinematics { locations };
    /**
     * The helper class that it used to convert swerve
     * module states into chassis speeds.
     */
    frc::SwerveDriveKinematics<4> moduleStates { kinematics };

    // The swerve modules on the robot.
    wpi::array<SwerveModule*, 4> swerveModules {
        new SwerveModule(CAN_SWERVE_DRIVE_BL, CAN_SWERVE_ROTATION_BL, CAN_SWERVE_CANCODER_BL, -16.963_deg+180_deg - 90_deg),
        new SwerveModule(CAN_SWERVE_DRIVE_BR, CAN_SWERVE_ROTATION_BR, CAN_SWERVE_CANCODER_BR, -61.963_deg - 90_deg),
        new SwerveModule(CAN_SWERVE_DRIVE_FR, CAN_SWERVE_ROTATION_FR, CAN_SWERVE_CANCODER_FR, -128.408_deg - 90_deg),
        new SwerveModule(CAN_SWERVE_DRIVE_FL, CAN_SWERVE_ROTATION_FL, CAN_SWERVE_CANCODER_FL, -125.332_deg+180_deg - 90_deg),
    };
    
};