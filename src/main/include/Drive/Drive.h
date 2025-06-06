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

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/HolonomicDriveController.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <ctre/phoenix6/Orchestra.hpp>

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"
#include "swerveModule.h"
#include "Drive/CSVTrajectory.h"
#include "Auto/Action.h"
#include "Limelight.h"

class SwerveFeedback : public wpi::Sendable {
  public:
    SwerveFeedback(wpi::array<SwerveModule*, 4>* _swerveModules);
    void InitSendable(wpi::SendableBuilder& builder);
    frc::Rotation2d robotRotation = frc::Rotation2d(0_deg);
    wpi::array<SwerveModule*, 4>* swerveModules;
};

class Drive : public Component {
  public:
    Drive(Limelight* _limelight);
    ~Drive();

    void process();
    void resetToMatchMode(MatchMode priorMode, MatchMode mode); 

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
    
    void sendFeedback();
    void doConfiguration(bool persist) override;

    /// MARK: Field Centric

    /**
     * Returns whether the current process is finished or not.
     */
    bool isFinished() const;

    /**
     * Calibrates the IMU (Pauses the robot for 4 seconds while it calibrates).
     */
    void calibrateIMU();

    /**
     * Returns whether the IMU is calibrated.
     */
    bool isIMUCalibrated();

    /**
     * Resets the position and rotation of the drivetrain on the field to a
     * specified pose.
     */
    void resetOdometry(frc::Pose2d pose = frc::Pose2d());

    /**
     * Returns the position and rotation of the robot on the field.
     */
    frc::Pose2d getEstimatedPose();

    /**
     * Returns the raw rotation of the robot as recorded by the IMU.
     */
    frc::Rotation2d getRotation();

    /**
     * Resets all drive PID controllers.
     */
    void resetPIDControllers();

    /// MARK: Trajecory

    /**
     * Runs a trajectory.
     */
    void runTrajectory(const CSVTrajectory* trajectory, const std::map<u_int32_t, Action*>& actionMap);

    void setupInitialTrajectoryPosition(const CSVTrajectory* trajectory);

    void setAccelerationReduction(double reduction);

    void slowYourRoll();
    void unslowYourRoll();

    void beginLineup(LineupHorizontal lineupHorizontal, bool L4);

    bool isLineUpDone();

private:
    void executeVelocityData();
    void setModuleStates(frc::ChassisSpeeds speeds);

    void stop();

    void makeBrick();

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
        LINEUP,
        ORCHESTRA,
    };

    DriveMode driveMode = DriveMode::STOPPED;


    /// MARK: Modules


    wpi::array<frc::Translation2d, 4> locations {
        frc::Translation2d(+PreferencesDrive::ROBOT_LENGTH/2, +PreferencesDrive::ROBOT_WIDTH/2), // FRONT LEFT.
        frc::Translation2d(-PreferencesDrive::ROBOT_LENGTH/2, +PreferencesDrive::ROBOT_WIDTH/2), // BACK LEFT.
        frc::Translation2d(-PreferencesDrive::ROBOT_LENGTH/2, -PreferencesDrive::ROBOT_WIDTH/2), // BACK RIGHT.
        frc::Translation2d(+PreferencesDrive::ROBOT_LENGTH/2, -PreferencesDrive::ROBOT_WIDTH/2), // FRONT RIGHT.
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
    wpi::array<SwerveModule*, 4> swerveModules { // ENCODER OFFSETS: 2/18/2025 REAL BOT
        new SwerveModule(CAN_SWERVE_DRIVE_FL, CAN_SWERVE_ROTATION_FL, CAN_SWERVE_CANCODER_FL, 128.49629_deg),
        new SwerveModule(CAN_SWERVE_DRIVE_BL, CAN_SWERVE_ROTATION_BL, CAN_SWERVE_CANCODER_BL, -23.02734375_deg),
        new SwerveModule(CAN_SWERVE_DRIVE_BR, CAN_SWERVE_ROTATION_BR, CAN_SWERVE_CANCODER_BR, -28.8281245_deg + 180_deg),
        new SwerveModule(CAN_SWERVE_DRIVE_FR, CAN_SWERVE_ROTATION_FR, CAN_SWERVE_CANCODER_FR, 142.11914_deg + 180_deg),
    };


    /// MARK: Field Centric


    ctre::phoenix6::hardware::Pigeon2 pigeon { CAN_PIGEON };

     /**
     * The class that handles tracking the position of the robot on the field
     * during the match.
     */
    frc::SwerveDrivePoseEstimator<4> poseEstimator {
        kinematics,
        getRotation(),
        getModulePositions(),
        frc::Pose2d(),
        { 0.01, 0.01, 0.01 }, // Standard deviations of model states.
        { 0.1, 0.1, 999.99 } // Standard deviations of the vision measurements.
    };
    bool limelightReliable = false;

    // PID Controller for angular drivetrain movement.
    frc::ProfiledPIDController<units::radians> manualThetaPIDController {
        PreferencesDrive::PID_THETA.Kp, PreferencesDrive::PID_THETA.Ki, PreferencesDrive::PID_THETA.Kd,
        frc::TrapezoidProfile<units::radians>::Constraints(PreferencesDrive::DRIVE_MANUAL_MAX_ANG_VEL, PreferencesDrive::DRIVE_MANUAL_MAX_ANG_ACCEL)
    };

    bool imuCalibrated = false;

    /**
     * Returns the states of the swerve modules. (velocity and rotatation)
     */
    wpi::array<frc::SwerveModuleState, 4> getModuleStates();

    /**
     * Returns the positions of the swerve modules.
     */
    wpi::array<frc::SwerveModulePosition, 4> getModulePositions();

    /**
     * Updates the position and rotation of the drivetrain on the field.
     */
    void updateOdometry();


    /// MARK: Trajectory


    frc::Pose2d targetPose;

    bool actionExecuting = false;

    // PID Controller for X and Y axis drivetrain movement.
    frc::PIDController xPIDController { PreferencesDrive::PID_XY.Kp, PreferencesDrive::PID_XY.Ki, PreferencesDrive::PID_XY.Kd },
                       yPIDController { PreferencesDrive::PID_XY.Kp, PreferencesDrive::PID_XY.Ki, PreferencesDrive::PID_XY.Kd };

    // PID Controller for angular drivetrain movement.
    frc::ProfiledPIDController<units::radians> trajectoryThetaPIDController {
        PreferencesDrive::PID_THETA.Kp, PreferencesDrive::PID_THETA.Ki, PreferencesDrive::PID_THETA.Kd,
        frc::TrapezoidProfile<units::radians>::Constraints(PreferencesDrive::DRIVE_AUTO_MAX_ANG_VEL, PreferencesDrive::DRIVE_AUTO_MAX_ANG_ACCEL)
    };

   // The drive controller that will handle the drivetrain movement.
    frc::HolonomicDriveController driveController;

    // The trajectory that is currently being run.
    const CSVTrajectory* trajectory = nullptr;

    // The available actions.
    const std::map<u_int32_t, Action*>* trajectoryActions = nullptr;

    // Actions that are completed.
    std::vector<u_int32_t> doneTrajectoryActions;

    // The current action.
    std::map<units::second_t, u_int32_t>::const_iterator trajectoryActionIter;

    frc::Timer trajectoryTimer;

    frc::Field2d feedbackField {};
    frc::Field2d trajectoryField {};
    frc::Field2d lineupField {};

    /**
     * Executes the instructions for when the robot is running a trajectory.
     */
    void execTrajectory();
    void driveToState(CSVTrajectory::State state, bool isLineup, double limiting);

    double speedLimiting = 0.65; // This multiplies

    bool actionExecutingButLineup = false;

    /// MARK: Limelight


    Limelight* limelight;


    /// MARK: Dashboard Extras


    SwerveFeedback swerveFeedback {&swerveModules};

    enum Quadrant {
        kNONE,
        kLEFT,
        kRIGHT
    };

    Quadrant getCurrentQuadrant();

    std::string getCurrentQuadrantAsString();

    Quadrant currentQuadrant = Quadrant::kNONE;
    Quadrant lastQuadrant = Quadrant::kNONE;

    // MARK: Lineup
    void execLineup();


    frc::Pose2d calculateFinalLineupPose(int posId, LineupHorizontal lineupHorizontal, bool isL4);

    // PID Controller for X and Y axis lineup.
    frc::PIDController lineupXPIDController { PreferencesDrive::PID_LINEUP_XY.Kp, PreferencesDrive::PID_LINEUP_XY.Ki, PreferencesDrive::PID_LINEUP_XY.Kd};
    frc::PIDController lineupYPIDController { PreferencesDrive::PID_LINEUP_XY.Kp, PreferencesDrive::PID_LINEUP_XY.Ki, PreferencesDrive::PID_LINEUP_XY.Kd};
    // frc::ProfiledPIDController<units::meters> lineupXPIDController {
    //     PreferencesDrive::PID_LINEUP_XY.Kp, PreferencesDrive::PID_LINEUP_XY.Ki, PreferencesDrive::PID_LINEUP_XY.Kd,
    //     frc::TrapezoidProfile<units::meters>::Constraints(PreferencesDrive::MAX_LINEUP_VEL, PreferencesDrive::MAX_LINEUP_ACCEL)
    // };
    // frc::ProfiledPIDController<units::meters> lineupYPIDController { PreferencesDrive::PID_LINEUP_XY.Kp, PreferencesDrive::PID_LINEUP_XY.Ki, PreferencesDrive::PID_LINEUP_XY.Kd,
    //     frc::TrapezoidProfile<units::meters>::Constraints(PreferencesDrive::MAX_LINEUP_VEL, PreferencesDrive::MAX_LINEUP_ACCEL)
    // };

    // PID Controller for angular lineup.
    frc::ProfiledPIDController<units::radians> lineupThetaPIDController {
        PreferencesDrive::PID_LINEUP_THETA.Kp, PreferencesDrive::PID_LINEUP_THETA.Ki, PreferencesDrive::PID_LINEUP_THETA.Kd,
        frc::TrapezoidProfile<units::radians>::Constraints(PreferencesDrive::DRIVE_AUTO_MAX_ANG_VEL, PreferencesDrive::DRIVE_AUTO_MAX_ANG_ACCEL)
    };

   // The drive controller that will handle the lineup.
    frc::HolonomicDriveController driveLineupController;

    frc::Pose2d lineupPose = {};
    std::optional<lineup_t> lineupTargetData = std::nullopt;

    bool isAuto = false;
    bool lineUpDone = false;

    double distToLineupPose();
    
    uint8_t inline const static poseIDToAprilTag(int id) {
        if (auto ally = frc::DriverStation::GetAlliance()) {
            if (ally == frc::DriverStation::Alliance::kRed) {
                if (id == 5) return 6;
                return id + 7;
            }
        }
        const std::vector<uint8_t> l = {18, 17, 22, 21, 20, 19, 18};
        return l[id];
    }

    // MARK: Orchestra

    ctre::phoenix6::Orchestra swerveOrchestra;

    bool isPlaying = false;
    void orchestrate(std::string fp);
    void unOrchestrate();

    friend class Robot;
    friend class Gamepiece;
};