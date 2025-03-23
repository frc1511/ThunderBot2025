#include "Preferences.h"

/***********************************************************/
// Drive

const units::inch_t PreferencesDrive::WHEEL_DISTANCE_FROM_FRAME = 2.625_in;
const units::inch_t PreferencesDrive::ROBOT_WIDTH = 29_in - WHEEL_DISTANCE_FROM_FRAME * 2;
const units::inch_t PreferencesDrive::ROBOT_LENGTH = 30_in - WHEEL_DISTANCE_FROM_FRAME * 2;
const units::radians_per_second_t PreferencesDrive::DRIVE_AUTO_MAX_ANG_VEL = 9.28_rad_per_s;
const units::radians_per_second_squared_t PreferencesDrive::DRIVE_AUTO_MAX_ANG_ACCEL = 6.14_rad_per_s_sq;

const units::meters_per_second_t PreferencesDrive::DRIVE_MANUAL_MAX_VEL = 5_mps;
const units::degrees_per_second_t PreferencesDrive::DRIVE_MANUAL_MAX_ANG_VEL = 540_deg_per_s; // 540
const units::radians_per_second_squared_t PreferencesDrive::DRIVE_MANUAL_MAX_ANG_ACCEL = 9.42_rad_per_s_sq; // 9.42
const units::meters_per_second_squared_t PreferencesDrive::MAX_ACCEL = 0.01_mps_sq;
const double PreferencesDrive::DRIVE_ROTATION_SPEED_MANUAL_LIMITER_SLOWNESS_FACTOR = .5;
const double PreferencesDrive::DRIVE_VELOCITY_SPEED_MANUAL_LIMITER_SLOWNESS_FACTOR = .5;

const frc::Pose2d PreferencesDrive::REEF_POSE = {4.493839_m, 4.025221_m, frc::Rotation2d(0_deg)};
const units::meter_t PreferencesDrive::HORIZONTAL_REEF_MOVE = 0.17145_m;
const units::meter_t PreferencesDrive::VERTICAL_REEF_MOVE = 0.18_m;

const PID_t PreferencesDrive::PID_XY = {
    .Kp = 7.5
};
const PID_t PreferencesDrive::PID_THETA = {
    .Kp = 6,
    .Kd = 0.125
};

const PID_t PreferencesDrive::PID_LINEUP_XY = {
    .Kp = 5.5,
    .Kd = 0.5
};
const PID_t PreferencesDrive::PID_LINEUP_THETA = {
    .Kp = 6,
    .Kd = 0.125
};

/**
 * Reef x, 0
 * Field Width x, Reef y
 */
const std::vector<Point> PreferencesDrive::QUADRANT_LEFT = {
    Point{.x = 4.493839_m, .y = 0_m},
    Point{.x = 17.55_m, .y = 4.025221_m}
};
/**
 * 0, 0
 * Reef x, Reef y
 */
const std::vector<Point> PreferencesDrive::QUADRANT_RIGHT = {
    Point{.x = 0_m, .y = 0_m},
    Point{.x = 4.493839_m, .y = 4.025221_m}
};

// Point for welded field
const Point PreferencesDrive::APRIL_TAG_18 {
    .x = 3.6576_m,
    .y = 4.0259_m
};

const units::meter_t PreferencesDrive::ROBOT_WITH_BUMPERS_LENGTH = 0.930275_m;

const frc::Pose2d PreferencesDrive::MASTER_LINEUP_POSE = {
    PreferencesDrive::APRIL_TAG_18.x - PreferencesDrive::ROBOT_WITH_BUMPERS_LENGTH / 2,
    PreferencesDrive::APRIL_TAG_18.y,
    frc::Rotation2d(0_deg)
};

const double PreferencesDrive::LINEUP_POSE_TOLERANCE = 0.008;
const double PreferencesDrive::LINEUP_LIMELIGHT_DEADZONE= 0.02;


/***********************************************************/
// Swerve Drive Motor

const PID_t PreferencesDriveMotor::PID = {
    .Kp = 0.1,
    .Kv = 0.124,
    .Kff = 0
};
const units::current::ampere_t PreferencesDriveMotor::MAX_AMPERAGE = 40_A;
const double PreferencesDriveMotor::METERS_TO_TURNS = 16.48828125;
const double PreferencesDriveMotor::TURNS_TO_METERS = (1 / (METERS_TO_TURNS));


/***********************************************************/
// Swerve Turn

const PID_t PreferencesTurnMotor::PID = {
    .Kp = 33,
    .Ki = 0,
    .Kd = 0
};

const units::current::ampere_t PreferencesTurnMotor::MAX_AMPERAGE = 30_A;
const double PreferencesTurnMotor::TURN_RADIAN_TO_ENCODER_FACTOR = 2.03362658302;


/***********************************************************/
// Swerve

const units::second_t PreferencesSwerve::DRIVE_RAMP_TIME = 0.3_s;



/***********************************************************/
// Controls

const double PreferencesControls::AXIS_DEADZONE = 0.1;

const double PreferencesControls::kGROUND = 0.0; 
const double PreferencesControls::kPROCESSOR = 0.0;
const double PreferencesControls::kCORAL_STATION = 0.0;
const double PreferencesControls::kL1 = 0.0;
const double PreferencesControls::kL2 = 0.0;
const double PreferencesControls::kL3 = 0.5;
const double PreferencesControls::kL4 = 0.5;
const double PreferencesControls::kNET = 0.5;
const bool PreferencesControls::wristEncoderBroken = false;



/***********************************************************/
// Calgae

const double PreferencesCalgae::MOTOR_SPEED_STOPPED = 0.0;
const double PreferencesCalgae::MOTOR_SPEED_INTAKE_CORAL = 1.0;
const double PreferencesCalgae::MOTOR_SPEED_SHOOT_CORAL = -0.7;
const double PreferencesCalgae::MOTOR_SPEED_INTAKE_ALGAE = 1.0;
const double PreferencesCalgae::MOTOR_SPEED_SHOOT_ALGAE = -0.7;
const double PreferencesCalgae::MOTOR_SPEED_INTAKE_REGRAB = 0.5;



/***********************************************************/
// Wrist

const double PreferencesWrist::MAX_SPEED = 0.4;
const double PreferencesWrist::MAX_PIT_SPEED = 0.2;
const units::degree_t PreferencesWrist::UP_ZERO = 100_deg;
const double PreferencesWrist::MAX_FEED_FORWARD_POWER_HIGH_ANGLE = 0.07;
const double PreferencesWrist::MAX_FEED_FORWARD_POWER_LOW_ANGLE = 0.1;
const units::degree_t PreferencesWrist::ANGLE_TOLERANCE = 1_deg;
const units::degree_t PreferencesWrist::ANGLE_TOLERANCE_AUTO = 3_deg;
const units::degree_t PreferencesWrist::LOWEST_ANGLE = -35_deg;

const units::degree_t PreferencesWrist::HIGHEST_ANGLE = 100_deg;
const units::degree_t PreferencesWrist::ENCODER_FAILURE_OUTBOUND = 10_deg;
const units::degree_t PreferencesWrist::UNSAFE_MIN = 80_deg;



/***********************************************************/
// Elevator

const double PreferencesElevator::MAX_DOWN_SPEED = 0.7;
const double PreferencesElevator::MAX_UP_SPEED = 0.7;
const double PreferencesElevator::MAX_DOWN_PIT_SPEED = 0.5;
const double PreferencesElevator::MAX_UP_PIT_SPEED = 0.5;
const double PreferencesElevator::TARGET_TOLERANCE = 0.2; // in turn_t



/***********************************************************/
// Trajectory

const units::meter_t PreferencesTrajectory::FIELD_X = 17.55_m;
const units::meter_t PreferencesTrajectory::FIELD_Y = 8.05_m;



/***********************************************************/
// Blinky

const int PreferencesBlinkyBlinky::CAGE_STATUS_ID = 0; // This does not mean the first LED, this is the first LED in the status bar
const int PreferencesBlinkyBlinky::ELEVATOR_STATUS_ID = 1;
const int PreferencesBlinkyBlinky::CORAL_STATUS_ID = 2;
const int PreferencesBlinkyBlinky::ALGAE_STATUS_ID = 3;



/***********************************************************/
// Hang

const double PreferencesHang::MAX_POSITION = -1;
const double PreferencesHang::MAX_HANG_SPEED_UP = 1;
const double PreferencesHang::MAX_HANG_SPEED_DOWN = -1;
const double PreferencesHang::HANG_SPEED_DOWN_SLOW = -0.5;
const double PreferencesHang::BACKTRACKING_SPEED = -0.1;
const double PreferencesHang::BACKTRACKING_DISTANCE = 1;
const units::second_t PreferencesHang::DISENGAGE_DURATION = 0.3_s;



/***********************************************************/
// Limelight

const std::string PreferencesLimelight::LIMELIGHT_FRONT = "limelight-left";
const std::string PreferencesLimelight::LIMELIGHT_BACK = "limelight-right";

const int PreferencesLimelight::PIPELINE_APRILTAGS = 0;
const int PreferencesLimelight::PIPELINE_EMPTY = 1;