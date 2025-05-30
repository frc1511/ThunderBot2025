#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/voltage.h>
#include <vector>
#include <frc/geometry/Pose2d.h>

struct PID_t
{
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kv = 0;
    double Ks = 0;
    double Kg = 0;
    double Kff = 0;
    double Kizone = 0;
    
    // Elevator ff has weird ones
    double Kv_EVFF = 0;
    double Ka_EVFF = 0;

    units::angular_velocity::degrees_per_second_t MaxVel = 0_deg_per_s;
    units::angular_acceleration::degrees_per_second_squared_t MaxAccel = 0_deg_per_s_sq;
    void clear() 
    {
        Kp = Ki = Kd = Kff = Kizone = Kv = Ks = Kg = 0;
        MaxVel = 0_deg_per_s;
        MaxAccel = 0_deg_per_s_sq;
    }
};

struct Point {
    units::meter_t x;
    units::meter_t y;
};


struct PreferencesDrive {
    static const units::inch_t WHEEL_DISTANCE_FROM_FRAME;

    static const units::inch_t ROBOT_WIDTH;
    static const units::inch_t ROBOT_LENGTH;

    static const units::radians_per_second_t DRIVE_AUTO_MAX_ANG_VEL;
    static const units::radians_per_second_squared_t DRIVE_AUTO_MAX_ANG_ACCEL;

    static const units::meters_per_second_t DRIVE_MANUAL_MAX_VEL;
    static const units::degrees_per_second_t DRIVE_MANUAL_MAX_ANG_VEL;
    static const units::radians_per_second_squared_t DRIVE_MANUAL_MAX_ANG_ACCEL;
    static const units::meters_per_second_squared_t MAX_ACCEL;

    static const units::meters_per_second_t MAX_LINEUP_VEL;
    static const units::meters_per_second_squared_t MAX_LINEUP_ACCEL;

    static const double DRIVE_ROTATION_SPEED_MANUAL_LIMITER_SLOWNESS_FACTOR;
    static const double DRIVE_VELOCITY_SPEED_MANUAL_LIMITER_SLOWNESS_FACTOR;
    
    static const frc::Pose2d REEF_POSE;
    static const units::meter_t HORIZONTAL_REEF_MOVE;
    static const units::meter_t VERTICAL_REEF_MOVE;

    static const PID_t PID_XY;
    static const PID_t PID_THETA;

    static const PID_t PID_LINEUP_XY;
    static const PID_t PID_LINEUP_THETA;

    static const std::vector<Point> QUADRANT_LEFT;
    static const std::vector<Point> QUADRANT_RIGHT;

    static const Point APRIL_TAG_18;
    
    static const units::meter_t ROBOT_WITH_BUMPERS_LENGTH;
    
    static const frc::Pose2d MASTER_LINEUP_POSE;

    static const double LINEUP_POSE_TOLERANCE;

    static const double LINEUP_LIMELIGHT_DEADZONE;
};


enum class Branch {
    kL2,
    kL3,
    kL4
};

enum class LineupHorizontal {
    kLEFT,
    kCENTER,
    kRIGHT
};

typedef std::pair<uint8_t, LineupHorizontal> lineup_t;

extern const std::map<lineup_t, std::map<Branch, units::turn_t>> ELEVATOR_BRANCH_OFFSETS;

#define DRIVE_BRANCH_OFFSET_HORIZONTAL_INDEX 0
#define DRIVE_BRANCH_OFFSET_L4_INDEX 1
//                                        (HORIZONTAL, L4) If these are 0, they are disregarded
extern const std::map<lineup_t, std::tuple<units::meter_t, units::meter_t>> DRIVE_BRANCH_OFFSETS;

struct PreferencesDriveMotor
{
    static const PID_t PID;
    static const units::current::ampere_t MAX_AMPERAGE;
    static const double METERS_TO_TURNS;
    static const double TURNS_TO_METERS;
};

struct PreferencesTurnMotor
{
    static const PID_t PID;

    static const units::current::ampere_t MAX_AMPERAGE;
    static const double TURN_RADIAN_TO_ENCODER_FACTOR;
};

struct PreferencesSwerve 
{
    static const units::second_t DRIVE_RAMP_TIME;
};

struct PreferencesControls
{
    static const double AXIS_DEADZONE;

    static const double kGROUND; 
    static const double kPROCESSOR;
    static const double kCORAL_STATION;
    static const double kL1;
    static const double kL2;
    static const double kL3;
    static const double kL4;
    static const double kNET;

    static const bool wristEncoderBroken;

};

struct PreferencesCalgae {
    static const double MOTOR_SPEED_STOPPED;
    static const double MOTOR_SPEED_INTAKE_CORAL;
    static const double MOTOR_SPEED_SHOOT_CORAL;
    static const double MOTOR_SPEED_INTAKE_ALGAE;
    static const double MOTOR_SPEED_SHOOT_ALGAE;
    static const double MOTOR_SPEED_INTAKE_REGRAB;
};

struct PreferencesWrist {
    static const double MAX_SPEED;
    static const double MAX_PIT_SPEED;
    static const units::degree_t UP_ZERO;
    static const double MAX_FEED_FORWARD_POWER_HIGH_ANGLE;
    static const double MAX_FEED_FORWARD_POWER_LOW_ANGLE;
    static const units::degree_t ANGLE_TOLERANCE;
    static const units::degree_t ANGLE_TOLERANCE_AUTO;
    static const units::degree_t ANGLE_TOLERANCE_OOO_CONTINUE; // Order of operations
    static const units::degree_t LOWEST_ANGLE;

    static const units::degree_t HIGHEST_ANGLE;
    static const units::degree_t ENCODER_FAILURE_OUTBOUND;
    static const units::degree_t UNSAFE_MIN;
};

struct PreferencesElevator {
    static const units::volt_t MAX_DOWN_VOLTS;
    static const units::volt_t MAX_UP_VOLTS;
    static const double MAX_DOWN_PIT_SPEED;
    static const double MAX_UP_PIT_SPEED;
    static const double TARGET_TOLERANCE; // In turn_t

    static const PID_t PID;
    static const units::turns_per_second_t MAX_VEL;
    static const units::turns_per_second_squared_t MAX_ACCEL;
};

struct PreferencesTrajectory
{
    static const units::meter_t FIELD_X;
    static const units::meter_t FIELD_Y;
};


struct PreferencesBlinkyBlinky
{
    static constexpr size_t LED_TOTAL = 42;
    static constexpr size_t LED_SIDE_STRIP_TOTAL= 19;
    static constexpr size_t LED_STATUS_STRIP_TOTAL = 4;
    static const int CAGE_STATUS_ID ; // This does not mean the first LED, this is the first LED in the status bar
    static const int ELEVATOR_STATUS_ID;
    static const int CORAL_STATUS_ID;
    static const int ALGAE_STATUS_ID;
};

struct PreferencesHang
{
    static const double MAX_POSITION;
    static const double RETRACT_POSITION;
    static const double MAX_HANG_SPEED_UP;
    static const double MAX_HANG_SPEED_DOWN;
    static const double PIT_HANG_SPEED_DOWN;
    static const double HANG_SPEED_DOWN_SLOW;
    static const double BACKTRACKING_SPEED;
    static const double BACKTRACKING_DISTANCE;
    static const double MAX_DEPLOY_POSITION;
    static const units::second_t DISENGAGE_DURATION;
    static const double POSITION_TOL;
    static const double RETRACT_POSITION_TOL;
};


struct PreferencesLimelight
{
    static const std::string LIMELIGHT_FRONT;
    static const std::string LIMELIGHT_BACK;

    static const int PIPELINE_APRILTAGS;
    static const int PIPELINE_EMPTY;
};
