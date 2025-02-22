#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/current.h>

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
    units::angular_velocity::degrees_per_second_t MaxVel = 0_deg_per_s;
    units::angular_acceleration::degrees_per_second_squared_t MaxAccel = 0_deg_per_s_sq;
    void clear() 
    {
        Kp = Ki = Kd = Kff = Kizone = Kv = Ks = Kg = 0;
        MaxVel = 0_deg_per_s;
        MaxAccel = 0_deg_per_s_sq;
    }
};

const units::inch_t WHEEL_DISTANCE_FROM_FRAME = 2.625_in;

struct DrivePreferences {
    PID_t PID_XY;
    PID_t PID_THETA;
    units::inch_t ROBOT_WIDTH = 29_in - WHEEL_DISTANCE_FROM_FRAME * 2;
    units::inch_t ROBOT_LENGTH = 30_in - WHEEL_DISTANCE_FROM_FRAME * 2;

    units::radians_per_second_t DRIVE_AUTO_MAX_ANG_VEL = 9.28_rad_per_s;
    units::radians_per_second_squared_t DRIVE_AUTO_MAX_ANG_ACCEL = 6.14_rad_per_s_sq;

    units::meters_per_second_t DRIVE_MANUAL_MAX_VEL = 5_mps;
    units::degrees_per_second_t DRIVE_MANUAL_MAX_ANG_VEL = 240_deg_per_s; // 540
    units::radians_per_second_squared_t DRIVE_MANUAL_MAX_ANG_ACCEL = 4.5_rad_per_s_sq; // 9.42

    units::meters_per_second_squared_t MAX_ACCEL = 0.01_mps_sq;

    DrivePreferences()
    {
        PID_XY.Kp = 7.5;
        // PID_XY.Ki = 0.001;
        // PID_XY.Kd = 0.1;

        PID_THETA.Kp = 13;
        PID_THETA.Kd = 0.125;
    }
};
static const DrivePreferences DRIVE_PREFERENCES;

struct PreferencesDriveMotor
{
    PID_t PID;
    units::current::ampere_t MAX_AMPERAGE = 40_A;
    double METERS_TO_TURNS = 16.6474609375;
    double TURNS_TO_METERS = (1 / (METERS_TO_TURNS));
};

struct PreferencesTurnMotor
{
    PID_t PID;

    units::current::ampere_t MAX_AMPERAGE = 30_A;
    double TURN_RADIAN_TO_ENCODER_FACTOR = 2.03362658302;
};

struct PreferencesSwerve 
{
    PreferencesDriveMotor DRIVE_MOTOR;
    PreferencesTurnMotor TURN_MOTOR;
    units::second_t DRIVE_RAMP_TIME = 0.3_s;
    PreferencesSwerve()
    {
        DRIVE_MOTOR.PID.Kp = 0.1;
        DRIVE_MOTOR.PID.Kv = 0.124;
        DRIVE_MOTOR.PID.Kff = 0;

        TURN_MOTOR.PID.Kp = 33;
        TURN_MOTOR.PID.Ki = 0;
        TURN_MOTOR.PID.Kd = 0;
    }
};
static PreferencesSwerve SWERVE_PREFERENCE;

struct PreferencesControls
{
    double AXIS_DEADZONE = 0.1;

    double kGROUND = 0.0; 
    double kPROCESSOR = 0.0;
    double kCORAL_STATION = 0.0;
    double kL1 = 0.0;
    double kL2 = 0.0;
    double kL3 = 0.5;
    double kL4 = 0.5;
    double kNET = 0.5;

    static const bool wristEncoderBroken = false;

};
static PreferencesControls CONTROLS_PREFERENCE;

struct PreferencesCalgae {
    double MOTOR_SPEED_STOPPED = 0.0;
    double MOTOR_SPEED_INTAKE_CORAL = 0.75;
    double MOTOR_SPEED_SHOOT_CORAL = -0.7;
    double MOTOR_SPEED_INTAKE_ALGAE = 1.0;
    double MOTOR_SPEED_SHOOT_ALGAE = -0.7;
    double MOTOR_SPEED_INTAKE_REGRAB = 0.5;
};
static PreferencesCalgae CALGAE_PREFERENCE;

struct PreferencesWrist {
    double MAX_SPEED = 0.3;
    units::degree_t UP_ZERO = 100_deg;
    double MAX_FEED_FORWARD_POWER_HIGH_ANGLE = 0.07;
    double MAX_FEED_FORWARD_POWER_LOW_ANGLE = 0.1;
    units::degree_t ANGLE_TOLERANCE = 0.25_deg;
    units::degree_t LOWEST_ANGLE = -35_deg;

    units::degree_t HIGHEST_ANGLE = 100_deg;
    units::degree_t ENCODER_FAILURE_OUTBOUND = 10_deg;
    units::degree_t UNSAFE_MIN = 80_deg;
    PreferencesWrist() {}
};
static PreferencesWrist WRIST_PREFERENCE;

struct PreferencesElevator {
    double MAX_DOWN_SPEED = 0.3;
    double MAX_UP_SPEED = 0.35;
    double TARGET_TOLERANCE = 0.2; // in turn_t
    PreferencesElevator() {}
};

static PreferencesElevator ELEVATOR_PREFERENCE;

struct PreferencesTrajectory
{
    units::meter_t FIELD_X = 17.55_m;
    units::meter_t FIELD_Y = 8.05_m;
};
static PreferencesTrajectory TRAJECTORY_PREFERENCE;

// #define for array, it won't accept the preferences value
#define BLINKY_BLINKY_LED_TOTAL 42
#define BLINKY_BLINKY_LED_SIDE_TOTAL 19
#define BLINKY_BLINKY_LED_STATUS_TOTAL 4
struct PreferencesBlinkyBlinky
{
    size_t LED_TOTAL = BLINKY_BLINKY_LED_TOTAL;
    size_t LED_SIDE_STRIP_TOTAL = BLINKY_BLINKY_LED_SIDE_TOTAL;
    size_t LED_STATUS_STRIP_TOTAL = BLINKY_BLINKY_LED_STATUS_TOTAL;
    int CAGE_STATUS_ID = 0; // This does not mean the first LED, this is the first LED in the status bar
    int ELEVATOR_STATUS_ID = 1;
    int CORAL_STATUS_ID = 2;
    int ALGAE_STATUS_ID = 3;
};
static PreferencesBlinkyBlinky BLINKY_BLINKY_PREFERENCE;

struct PreferencesHang
{
    double MAX_POSITION = 1;
    double MAX_HANG_SPEED_UP = 0.5; // TODO: CHANGE LATER
    double MAX_HANG_SPEED_DOWN = -0.5;
    double BACKTRACKING_SPEED = -0.1;
    double BACKTRACKING_DISTANCE = 1;
    units::second_t DISENGAGE_DURATION = 0.3_s;
};

static PreferencesHang HANG_PREFERENCE;


struct PreferencesLimelight
{
    std::string LIMELIGHT_NAME = "limelight";
};
static PreferencesLimelight LIMELIGHT_PREFERENCE;