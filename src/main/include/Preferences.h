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
    double Kff = 0;
    double Kizone = 0;
    units::angular_velocity::degrees_per_second_t MaxVel = 0_deg_per_s;
    units::angular_acceleration::degrees_per_second_squared_t MaxAccel = 0_deg_per_s_sq;
    void clear() 
    {
        Kp = Ki = Kd = Kff = Kizone = Kv = Ks = 0;
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

    units::radians_per_second_t DRIVE_AUTO_MAX_ANG_VEL = 6.28_rad_per_s;
    units::radians_per_second_squared_t DRIVE_AUTO_MAX_ANG_ACCEL = 3.14_rad_per_s_sq;

    units::meters_per_second_t DRIVE_MANUAL_MAX_VEL = 2.5_mps;
    units::degrees_per_second_t DRIVE_MANUAL_MAX_ANG_VEL = 240_deg_per_s; // 540
    units::radians_per_second_squared_t DRIVE_MANUAL_MAX_ANG_ACCEL = 4.5_rad_per_s_sq; // 9.42

    DrivePreferences()
    {
        PID_XY.Kp = 7.5;
        // PID_XY.Ki = 0.001;
        PID_XY.Kd = 0.1;

        PID_THETA.Kp = 4.0;
        PID_THETA.Kd = 0.1;
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
    double AXIS_DEADZONE = .2;
    const bool DRIVE_DISABLED = true;
    const bool AUX_DISABLED = false;
};
static PreferencesControls CONTROLS_PREFERENCE;

struct PreferencesCalgae {
    double MOTOR_SPEED_STOPPED = 0.0;
    double MOTOR_SPEED_INTAKE_CORAL = 0.75;
    double MOTOR_SPEED_SHOOT_CORAL = -0.75;
    double MOTOR_SPEED_INTAKE_ALGAE = 1.0;
    double MOTOR_SPEED_SHOOT_ALGAE = -1.0;
    double MOTOR_SPEED_INTAKE_REGRAB = 0.5;
};
static PreferencesCalgae CALGAE_PREFERENCE;

struct PreferencesWrist {
    PID_t PID;
    double MAX_SPEED = 1;
    PreferencesWrist() {
    }
};
static PreferencesWrist WRIST_PREFERENCE;

struct PreferencesElevator {
    PID_t PID;
    double MAX_SPEED = 0.05;
    PreferencesElevator() {
        PID.Kp = 0.1;
    }
};

static PreferencesElevator ELEVATOR_PREFERENCE;

struct PreferencesTrajectory
{
    units::meter_t FIELD_X = 17.55_m;
    units::meter_t FIELD_Y = 8.05_m;
};
static PreferencesTrajectory TRAJECTORY_PREFERENCE;
