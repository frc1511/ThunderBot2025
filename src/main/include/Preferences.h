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



struct DrivePreferences {
    static const units::inch_t WHEEL_DISTANCE_FROM_FRAME;

    static const PID_t PID_XY;
    static const PID_t PID_THETA;

    static const units::inch_t ROBOT_WIDTH;
    static const units::inch_t ROBOT_LENGTH;

    static const units::radians_per_second_t DRIVE_AUTO_MAX_ANG_VEL;
    static const units::radians_per_second_squared_t DRIVE_AUTO_MAX_ANG_ACCEL;

    static const units::meters_per_second_t DRIVE_MANUAL_MAX_VEL;
    static const units::degrees_per_second_t DRIVE_MANUAL_MAX_ANG_VEL;
    static const units::radians_per_second_squared_t DRIVE_MANUAL_MAX_ANG_ACCEL;
};

struct PreferencesDriveMotor
{
    PID_t PID;
    units::current::ampere_t MAX_AMPERAGE;
    double METERS_TO_TURNS;
    double TURNS_TO_METERS;

    PreferencesDriveMotor();
};

struct PreferencesTurnMotor
{
    PID_t PID;

    units::current::ampere_t MAX_AMPERAGE;
    double TURN_RADIAN_TO_ENCODER_FACTOR;

    PreferencesTurnMotor();
};

struct PreferencesSwerve 
{
    static const PreferencesDriveMotor DRIVE_MOTOR;
    static const PreferencesTurnMotor TURN_MOTOR;
    static const units::second_t DRIVE_RAMP_TIME;
};

struct PreferencesControls
{
    static const double AXIS_DEADZONE;
};
