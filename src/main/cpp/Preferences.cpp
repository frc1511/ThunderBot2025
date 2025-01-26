#include "Preferences.h"

const units::inch_t DrivePreferences::WHEEL_DISTANCE_FROM_FRAME = 2.625_in;

const PID_t DrivePreferences::PID_XY = {
    .Kp = 3.25,
    .Ki = 0.1
};

const PID_t DrivePreferences::PID_THETA = {
    .Kp = 8.0,
    .Kd = 0.1
};


const units::inch_t DrivePreferences::ROBOT_WIDTH = 29_in - WHEEL_DISTANCE_FROM_FRAME * 2;
const units::inch_t DrivePreferences::ROBOT_LENGTH = 30_in - WHEEL_DISTANCE_FROM_FRAME * 2;
const units::radians_per_second_t DrivePreferences::DRIVE_AUTO_MAX_ANG_VEL = 6.28_rad_per_s;
const units::radians_per_second_squared_t DrivePreferences::DRIVE_AUTO_MAX_ANG_ACCEL = 3.14_rad_per_s_sq;
const units::meters_per_second_t DrivePreferences::DRIVE_MANUAL_MAX_VEL = 2.5_mps;
const units::degrees_per_second_t DrivePreferences::DRIVE_MANUAL_MAX_ANG_VEL = 240_deg_per_s; // 540
const units::radians_per_second_squared_t DrivePreferences::DRIVE_MANUAL_MAX_ANG_ACCEL = 4.5_rad_per_s_sq;

PreferencesDriveMotor::PreferencesDriveMotor() {
    PID.Kp = 0.1,
    PID.Kv = 0.124;
    PID.Kff = 0;

    MAX_AMPERAGE = 40_A;
    METERS_TO_TURNS = 16.6474609375;
    TURNS_TO_METERS = (1 / (METERS_TO_TURNS));
}

PreferencesTurnMotor::PreferencesTurnMotor() {
    PID.Kp = 33;
    PID.Ki = 0;
    PID.Kd = 0;

    MAX_AMPERAGE = 30_A;
    TURN_RADIAN_TO_ENCODER_FACTOR = 2.03362658302;
}

const units::second_t PreferencesSwerve::DRIVE_RAMP_TIME = 0.3_s;
const PreferencesDriveMotor PreferencesSwerve::DRIVE_MOTOR;
const PreferencesTurnMotor PreferencesSwerve::TURN_MOTOR;


const double PreferencesControls::AXIS_DEADZONE = .2;
