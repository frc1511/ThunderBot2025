#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

#include <frc/PWM.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Calgae : public Component {
  public:
    Calgae();
    ~Calgae();

    void doPersistentConfiguration();

    void resetToMatchMode(Component::MatchMode lastMode, Component::MatchMode mode);

    void sendFeedback();

    void process();

    /**
     * The set of possible motor modes
     */
    enum MotorModes {
        kNONE,
        kCORAL_INTAKE,
        kALGAE_INTAKE,
        kSHOOT,
        kSHOOT_OVERRIDE,
        kDONE_SHOOTING
    };

    /**
     * Set the motor modes based on controls
     */
    void setMotorMode(Calgae::MotorModes mode);

    /**
     * Reset the had gamepiece states to false (Used by Controls for a manual reset)
     */
    void resetHadGamepiece();

  private:
    /**
     * Get calculated state (non-raw) of the Coral Retroreflective sensor
     */
    bool coralRetroreflectiveTripped();

    /**
     * Get calculated state (non-raw) of the Algae Retroreflective sensor
     */
    bool algaeRetroreflectiveTripped();

    /**
     * Stop all motors
     */
    void stopMotors();

    /**
     * Set the speed of the motors (-1 to 1)
     */
    void runMotors(double speed);

    /**
     * Convert the lastGamepieceState to a string for smartdashboard purposes
     */
    std::string lastGamepieceStateToString();

    /**
     * Mode set by controls through setMotorMode()
     */
    Calgae::MotorModes motorMode = MotorModes::kNONE;

    /**
     * Possible states for the gamepiece (based on the sensors)
     */
    enum GamepieceState {
        kNO_GP,
        kHAS_CORAL,
        kHAS_ALGAE
    };

    enum lastGamepieceState {
        kHAD_NONE,
        kHAD_CORAL,
        kHAD_ALGAE, 
        kSENSOR_BROKEN
    };

    /**
     * The variable storing what our last gamepiece state was
     */
    Calgae::lastGamepieceState lastGamepieceState = lastGamepieceState::kHAD_NONE;

    /**
     * Return the current sensed state of the gamepieces (which one we have)
     */
    Calgae::GamepieceState updateGamepieceState();

    /**
     * The possible speed settings (does not differentiate intake versus shoot, rather, it differentiates based on gamepiece)
     */
    enum MotorSpeed {
        kSTOPPED,
        kCORAL,
        kALGAE,
        kREGRAB,
        _enum_MAX
    };

    /**
     * The variable storing what our current motor speed setting is based on MotorSpeeds
     */
    Calgae::MotorSpeed motorSpeed = MotorSpeed::kSTOPPED;

    /**
     * The set of intake speeds
     */
    double presetIntakeSpeeds [MotorSpeed::_enum_MAX] = {
        CALGAE_PREFERENCE.MOTOR_SPEED_STOPPED, // Stopped
        CALGAE_PREFERENCE.MOTOR_SPEED_INTAKE_CORAL, // Coral
        CALGAE_PREFERENCE.MOTOR_SPEED_INTAKE_ALGAE, // Algae
        CALGAE_PREFERENCE.MOTOR_SPEED_INTAKE_REGRAB // Regrab
    };

    /**
     * The set of shooter speeds
     */
    double presetShooterSpeeds [MotorSpeed::_enum_MAX] = {
        CALGAE_PREFERENCE.MOTOR_SPEED_STOPPED, // Stopped
        CALGAE_PREFERENCE.MOTOR_SPEED_SHOOT_CORAL, // Coral
        CALGAE_PREFERENCE.MOTOR_SPEED_SHOOT_ALGAE, // Algae
    };

    frc::PWM rightMotor {PWM_RIGHT_CALGAE};
    frc::PWM leftMotor {PWM_LEFT_CALGAE};

    frc::DigitalInput coralRetroreflective {DIO_CORAL_RETROREFLECTIVE};
    frc::DigitalInput algaeRetroreflective {DIO_ALGAE_RETROREFLECTIVE};
};