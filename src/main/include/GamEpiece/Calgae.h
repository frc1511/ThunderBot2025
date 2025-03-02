#pragma once

#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Basic/Component.h"
#include "Basic/IOMap.h"
#include "Preferences.h"

class Calgae : public Component {
  public:
    Calgae();
    ~Calgae();

    void doConfiguration(bool persist);

    void resetToMatchMode(Component::MatchMode lastMode, Component::MatchMode mode);

    void sendFeedback();

    void process();

    /**
     * The set of possible motor modes
     */
    enum MotorModes {
        kSTOP,
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

    bool hasGamepiece();
    bool hasCoral();
    bool hasAlgae();

    /**
     * Get calculated state (non-raw) of the Coral Retroreflective sensor
     */
    bool coralRetroreflectiveTripped();

    /**
     * Get calculated state (non-raw) of the Algae Retroreflective sensor
     */
    bool algaeRetroreflectiveTripped();

    bool isShootDone();

    void autoShoot();

    bool isAutoShooting = false;
  private:
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
     * Convert the motorSpeed to a string for smartdashboard purposes
     */
    std::string motorSpeedToString();

    /**
     * Mode set by controls through setMotorMode()
     */
    Calgae::MotorModes motorMode = MotorModes::kSTOP;

    /**
     * Possible states for the gamepiece (based on the sensors)
     */
    enum GamepieceState {
        kNONE,
        kCORAL,
        kALGAE, 
        kSENSOR_BROKEN
    };

    /**
     * The variable storing what our last gamepiece state was
     */
    Calgae::GamepieceState lastGamepieceState = GamepieceState::kNONE;

    /**
     * Return the current sensed state of the gamepieces (which one we have)
     */
    void updateGamepieceState();

    /**
     * The possible speed settings (does not differentiate intake versus shoot, rather, it differentiates based on gamepiece)
     */
    enum MotorSpeed {
        kSTOPPED,
        kCORAL_SPEED,
        kALGAE_SPEED,
        kREGRAB_SPEED,
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
        PreferencesCalgae::MOTOR_SPEED_STOPPED, // Stopped
        PreferencesCalgae::MOTOR_SPEED_INTAKE_CORAL, // Coral
        PreferencesCalgae::MOTOR_SPEED_INTAKE_ALGAE, // Algae
        PreferencesCalgae::MOTOR_SPEED_INTAKE_REGRAB // Regrab
    };

    /**
     * The set of shooter speeds
     */
    double presetShooterSpeeds [MotorSpeed::_enum_MAX] = {
        PreferencesCalgae::MOTOR_SPEED_STOPPED, // Stopped
        PreferencesCalgae::MOTOR_SPEED_SHOOT_CORAL, // Coral
        PreferencesCalgae::MOTOR_SPEED_SHOOT_ALGAE, // Algae
    };

    enum Calgae::GamepieceState currentGamepieceState = GamepieceState::kNONE;

    frc::PWMSparkMax motor {PWM_CALGAE};

    frc::DigitalInput coralRetroreflective {DIO_CORAL_RETROREFLECTIVE};
    frc::DigitalInput algaeRetroreflective {DIO_ALGAE_RETROREFLECTIVE};

    bool isAuto = false;

    frc::Timer regrabTimeout;
    frc::Timer shootTimer;
};