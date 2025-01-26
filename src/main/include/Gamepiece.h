#pragma once

#include "Basic/Component.h"
#include "Basic/IOMap.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/ClosedLoopConfig.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Gamepiece : public Component {
  public:
    Gamepiece();
    ~Gamepiece();

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
    void setMotorMode(Gamepiece::MotorModes mode);

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
    Gamepiece::MotorModes motorMode = MotorModes::kNONE;

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
    enum Gamepiece::lastGamepieceState lastGamepieceState = lastGamepieceState::kHAD_NONE;

    /**
     * Return the current sensed state of the gamepieces (which one we have)
     */
    Gamepiece::GamepieceState updateGamepieceState();

    /**
     * The possible speed settings (does not differentiate intake versus shoot, rather, it differentiates based on gamepiece)
     */
    enum MotorSpeed {
        kSTOPPED,
        kCORAL,
        kALGAE,
        kREGRAB,
        kMAX
    };

    /**
     * The variable storing what our current motor speed setting is based on MotorSpeeds
     */
    Gamepiece::MotorSpeed motorSpeed = MotorSpeed::kSTOPPED;

    /**
     * The set of intake speeds
     */
    double presetIntakeSpeeds [MotorSpeed::kMAX] = {
        0.0,
        0.75, // Coral
        1, // Algae
        .5 // Regrab
    };

    /**
     * The set of shooter speeds
     */
    double presetShooterSpeeds [MotorSpeed::kMAX] = {
        0.0,
        -0.75, // Coral
        -1 // Algae
    };

    rev::spark::SparkMax leftSparkMax {CAN_LEFT_CALGAE, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax rightSparkMax {CAN_RIGHT_CALGAE, rev::spark::SparkLowLevel::MotorType::kBrushless};
    // rev::spark::SparkClosedLoopController leftPidController = leftSparkMax.GetClosedLoopController(); // No PID Yet (also this probably won't be what we use , I think we're doing PWM)
    // rev::spark::SparkClosedLoopController rightPidController = rightSparkMax.GetClosedLoopController();

    frc::DigitalInput coralRetroreflective {DIO_CORAL_RETROREFLECTIVE};
    frc::DigitalInput algaeRetroreflective {DIO_ALGAE_RETROREFLECTIVE};
};