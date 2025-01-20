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

    void resetToMatchMode(MatchMode mode);

    void sendFeedback();

    void process();

    /**
     * The set of possible motor modes
     */
    enum MotorModes {
        kNONE,
        kCORAL_INTAKE,
        kALGAE_INTAKE,
        kSHOOT
    };

    /**
     * Set the motor modes based on controls
     */
    void setMotorMode(Gamepiece::MotorModes mode);

    /**
     * Reset the had gamepiece states to false
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
     * Manually Set which gamepiece we have, if any
     */
    void updateGamepieceState();

    /**
     * Mode set by controls through setMotorMode()
     */
    Gamepiece::MotorModes motorMode = MotorModes::kNONE;

    /**
     * Possible states for the gamepiece (based on the sensors)
     */
    enum GamepieceStates {
        kNO_GP,
        kHAS_CORAL,
        kHAS_ALGAE
    };

    /**
     * The variable storing what our current gamepiece state is
     */
    enum Gamepiece::GamepieceStates currentGamepieceState = GamepieceStates::kNO_GP;

    /**
     * The variable storing whether or not we had a coral (used to recapture dropped gamepieces)
     */
    bool hadCoral = false;
    /**
     * The variable storing whether or not we had an algae (used to recapture dropped gamepieces)
     */
    bool hadAlgae = false;

    /**
     * The possible speed settings (does not differentiate intake versus shoot, rather, it differentiates based on gamepiece)
     */
    enum MotorSpeeds {
        kSTOPPED,
        kCORAL,
        kALGAE,
        kMAX
    };

    /**
     * The variable storing what our current motor speed setting is based on MotorSpeeds
     */
    Gamepiece::MotorSpeeds motorSpeed = MotorSpeeds::kSTOPPED;

    /**
     * The set of intake speeds
     */
    double presetIntakeSpeeds [MotorSpeeds::kMAX] = {
        0.0,
        0.75, // Coral
        1 // Algae
    };

    /**
     * The set of shooter speeds
     */
    double presetShooterSpeeds [MotorSpeeds::kMAX] = {
        0.0,
        -0.75, // Coral
        -1 // Algae
    };

    rev::spark::SparkMax leftSparkMax {CAN_LEFT_CALGAE, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMaxConfig leftSparkMaxConfig {};
    rev::spark::SparkMax rightSparkMax {CAN_RIGHT_CALGAE, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMaxConfig rightSparkMaxConfig {};
    // rev::spark::SparkClosedLoopController leftPidController = leftSparkMax.GetClosedLoopController(); // No PID Yet (also this probably won't be what we use , I think we're doing PWM)
    // rev::spark::SparkClosedLoopController rightPidController = rightSparkMax.GetClosedLoopController();

    frc::DigitalInput coralRetroreflective {DIO_CORAL_RETROREFLECTIVE};
    frc::DigitalInput algaeRetroreflective {DIO_ALGAE_RETROREFLECTIVE};
};