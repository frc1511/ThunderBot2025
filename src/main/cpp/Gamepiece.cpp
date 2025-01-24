#include "Gamepiece.h"

Gamepiece::Gamepiece() {
    doPersistentConfiguration();
}

Gamepiece::~Gamepiece() {
}

void Gamepiece::doPersistentConfiguration() {
    leftSparkMaxConfig.Inverted(false);
    rightSparkMaxConfig.Inverted(true);
    leftSparkMaxConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    rightSparkMaxConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    // ↓ Wait until testing/implementing PID ↓
    // leftSparkMaxConfig.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    // rightSparkMaxConfig.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    // leftSparkMaxConfig.closedLoop.Pid(0.0, 0.0, 0.0);
    // rightSparkMaxConfig.closedLoop.Pid(0.0, 0.0, 0.0);

    leftSparkMax.Configure(
       leftSparkMaxConfig,
       rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
       rev::spark::SparkMax::PersistMode::kPersistParameters
    );
    rightSparkMax.Configure(
        rightSparkMaxConfig, 
        rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
        rev::spark::SparkMax::PersistMode::kPersistParameters
    );
}

void Gamepiece::resetToMatchMode(Component::MatchMode lastMode, Component::MatchMode mode) {
    switch (mode) {
    case Component::MatchMode::DISABLED:
        motorSpeed = MotorSpeeds::kSTOPPED;
        lastGamepieceState = lastGamepieceStates::kHAD_NONE;
        stopMotors();
        break;
    case Component::MatchMode::AUTO:
        /* code */
        break;
    case Component::MatchMode::TELEOP:
        /* code */
        break;
    case Component::MatchMode::TEST:
        /* code */
        break;
    
    default:
        motorSpeed = MotorSpeeds::kSTOPPED;
        lastGamepieceState = lastGamepieceStates::kHAD_NONE;
        stopMotors();
        break;
    }
}

void Gamepiece::sendFeedback() {
    frc::SmartDashboard::PutNumber ("Left SparkMax Speed -1 to 1"         , leftSparkMax.Get()                      );
    frc::SmartDashboard::PutNumber ("Left SparkMax Temp C*"               , leftSparkMax.GetMotorTemperature()      );
    frc::SmartDashboard::PutNumber ("Left SparkMax Rotation"              , leftSparkMax.GetEncoder().GetPosition() );
    frc::SmartDashboard::PutNumber ("Left SparkMax Speed Vel"             , leftSparkMax.GetEncoder().GetVelocity() );
    frc::SmartDashboard::PutNumber ("Right SparkMax Speed -1 to 1"        , rightSparkMax.Get()                     );
    frc::SmartDashboard::PutNumber ("Right SparkMax Temp C*"              , rightSparkMax.GetMotorTemperature()     );
    frc::SmartDashboard::PutNumber ("Right SparkMax Position"             , rightSparkMax.GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber ("Right SparkMax Speed Vel"            , rightSparkMax.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutBoolean("Coral Retroreflective Raw"           , coralRetroreflective.Get()              );
    frc::SmartDashboard::PutBoolean("Coral Retroreflective"               , coralRetroreflectiveTripped()           );
    frc::SmartDashboard::PutBoolean("Algae Retroreflective Raw"           , algaeRetroreflective.Get()              );
    frc::SmartDashboard::PutBoolean("Algae Retroreflective"               , algaeRetroreflectiveTripped()           );
    frc::SmartDashboard::PutString ("Last Gamepiece"                      , lastGamepieceStateToString()            );
}

void Gamepiece::process() {    
    enum Gamepiece::GamepieceStates currentGamepieceState = updateGamepieceState();

    motorSpeed = MotorSpeeds::kSTOPPED; // In case we make it through the below logic without getting a speed

    if ((currentGamepieceState == GamepieceStates::kNO_GP)) { // If we don't have a gamepiece, so intake or nothing
        switch (motorMode) {
        case MotorModes::kNONE: // If we aren't being told to move motors
            motorSpeed = MotorSpeeds::kSTOPPED; // Set speed to stop
            // Uh Oh! These will only be false if we shot it out (or manually reset), so the gamepiece was removed without us shooting it
            if (lastGamepieceState == lastGamepieceStates::kHAD_CORAL || lastGamepieceState == lastGamepieceStates::kHAD_ALGAE) { // If we lost the coral
                motorSpeed = MotorSpeeds::kREGRAB;
            }
            break;
        case MotorModes::kCORAL_INTAKE: // If we are told to intake coral
            motorSpeed = MotorSpeeds::kCORAL; // Set speed to coral
            break;
        case MotorModes::kALGAE_INTAKE: // If we are told to intake algae
            motorSpeed = MotorSpeeds::kALGAE; // Set speed to algae
            break;
        case MotorModes::kSHOOT: // If we are told to shoot, do it even though we don't have anything sensed in case we lose detection early while shooting
            if (lastGamepieceState == lastGamepieceStates::kHAD_CORAL) { // If we had coral
                motorSpeed = MotorSpeeds::kCORAL;
            } else if (lastGamepieceState == lastGamepieceStates::kHAD_ALGAE) { // If we had algae
                motorSpeed = MotorSpeeds::kALGAE;
            } else {
                motorSpeed = MotorSpeeds::kSTOPPED; // For Safety!
            }
            runMotors(presetShooterSpeeds[motorSpeed]);
            return; // Return early so we don't intake further down
            break; // Don't know if we actually need this because it's past the return
        default:
            motorSpeed = MotorSpeeds::kSTOPPED; // Set the motors to stop just in case
            break;
        }

        runMotors(presetIntakeSpeeds[motorSpeed]);  // Run motors in at the speed ^
        return;

    } else if ((currentGamepieceState == GamepieceStates::kHAS_CORAL) || (currentGamepieceState == GamepieceStates::kHAS_ALGAE)) { // If we have Coral/Algae, so shoot or nothing
        if (currentGamepieceState == GamepieceStates::kHAS_CORAL) { // We have coral
            lastGamepieceState = lastGamepieceStates::kHAD_CORAL;
        }
        if (currentGamepieceState == GamepieceStates::kHAS_ALGAE) { // We have algae
            lastGamepieceState = lastGamepieceStates::kHAD_ALGAE;
        }
        switch (motorMode) {
        case MotorModes::kNONE: // If we aren't being told to move motors
            motorSpeed = MotorSpeeds::kSTOPPED; // Set speed to stop
            break;
        case MotorModes::kSHOOT: // If we are told to shoot
            if (currentGamepieceState == GamepieceStates::kHAS_CORAL) { // If we have coral
                motorSpeed = MotorSpeeds::kCORAL; // Set speed to coral
            } else if (currentGamepieceState == GamepieceStates::kHAS_ALGAE) { // If we have algae
                motorSpeed = MotorSpeeds::kALGAE; // Set speed to Algae
            }
            break;
        case MotorModes::kDONE_SHOOTING:
            if (lastGamepieceState == lastGamepieceStates::kHAD_CORAL) lastGamepieceState = lastGamepieceStates::kHAD_CORAL; // We assume we no longer have coral because we're done shooting it out
            else if (lastGamepieceState == lastGamepieceStates::kHAD_ALGAE) lastGamepieceState = lastGamepieceStates::kHAD_ALGAE; // We assume we no longer have algae because we're done shooting it out
            break;
        default:
            motorSpeed = MotorSpeeds::kSTOPPED; // Set the motors to stop because we shouldn't be intaking
            break;
        }

        runMotors(presetShooterSpeeds[motorSpeed]);  // Run motors out at the speed ^
        return;
    } else if (motorMode == MotorModes::kSHOOT_OVERRIDE) { // For when the sensor is broken
        motorSpeed = MotorSpeeds::kALGAE; // Stronger to accomodate for algae because we can't tell what we have
        runMotors(presetShooterSpeeds[motorSpeed]);
    }
}

void Gamepiece::setMotorMode(Gamepiece::MotorModes mode) {
    motorMode = mode;
}

void Gamepiece::resetHadGamepiece() {
    lastGamepieceState = lastGamepieceStates::kHAD_NONE;
}

bool Gamepiece::coralRetroreflectiveTripped() {
    return !coralRetroreflective.Get();
}

bool Gamepiece::algaeRetroreflectiveTripped() {
    return algaeRetroreflective.Get();
}

void Gamepiece::stopMotors() {
    leftSparkMax.Set(presetIntakeSpeeds[MotorSpeeds::kSTOPPED]);
    rightSparkMax.Set(presetIntakeSpeeds[MotorSpeeds::kSTOPPED]);    
}

void Gamepiece::runMotors(double speed) {
    leftSparkMax.Set(speed);
    rightSparkMax.Set(speed);
}

Gamepiece::GamepieceStates Gamepiece::updateGamepieceState() {
    enum Gamepiece::GamepieceStates currentGamepieceState = GamepieceStates::kNO_GP;
    if (algaeRetroreflectiveTripped()) {
        currentGamepieceState = GamepieceStates::kHAS_ALGAE;
    } else if (coralRetroreflectiveTripped()) {
        currentGamepieceState = GamepieceStates::kHAS_CORAL;
    }
    return currentGamepieceState;
}

std::string Gamepiece::lastGamepieceStateToString() {
    switch (lastGamepieceState) {
    case lastGamepieceStates::kHAD_NONE: return "None";
    case lastGamepieceStates::kHAD_CORAL: return "Algae";
    case lastGamepieceStates::kHAD_ALGAE: return "Coral";
    default: return "Error reading lastGamepieceState";
    }
}