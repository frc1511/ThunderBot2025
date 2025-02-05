#include "GamEpiece/Calgae.h"

Calgae::Calgae() {
    doPersistentConfiguration();
}

Calgae::~Calgae() {
}

void Calgae::doPersistentConfiguration() {
    rev::spark::SparkMaxConfig leftSparkMaxConfig {};
    rev::spark::SparkMaxConfig rightSparkMaxConfig {};
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

void Calgae::resetToMatchMode(Component::MatchMode lastMode, Component::MatchMode mode) {
    switch (mode) {
    case Component::MatchMode::DISABLED:
        motorSpeed = MotorSpeed::kSTOPPED;
        lastGamepieceState = lastGamepieceState::kHAD_NONE;
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
        motorSpeed = MotorSpeed::kSTOPPED;
        lastGamepieceState = lastGamepieceState::kHAD_NONE;
        stopMotors();
        break;
    }
}

void Calgae::sendFeedback() {
    frc::SmartDashboard::PutNumber ("Left SparkMax Speed -1 to 1"         , leftSparkMax.Get()                      );
    frc::SmartDashboard::PutNumber ("Left SparkMax Temp C"                , leftSparkMax.GetMotorTemperature()      );
    frc::SmartDashboard::PutNumber ("Left SparkMax Rotation"              , leftSparkMax.GetEncoder().GetPosition() );
    frc::SmartDashboard::PutNumber ("Left SparkMax Speed Vel"             , leftSparkMax.GetEncoder().GetVelocity() );
    frc::SmartDashboard::PutNumber ("Right SparkMax Speed -1 to 1"        , rightSparkMax.Get()                     );
    frc::SmartDashboard::PutNumber ("Right SparkMax Temp C"               , rightSparkMax.GetMotorTemperature()     );
    frc::SmartDashboard::PutNumber ("Right SparkMax Position"             , rightSparkMax.GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber ("Right SparkMax Speed Vel"            , rightSparkMax.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutBoolean("Coral Retroreflective Raw"           , coralRetroreflective.Get()              );
    frc::SmartDashboard::PutBoolean("Coral Retroreflective"               , coralRetroreflectiveTripped()           );
    frc::SmartDashboard::PutBoolean("Algae Retroreflective Raw"           , algaeRetroreflective.Get()              );
    frc::SmartDashboard::PutBoolean("Algae Retroreflective"               , algaeRetroreflectiveTripped()           );
    frc::SmartDashboard::PutString ("Last Gamepiece"                      , lastGamepieceStateToString()            );
}

void Calgae::process() {    
    Calgae::GamepieceState currentGamepieceState = updateGamepieceState();

    motorSpeed = MotorSpeed::kSTOPPED; // In case we make it through the below logic without getting a speed

    if (currentGamepieceState == GamepieceState::kNO_GP) { // If we don't have a gamepiece, so intake or nothing
        switch (motorMode) {
            case MotorModes::kNONE: // If we aren't being told to move motors
                motorSpeed = MotorSpeed::kSTOPPED; // Set speed to stop
                // Uh Oh! These will only be false if we shot it out (or manually reset), so the gamepiece was removed without us shooting it
                if ((lastGamepieceState == lastGamepieceState::kHAD_CORAL) || 
                     lastGamepieceState == lastGamepieceState::kHAD_ALGAE) 
                { // If we lost the gamepiece
                    motorSpeed = MotorSpeed::kREGRAB;
                }
                break;
            case MotorModes::kCORAL_INTAKE: // If we are told to intake coral
                motorSpeed = MotorSpeed::kCORAL; // Set speed to coral
                break;
            case MotorModes::kALGAE_INTAKE: // If we are told to intake algae
                motorSpeed = MotorSpeed::kALGAE; // Set speed to algae
                break;
            case MotorModes::kSHOOT: // If we are told to shoot, do it even though we don't have anything sensed, in case we lose detection early while shooting.
                
                if (lastGamepieceState == lastGamepieceState::kHAD_CORAL) { // If we had coral
                    motorSpeed = MotorSpeed::kCORAL;
                } else if (lastGamepieceState == lastGamepieceState::kHAD_ALGAE) { // If we had algae
                    motorSpeed = MotorSpeed::kALGAE;
                }
                runMotors(presetShooterSpeeds[motorSpeed]);
                return; // Return early so we don't intake further down

            case MotorModes::kDONE_SHOOTING:
                lastGamepieceState = lastGamepieceState::kHAD_NONE;
                break;
            default:
                motorSpeed = MotorSpeed::kSTOPPED; // Set the motors to stop because we shouldn't be intaking
                break;
        }

        runMotors(presetIntakeSpeeds[motorSpeed]);  // Run motors in at the speed ^
        return;

    } else if ((currentGamepieceState == GamepieceState::kHAS_CORAL) || 
               (currentGamepieceState == GamepieceState::kHAS_ALGAE)) 
        { // If we have Coral/Algae, so shoot or nothing
        if (currentGamepieceState == GamepieceState::kHAS_CORAL) { // We have coral
            lastGamepieceState = lastGamepieceState::kHAD_CORAL;
        }
        if (currentGamepieceState == GamepieceState::kHAS_ALGAE) { // We have algae
            lastGamepieceState = lastGamepieceState::kHAD_ALGAE;
        }
        switch (motorMode) {
            case MotorModes::kNONE: // If we aren't being told to move motors
                motorSpeed = MotorSpeed::kSTOPPED; // Set speed to stop
                break;
            case MotorModes::kSHOOT: // If we are told to shoot
                if (currentGamepieceState == GamepieceState::kHAS_CORAL) { // If we have coral
                    motorSpeed = MotorSpeed::kCORAL; // Set speed to coral
                } else if (currentGamepieceState == GamepieceState::kHAS_ALGAE) { // If we have algae
                    motorSpeed = MotorSpeed::kALGAE; // Set speed to Algae
                }
                break;
            case MotorModes::kDONE_SHOOTING:
                printf("Invalid GP State: Has GP and Done Shooting\n");
                lastGamepieceState = lastGamepieceState::kHAD_NONE;
                motorSpeed = MotorSpeed::kSTOPPED;
                break;
            default:
                motorSpeed = MotorSpeed::kSTOPPED; // Set the motors to stop because we shouldn't be intaking
                break;
        }

        runMotors(presetShooterSpeeds[motorSpeed]);  // Run motors out at the speed ^
        return;
    } else if (motorMode == MotorModes::kSHOOT_OVERRIDE) { // For when the sensor is broken
        motorSpeed = MotorSpeed::kALGAE; // Stronger to accomodate for algae because we can't tell what we have
        runMotors(presetShooterSpeeds[motorSpeed]);
    }
}

void Calgae::setMotorMode(Calgae::MotorModes mode) {
    motorMode = mode;
}

void Calgae::resetHadGamepiece() {
    lastGamepieceState = lastGamepieceState::kHAD_NONE;
}

bool Calgae::coralRetroreflectiveTripped() {
    return !coralRetroreflective.Get();
}

bool Calgae::algaeRetroreflectiveTripped() {
    return algaeRetroreflective.Get();
}

void Calgae::stopMotors() {
    leftSparkMax.Set(0);
    rightSparkMax.Set(0);
}

void Calgae::runMotors(double speed) {
    leftSparkMax.Set(speed);
    rightSparkMax.Set(speed);
}

Calgae::GamepieceState Calgae::updateGamepieceState() {
    enum Calgae::GamepieceState currentGamepieceState = GamepieceState::kNO_GP;
    if (algaeRetroreflectiveTripped() && coralRetroreflectiveTripped()) {
        printf("SENSOR ERROR: Coral and Algae Triggered Together\n");
        currentGamepieceState = GamepieceState::kHAS_ALGAE; // Prioritize algae for faster size
    } else if (algaeRetroreflectiveTripped()) {
        currentGamepieceState = GamepieceState::kHAS_ALGAE;
    } else if (coralRetroreflectiveTripped()) {
        currentGamepieceState = GamepieceState::kHAS_CORAL;
    }
    return currentGamepieceState;
}

std::string Calgae::lastGamepieceStateToString() {
    switch (lastGamepieceState) {
    case lastGamepieceState::kHAD_NONE: return "None";
    case lastGamepieceState::kHAD_CORAL: return "Algae";
    case lastGamepieceState::kHAD_ALGAE: return "Coral";
    default: return "Error reading lastGamepieceState";
    }
}