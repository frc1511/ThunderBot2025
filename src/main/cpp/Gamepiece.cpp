#include <Gamepiece.h>

Gamepiece::Gamepiece() {
    doPersistentConfiguration();
}

Gamepiece::~Gamepiece() {
}

void Gamepiece::doPersistentConfiguration() {
    leftSparkMaxConfig.Inverted(true);
    rightSparkMaxConfig.Inverted(false);
    leftSparkMaxConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    rightSparkMaxConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    // ↓ Wait until testing/implementing PID ↓
    // leftSparkMaxConfig.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    // rightSparkMaxConfig.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    // leftSparkMaxConfig.closedLoop.Pid(0.0, 0.0, 0.0);
    // rightSparkMaxConfig.closedLoop.Pid(0.0, 0.0, 0.0);

    leftSparkMax.Configure(
       leftSparkMaxConfig,
       rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, 
       rev::spark::SparkMax::PersistMode::kNoPersistParameters
    );
    rightSparkMax.Configure(
        rightSparkMaxConfig, 
        rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
        rev::spark::SparkMax::PersistMode::kPersistParameters
    );
}

void Gamepiece::resetToMatchMode(Component::MatchMode mode) {
    switch (mode)
    {
    case Component::MatchMode::DISABLED:
        motorSpeed = MotorSpeeds::kSTOPPED;
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
}

void Gamepiece::process() {
    updateGamepieceState();

    motorSpeed = MotorSpeeds::kSTOPPED; // In case we make it through the below logic without getting a speed

    // If we don't have a gamepiece, so intake
    if ((currentGamepieceState == GamepieceStates::kNONE)) {
        switch (motorMode) {
        case MotorModes::kNONE: // If we aren't being told to move motors
            motorSpeed = MotorSpeeds::kSTOPPED; // Set speed to stop
            break;
        case MotorModes::kCORAL_INTAKE: // If we are told to intake coral
            motorSpeed = MotorSpeeds::kCORAL; // Set speed to coral
            break;
        case MotorModes::kALGAE_INTAKE: // If we are told to intake algae
            motorSpeed = MotorSpeeds::kALGAE; // Set speed to algae
            break;
        default:
            motorSpeed = MotorSpeeds::kSTOPPED; // Set the motors to stop because we don't have anything to shoot
            break;
        }

        runMotors(presetIntakeSpeeds[motorSpeed]);  // Run motors in at the speed ^

    // If we have Coral
    } else if (currentGamepieceState == GamepieceStates::kHAS_CORAL) {
        switch (motorMode) {
        case MotorModes::kNONE: // If we aren't being told to move motors
            motorSpeed = MotorSpeeds::kSTOPPED; // Set speed to stop
            break;
        case MotorModes::kSHOOT: // If we are told to shoot
            motorSpeed = MotorSpeeds::kCORAL; // Set speed to coral
            break;
        default:
            motorSpeed = MotorSpeeds::kSTOPPED; // Set the motors to stop because we shouldn't be intaking
            break;
        }

        runMotors(presetShooterSpeeds[motorSpeed]);  // Run motors out at the speed ^

    // If we have Algae
    } else if (currentGamepieceState == GamepieceStates::kHAS_ALGAE) {
        switch (motorMode) {
        case MotorModes::kNONE: // If we aren't being told to move motors
            motorSpeed = MotorSpeeds::kSTOPPED; // Set speed to stop
            break;
        case MotorModes::kSHOOT: // If we are told to shoot
            motorSpeed = MotorSpeeds::kALGAE; // Set speed to algae
            break;
        default:
            motorSpeed = MotorSpeeds::kSTOPPED; // Set the motors to stop because we shouldn't be intaking
            break;
        }

        runMotors(presetShooterSpeeds[motorSpeed]); // Run motors out at the speed ^
    }
}

void Gamepiece::setMotorMode(Gamepiece::MotorModes mode) {
    motorMode = mode;
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

void Gamepiece::updateGamepieceState() {
    currentGamepieceState = GamepieceStates::kNONE;
    if (algaeRetroreflectiveTripped()) {
        currentGamepieceState = GamepieceStates::kHAS_ALGAE;
    } else if (coralRetroreflectiveTripped()) {
        currentGamepieceState = GamepieceStates::kHAS_CORAL;
    }
}