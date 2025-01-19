#include <Gamepiece.h>

Gamepiece::Gamepiece() {
    leftSparkMaxConfig.Inverted(true);
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
       rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, 
       rev::spark::SparkMax::PersistMode::kNoPersistParameters
    );
    rightSparkMax.Configure(
        rightSparkMaxConfig, 
        rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
        rev::spark::SparkMax::PersistMode::kPersistParameters
    );
}

Gamepiece::~Gamepiece() {
}

void Gamepiece::doPersistentConfiguration() {
    leftSparkMaxConfig.Inverted(true);
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
    switch (motorMode) { // motorMode is set by setMotorMode, which is called by controls
        case MotorModes::kNONE: 
            motorSpeed = MotorSpeeds::kSTOPPED;
            stopMotors();
            break;
        case MotorModes::kCORAL_INTAKE:
            motorSpeed = MotorSpeeds::kCORAL;
            if (!coralRetroreflectiveTripped()) {
                runMotors(presetIntakeSpeeds[motorSpeed]);
            } else {
                stopMotors();
            }
            break;
        case MotorModes::kCORAL_SHOOT:
            motorSpeed = MotorSpeeds::kCORAL;
            runMotors(presetShooterSpeeds[motorSpeed]);
            break;
        case MotorModes::kALGAE_INTAKE:
            motorSpeed = MotorSpeeds::kALGAE;
            if (!algaeRetroreflectiveTripped()) {
                runMotors(presetIntakeSpeeds[motorSpeed]);
            } else {
                stopMotors();
            }
            break;
        case MotorModes::kALGAE_SHOOT:
            motorSpeed = MotorSpeeds::kALGAE;
            runMotors(presetShooterSpeeds[motorSpeed]);
            break;
        default: // Just in case
            motorSpeed = MotorSpeeds::kSTOPPED;
            stopMotors();
            break;
    };
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