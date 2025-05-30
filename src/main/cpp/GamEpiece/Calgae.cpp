#include "GamEpiece/Calgae.h"

Calgae::Calgae() {
    doConfiguration(false);
}

Calgae::~Calgae() { }

void Calgae::doConfiguration(bool persist) { }

void Calgae::resetToMatchMode(Component::MatchMode lastMode, Component::MatchMode mode) {
    stopMotors();

    isAuto = false;
    motorMode = Calgae::MotorModes::kSTOP;
    switch (mode) {
        case Component::MatchMode::DISABLED:
            stopMotors();
            break;
        case Component::MatchMode::AUTO:
            updateGamepieceState();
            isAuto = true;
            lastGamepieceState = GamepieceState::kCORAL; // "I don't see why not." -Noah, "Huh?" -Trevor
            break;
        case Component::MatchMode::TELEOP:
            updateGamepieceState();
            lastGamepieceState = currentGamepieceState;
            break;
        default:
            motorSpeed = MotorSpeed::kSTOPPED;
            lastGamepieceState = GamepieceState::kNONE;
            stopMotors();
            break;
    }
}

void Calgae::sendFeedback() {
    frc::SmartDashboard::PutBoolean("Calgae Coral Retroreflective Raw"           , coralRetroreflective.Get()              );
    frc::SmartDashboard::PutBoolean("Calgae Coral Retroreflective"               , coralRetroreflectiveTripped()           );
    frc::SmartDashboard::PutBoolean("Calgae Algae Retroreflective Raw"           , algaeRetroreflective.Get()              );
    frc::SmartDashboard::PutBoolean("Calgae Algae Retroreflective"               , algaeRetroreflectiveTripped()           );
    frc::SmartDashboard::PutBoolean("Calgae Has Gamepiece"                       , hasGamepiece()                          );
    frc::SmartDashboard::PutString ("Calgae Last Gamepiece"                      , lastGamepieceStateToString()            );
    frc::SmartDashboard::PutBoolean("Calgae Algae Retroreflective"               , algaeRetroreflectiveTripped()           );
    frc::SmartDashboard::PutString ("Calgae Motor Target Speed"                  , motorSpeedToString()                    );
    frc::SmartDashboard::PutNumber ("Calgae Motor Out Voltage"                   , motor.GetVoltage().value()              );
    frc::SmartDashboard::PutBoolean("Calgae Is Shoot Done"                       , isShootDone()                           );
    frc::SmartDashboard::PutBoolean("Calgae Is Auto"                             , isAuto);
    frc::SmartDashboard::PutBoolean("Calgae Is Auto Intaking"                    , isAutoIntaking);
    frc::SmartDashboard::PutBoolean("Calgae Is Auto Shooting"                    , isAutoShooting);
    frc::SmartDashboard::PutString ("Calgae Motor Mode"                          , motorModeToString());
}

void Calgae::process() {
    updateGamepieceState();
    motorSpeed = MotorSpeed::kSTOPPED; // In case we make it through the below logic without getting a speed

    if (isAuto) {
        if (isShootDone()) {
            motorSpeed = MotorSpeed::kSTOPPED;
            motorMode = MotorModes::kSTOP;
            isAutoShooting = false;
            shootTimer.Stop();
            shootTimer.Reset();
            resetHadGamepiece();
            stopMotors();
            return;
        }
        if (intakeTimeout.Get() > 4_s && isAutoIntaking) {
            lastGamepieceState = GamepieceState::kALGAE;
            motorMode = MotorModes::kSTOP;
            intakeTimeout.Stop();
            intakeTimeout.Reset();
        }
    }

    if (currentGamepieceState == GamepieceState::kNONE) { // If we don't have a gamepiece, so intake or nothing
        switch (motorMode) {
            case MotorModes::kSTOP: // If we aren't being told to move motors
                motorSpeed = MotorSpeed::kSTOPPED; // Set speed to stop
                // Uh Oh! These will only be false if we shot it out (or manually reset), so the gamepiece was removed without us shooting it
                if ((lastGamepieceState == GamepieceState::kCORAL) || 
                     lastGamepieceState == GamepieceState::kALGAE) {
                 // If we lost the gamepiece
                    regrabTimeout.Start();
                    if (regrabTimeout.Get() >= 2_s) {
                        motorSpeed = MotorSpeed::kSTOPPED;
                        regrabTimeout.Reset();
                        resetHadGamepiece();
                    }
                    motorSpeed = MotorSpeed::kREGRAB_SPEED;
                }
                break;
            case MotorModes::kCORAL_INTAKE: // If we are told to intake coral
                motorSpeed = MotorSpeed::kCORAL_SPEED; // Set speed to coral
                break;
            case MotorModes::kALGAE_INTAKE: // If we are told to intake algae
                motorSpeed = MotorSpeed::kALGAE_SPEED; // Set speed to algae
                break;
            case MotorModes::kSHOOT: // If we are told to shoot, do it even though we don't have anything sensed, in case we lose detection early while shooting.
                if (lastGamepieceState == GamepieceState::kCORAL) { // If we had coral
                    motorSpeed = MotorSpeed::kCORAL_SPEED;
                } else if (lastGamepieceState == GamepieceState::kALGAE) { // If we had algae
                    motorSpeed = MotorSpeed::kALGAE_SPEED;
                } else {
                    motorSpeed = MotorSpeed::kALGAE_SPEED; // ALWAYS Shoot
                }
                runMotors(presetShooterSpeeds[motorSpeed]);
                return; // Return early so we don't intake further down
            case MotorModes::kDONE_SHOOTING:
                lastGamepieceState = GamepieceState::kNONE;
                motorSpeed = MotorSpeed::kSTOPPED;
                break;
            default:
                motorSpeed = MotorSpeed::kSTOPPED; // Set the motors to stop because we shouldn't be intaking
                break;
        }

        runMotors(presetIntakeSpeeds[motorSpeed]);  // Run motors in at the speed ^
        return;

    } else if ((currentGamepieceState == GamepieceState::kCORAL) || 
               (currentGamepieceState == GamepieceState::kALGAE)) {
         // If we have Coral/Algae, so shoot or nothing
        if (currentGamepieceState == GamepieceState::kCORAL) { // We have coral
            lastGamepieceState = GamepieceState::kCORAL;
            if (isAutoIntaking) {
                intakeTimeout.Stop();
                intakeTimeout.Reset();
                isAutoIntaking = false;
            }
        }

        if (currentGamepieceState == GamepieceState::kALGAE) { // We have algae
            lastGamepieceState = GamepieceState::kALGAE;
            if (isAutoIntaking) {
                intakeTimeout.Stop();
                intakeTimeout.Reset();
                isAutoIntaking = false;
            }
        }

        switch (motorMode) {
            case MotorModes::kSTOP: // If we aren't being told to move motors
                motorSpeed = MotorSpeed::kSTOPPED; // Set speed to stop
                break;
            case MotorModes::kSHOOT: // If we are told to shoot
                if (currentGamepieceState == GamepieceState::kCORAL) { // If we have coral
                    motorSpeed = MotorSpeed::kCORAL_SPEED; // Set speed to coral
                } else if (currentGamepieceState == GamepieceState::kALGAE) { // If we have algae
                    motorSpeed = MotorSpeed::kALGAE_SPEED; // Set speed to Algae
                } else {
                    motorSpeed = MotorSpeed::kALGAE_SPEED; // ALWAYS Shoot
                }
                break;
            case MotorModes::kDONE_SHOOTING:
                printf("Invalid GP State: Has GP and Done Shooting\n");
                lastGamepieceState = GamepieceState::kNONE;
                motorSpeed = MotorSpeed::kSTOPPED;
                break;
            default:
                motorSpeed = MotorSpeed::kSTOPPED; // Set the motors to stop because we shouldn't be intaking
                break;
        }

        runMotors(presetShooterSpeeds[motorSpeed]); // Run motors out at the speed ^
        return;
    }

    if (motorMode == MotorModes::kSHOOT_OVERRIDE) { // For when the sensor is broken
        motorSpeed = MotorSpeed::kALGAE_SPEED; // Stronger to accomodate for algae because we can't tell what we have
        runMotors(presetShooterSpeeds[motorSpeed]);
    }
}

void Calgae::setMotorMode(Calgae::MotorModes mode) {
    motorMode = mode;
}

void Calgae::resetHadGamepiece() {
    currentGamepieceState = GamepieceState::kNONE;
    lastGamepieceState = GamepieceState::kNONE;
}

bool Calgae::coralRetroreflectiveTripped() {
    return !coralRetroreflective.Get();
}

bool Calgae::algaeRetroreflectiveTripped() {
    return algaeRetroreflective.Get();
}

bool Calgae::isShootDone() {
    return shootTimer.Get() >= 0.5_s || (shootTimer.Get() >= 80_ms && !hasGamepiece());
}

void Calgae::autoShoot() {
    isAuto = true;
    isAutoShooting = true;
    shootTimer.Restart();
    setMotorMode(Calgae::MotorModes::kSHOOT);
}

// isCoral: True -> intake coral, isCoral: False -> intake algae
void Calgae::autoIntake(bool isCoral, bool shouldTimeout) {
    isAuto = true;
    isAutoIntaking = true;
    if (shouldTimeout) {
        intakeTimeout.Restart();
    }
    lastGamepieceState = GamepieceState::kNONE;
    if (isCoral) {
        setMotorMode(MotorModes::kCORAL_INTAKE);
    } else {
        setMotorMode(MotorModes::kALGAE_INTAKE);
    }
}
void Calgae::stopAutoIntake() {
    isAutoIntaking = false;
    setMotorMode(MotorModes::kSTOP);
}
void Calgae::stopMotors() {
    motor.StopMotor();
}

void Calgae::runMotors(double speed) {
    speed = std::clamp(speed, -1.0, 1.0);
    motor.Set(speed); // NOTE: + is for IN, - is for OUT
}

void Calgae::updateGamepieceState() {
    currentGamepieceState = GamepieceState::kNONE;
    if (algaeRetroreflectiveTripped() && coralRetroreflectiveTripped()) {
        printf("SENSOR ERROR: Coral and Algae Triggered Together\n");
        currentGamepieceState = GamepieceState::kALGAE; // Prioritize algae for faster size
    } else if (algaeRetroreflectiveTripped()) {
        currentGamepieceState = GamepieceState::kALGAE;
    } else if (coralRetroreflectiveTripped()) {
        currentGamepieceState = GamepieceState::kCORAL;
    }
}

bool Calgae::hasGamepiece() {
    switch (currentGamepieceState) {
    case kALGAE:
        return true;
    case kCORAL:
        return true;
    default:
        return false;
    }
}

bool Calgae::hasCoral() {
    return currentGamepieceState == Calgae::GamepieceState::kCORAL;
}

bool Calgae::hasAlgae() {
    return currentGamepieceState == Calgae::GamepieceState::kALGAE;
}

std::string Calgae::lastGamepieceStateToString() {
    switch (lastGamepieceState) {
        case GamepieceState::kNONE: return "None";
        case GamepieceState::kCORAL: return "Coral";
        case GamepieceState::kALGAE: return "Algae";
        default: return "Error reading lastGamepieceState";
    }
}

std::string Calgae::motorSpeedToString() {
    switch (motorSpeed) {
    case MotorSpeed::kSTOPPED: return "Stopped";
    case MotorSpeed::kCORAL_SPEED: return "Coral";
    case MotorSpeed::kALGAE_SPEED: return "Algae";
    case MotorSpeed::kREGRAB_SPEED: return "Regrab";
    default: return "Error reading motorSpeed";
    }
}

std::string Calgae::motorModeToString() {
    switch (motorMode) {
    case MotorModes::kSTOP:           return "Stop";
    case MotorModes::kALGAE_INTAKE:   return "Algae Intake";
    case MotorModes::kCORAL_INTAKE:   return "Coral Intake";
    case MotorModes::kDONE_SHOOTING:  return "Done SHooting";
    case MotorModes::kSHOOT_OVERRIDE: return "Shoot Override";
    case MotorModes::kSHOOT:          return "Shoot";
    default: return "Error reading motorMode";
    }
}