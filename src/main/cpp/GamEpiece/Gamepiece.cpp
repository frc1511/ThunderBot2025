#include "GamEpiece/Gamepiece.h"

Gamepiece::Gamepiece(Calgae *calgae_, Wrist *wrist_, Elevator *elevator_)
: calgae(calgae_),
  wrist(wrist_),
  elevator(elevator_)
{
    elevator->wristExists = (wrist != nullptr);
}

void Gamepiece::process() {
    if (wrist != nullptr)
        elevator->wristIsUnsafe = wrist->wristIsUnsafe();
    moveToTarget();
}

void Gamepiece::doConfiguration(bool persist) { }

void Gamepiece::resetToMatchMode(Component::MatchMode lastMode, Component::MatchMode mode) {
    if (mode == Component::MatchMode::AUTO) {
        moveToPreset(Preset::kTRANSIT);
    }
}

void Gamepiece::sendFeedback() {
    frc::SmartDashboard::PutBoolean("Calgae Autopilot",    calgaeAutopilot);
    frc::SmartDashboard::PutBoolean("Wrist Autopilot",     wristAutopilot);
    frc::SmartDashboard::PutBoolean("Elevator Autopilot",  elevatorAutopilot);
    frc::SmartDashboard::PutString ("Target Preset",       targetPresetAsString());
    frc::SmartDashboard::PutBoolean("Gamepiece at Target", isAtPreset());
}

void Gamepiece::moveToPreset(Preset preset) {
    Preset currentPreset = targetPreset; // For readability

    if (currentPreset == preset) {
        moveToTarget();
        return;
    }

    if (preset < currentPreset) {
        isMovingDown = true;
    } else {
        isMovingDown = false;
    }

    wristMoveDone = false;
    elevatorMoveDone = false;

    targetPreset = preset;
    moveToTarget();
}

void Gamepiece::moveToTarget() {
    if (calgae != nullptr) {
        // calgaeAutopilot = true;
        switch (targetPreset) {
            case Gamepiece::kCORAL_STATION:
                //// calgae->setMotorMode(Calgae::MotorModes::kCORAL_INTAKE); // Intake for the coral station
                break;
            case Gamepiece::kGROUND:
                //// calgae->setMotorMode(Calgae::MotorModes::kALGAE_INTAKE); // Intake algae if on the ground
                break;
            // kSTOP preset should be handled by controls
            default:
                // Not a preset where we do something? Don't do anything
                calgaeAutopilot = false; 
                break;
        }
    }

    bool isMovingUp = !isMovingDown; // Readability

    // If moving down, move wrist first
    // If moving up, move elevator first
    
    if (wrist != nullptr && wristDisable) {
        wrist->setSpeed(0);
    } else if (wrist != nullptr && !wristDisable &&
              (isMovingDown || (isMovingUp && elevatorMoveDone))) {
        wristAutopilot = true;
        switch (targetPreset) {
            case Gamepiece::kCORAL_STATION_LOW: wrist->toPreset(Wrist::Preset::kCORAL_STATION_LOW);  break;
            case Gamepiece::kCORAL_STATION:     wrist->toPreset(Wrist::Preset::kSTATION);            break;
            case Gamepiece::kGROUND:            wrist->toPreset(Wrist::Preset::kGROUND);             break;
            case Gamepiece::kL1:                wrist->toPreset(Wrist::Preset::kTROUGH);             break;
            case Gamepiece::kL2:                wrist->toPreset(Wrist::Preset::kBRANCH2_3);          break;
            case Gamepiece::kL3:                wrist->toPreset(Wrist::Preset::kBRANCH2_3);          break;
            case Gamepiece::kL4:                wrist->toPreset(Wrist::Preset::kBRANCH4);            break;
            case Gamepiece::kNET:               wrist->toPreset(Wrist::Preset::kTROUGH);             break;
            case Gamepiece::kPROCESSOR:         wrist->toPreset(Wrist::Preset::kPROCESSOR);          break;
            case Gamepiece::kTRANSIT:           wrist->toPreset(Wrist::Preset::kTRANSIT);            break;
            case Gamepiece::kREEF_HIGH:         wrist->toPreset(Wrist::Preset::kREEF);               break;
            case Gamepiece::kREEF_LOW:          wrist->toPreset(Wrist::Preset::kREEF);               break;
            case Gamepiece::kSTOP:              wrist->setSpeed(0);                                  break;
            default:
                // Not a preset where we do something? Don't do anything
                wristAutopilot = false; 
                break;
        }

        wristMoveDone = wrist->atPreset();
    } else {
        wristMoveDone = true;
    }

    elevator->isDisabled = elevatorDisable;
    if (elevator != nullptr && elevatorDisable) {
        elevator->goToPreset(Elevator::Preset::kSTOP);

    } else if (elevator != nullptr && !elevatorDisable &&
              (isMovingUp || (isMovingDown && wristMoveDone))) {
        elevatorAutopilot = true;
        switch (targetPreset) {
            case Gamepiece::kCORAL_STATION_LOW: elevator->goToPreset(Elevator::Preset::kCORAL_STATION_LOW); break;
            case Gamepiece::kCORAL_STATION:     elevator->goToPreset(Elevator::Preset::kCORAL_STATION);     break;
            case Gamepiece::kGROUND:            elevator->goToPreset(Elevator::Preset::kGROUND);            break;
            case Gamepiece::kL1:                elevator->goToPreset(Elevator::Preset::kL1);                break;
            case Gamepiece::kL2:                elevator->goToPreset(Elevator::Preset::kL2);                break;
            case Gamepiece::kL3:                elevator->goToPreset(Elevator::Preset::kL3);                break;
            case Gamepiece::kL4:                elevator->goToPreset(Elevator::Preset::kL4);                break;
            case Gamepiece::kNET:               elevator->goToPreset(Elevator::Preset::kNET);               break;
            case Gamepiece::kPROCESSOR:         elevator->goToPreset(Elevator::Preset::kPROCESSOR);         break;
            case Gamepiece::kTRANSIT:           elevator->goToPreset(Elevator::Preset::kTRANSIT);           break;
            case Gamepiece::kREEF_HIGH:         elevator->goToPreset(Elevator::Preset::kREEF_HIGH);         break;
            case Gamepiece::kREEF_LOW:          elevator->goToPreset(Elevator::Preset::kREEF_LOW);          break;
            case Gamepiece::kSTOP:              elevator->goToPreset(Elevator::Preset::kSTOP);              break;
            default:
                // Not a preset where we do something? Don't do anything
                elevatorAutopilot = false; 
                break;
        }
        elevatorMoveDone = elevator->atPreset();
    } else {
        elevatorMoveDone = true;
    }
}

bool Gamepiece::isAtPreset() {
    if (wrist != nullptr)
        if (!wrist->atPreset())
            return false;

    if (elevator != nullptr)
        if (!elevator->atPreset())
            return false;

    return true;
}

bool Gamepiece::hasGamepiece() {
    if (calgae != nullptr) {
        return calgae->hasGamepiece();
    }

    return true;
}

std::string Gamepiece::targetPresetAsString() {
    switch (targetPreset) {
        case Preset::kSTOP: return "Stop";
        case Preset::kGROUND: return "Ground";
        case Preset::kPROCESSOR: return "Processor";
        case Preset::kCORAL_STATION: return "Coral Station";
        case Preset::kCORAL_STATION_LOW: return "Coral Station Low";
        case Preset::kL1: return "L1 (Trough)";
        case Preset::kL2: return "L2";
        case Preset::kL3: return "L3";
        case Preset::kL4: return "L4";
        case Preset::kNET: return "Net";
        case Preset::kTRANSIT: return "Transit";
        case Preset::kREEF_LOW: return "Reef Low";
        case Preset::kREEF_HIGH: return "Reef High";
        default: return "Error reading";
    }
}