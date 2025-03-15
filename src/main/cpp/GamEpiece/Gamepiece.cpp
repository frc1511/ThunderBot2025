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
            case Gamepiece::kNET:               wrist->toPreset(Wrist::Preset::kNET);                break;
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
    if (wrist != nullptr) {
        // Silly Stuff for auto (b/c wrist target preset isnt set until elev @ target)
        //! Mason, if you see this before I come back to it, I think the problem w/ L4 auto is that wrist reads atPreset==true until we give it a new one, and it returns true and gamepiece thinks it's at the preset before the wrist is told it's new position. Just a hypothesis. Didn't have time to debug and couldn't tell if I even did this right, but I didn't see results from what is right here. 
        switch (targetPreset) {
            case Preset::kGROUND:            if (!(wrist->currentPreset == Wrist::Preset::kGROUND))            return false; else break;
            case Preset::kPROCESSOR:         if (!(wrist->currentPreset == Wrist::Preset::kPROCESSOR))         return false; else break;
            case Preset::kTRANSIT:           if (!(wrist->currentPreset == Wrist::Preset::kTRANSIT))           return false; else break;
            case Preset::kL1:                if (!(wrist->currentPreset == Wrist::Preset::kTROUGH))            return false; else break;
            case Preset::kL2:                if (!(wrist->currentPreset == Wrist::Preset::kBRANCH2_3))         return false; else break;
            case Preset::kREEF_LOW:          if (!(wrist->currentPreset == Wrist::Preset::kREEF))              return false; else break;
            case Preset::kCORAL_STATION_LOW: if (!(wrist->currentPreset == Wrist::Preset::kCORAL_STATION_LOW)) return false; else break;
            case Preset::kCORAL_STATION:     if (!(wrist->currentPreset == Wrist::Preset::kSTATION))           return false; else break;
            case Preset::kL3:                if (!(wrist->currentPreset == Wrist::Preset::kBRANCH2_3))         return false; else break;
            case Preset::kREEF_HIGH:         if (!(wrist->currentPreset == Wrist::Preset::kREEF))              return false; else break;
            case Preset::kNET:               if (!(wrist->currentPreset == Wrist::Preset::kNET))               return false; else break;
            default:
                // Don't return true
                break;
        }

        if (!wrist->atPreset()) {
            return false;
        }
    }

    if (elevator != nullptr) {
        if (!elevator->atPreset()) {
            return false;
        }
    }

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