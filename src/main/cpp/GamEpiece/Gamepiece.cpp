#include "GamEpiece/Gamepiece.h"

Gamepiece::Gamepiece(Calgae *calgae_, Wrist *wrist_, Elevator *elevator_)
: calgae(calgae_),
  wrist(wrist_),
  elevator(elevator_)
{}

void Gamepiece::process() {

}

void Gamepiece::doPersistentConfiguration() {}

void Gamepiece::sendFeedback() {
    frc::SmartDashboard::PutBoolean("Calgae Autopilot",   calgaeAutopilot);
    frc::SmartDashboard::PutBoolean("Wrist Autopilot",    wristAutopilot);
    frc::SmartDashboard::PutBoolean("Elevator Autopilot", elevatorAutopilot);
    frc::SmartDashboard::PutString ("Target Preset",      targetPresetAsString());
}

void Gamepiece::moveToPreset(Preset preset) {
    targetPreset = preset;

    if (calgae != nullptr) {
        calgaeAutopilot = true;
        switch (targetPreset) {
        case Gamepiece::kCORAL_STATION:
            calgae->setMotorMode(Calgae::MotorModes::kCORAL_INTAKE); // Intake for the coral station
            break;
        case Gamepiece::kGROUND:
            calgae->setMotorMode(Calgae::MotorModes::kALGAE_INTAKE); // Intake algae if on the ground
            break;
        // kSTOP preset should be handled by controls
        default:
            // Not a preset where we do something? Don't do anything
            calgaeAutopilot = false; 
            break;
        }
    }

    if (wrist != nullptr) {
        wristAutopilot = true;
        switch (targetPreset) {
        case Gamepiece::kCORAL_STATION: wrist->toPreset(Wrist::Preset::kSTATION);   break;
        case Gamepiece::kGROUND:        wrist->toPreset(Wrist::Preset::kGROUND);    break;
        case Gamepiece::kL1:            wrist->toPreset(Wrist::Preset::kTROUGH);    break;
        case Gamepiece::kL2:            wrist->toPreset(Wrist::Preset::kBRANCH2_3); break;
        case Gamepiece::kL3:            wrist->toPreset(Wrist::Preset::kBRANCH2_3); break;
        case Gamepiece::kL4:            wrist->toPreset(Wrist::Preset::kBRANCH4);   break;
        case Gamepiece::kPROCESSOR:     wrist->toPreset(Wrist::Preset::kPROCESSOR); break;
        // kSTOP preset should be handled by controls
        default:
            // Not a preset where we do something? Don't do anything
            wristAutopilot = false; 
            break;
        }
    }

    if (elevator != nullptr) {
        elevatorAutopilot = true;
        switch (targetPreset) {
        case Gamepiece::kCORAL_STATION: elevator->goToPreset(Elevator::Preset::kCORAL_STATION); break;
        case Gamepiece::kGROUND:        elevator->goToPreset(Elevator::Preset::kGROUND);        break;
        case Gamepiece::kL1:            elevator->goToPreset(Elevator::Preset::kL1);            break;
        case Gamepiece::kL2:            elevator->goToPreset(Elevator::Preset::kL2);            break;
        case Gamepiece::kL3:            elevator->goToPreset(Elevator::Preset::kL3);            break;
        case Gamepiece::kL4:            elevator->goToPreset(Elevator::Preset::kL4);            break;
        case Gamepiece::kNET:           elevator->goToPreset(Elevator::Preset::kNET);           break;
        case Gamepiece::kPROCESSOR:     elevator->goToPreset(Elevator::Preset::kPROCESSOR);     break;
        // kSTOP preset should be handled by controls
        default:
            // Not a preset where we do something? Don't do anything
            elevatorAutopilot = false; 
            break;
        }
    }
}

bool Gamepiece::isAtPreset() {
    if (calgae != nullptr) 
        if (!calgae->atSpeed())
            return false;

    if (wrist != nullptr)
        if (!wrist->atPreset())
            return false;

    if (elevator != nullptr)
        if (!elevator->atPreset())
            return false;
    return true;
}

bool Gamepiece::hasGamepiece() {
    return calgae->hasGamepiece();
}

std::string Gamepiece::targetPresetAsString() {
    switch (targetPreset) {
    case Preset::kSTOP: return "Stop";
    case Preset::kGROUND: return "Ground";
    case Preset::kPROCESSOR: return "Processor";
    case Preset::kCORAL_STATION: return "Coral Station";
    case Preset::kL1: return "L1 (Trough)";
    case Preset::kL2: return "L2";
    case Preset::kL3: return "L3";
    case Preset::kL4: return "L4";
    case Preset::kNET: return "Net";
    default: return "Error reading motorSpeed";
    }
}