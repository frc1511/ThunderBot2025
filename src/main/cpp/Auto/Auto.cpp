#include "Auto/Auto.h"

Auto::Auto(Drive* drive)
    : drive(drive) {

}
void Auto::resetToMatchMode(MatchMode priorMode, MatchMode mode) {
    if (mode == MatchMode::AUTO) {
        step = 0;
        drive->calibrateIMU();
    }
}
void Auto::process() { //called during auto
    /// Seperate Red and Blue paths are not required
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        paths = &redPaths;
    }
    else {
        paths = &bluePaths;
    }

    switch (mode) { //find what auto mode you are using and do it
        using enum AutoMode;
        case DO_NOTHING:
            doNothing();
            break;
        case TEST:
            test();
            break;
    }
}

void Auto::test() { //test auto, leave, grab a note, and shoot
    if (step == 0) {
        drive->setupInitialTrajectoryPosition(&paths->at(Path::TEST));
        drive->runTrajectory(&paths->at(Path::TEST), actions);
        step++;
    } else if (step == 1 && drive->isFinished()) {
        printf("Finished Drive!\n");
        step++;
    }
}

void Auto::doNothing() {
    // If it does nothing is it doing something or nothing? - trevor(2020)
        //it does something because it is doing nothing - ishan(2022)
        //I disagree - peter(2022)
        //I agree with peter -L Wrench
        //I still disagree with ishan - peter(2023)
        //it does something because this function exists and can be called as an action for the robot - ben d(2024)
        //also for just this year we made it do stuff - also ben d(2024)

    // Good function.
    // Very good function. - jeff downs
    // Very bad function. - jeff ups
    // I agree with jeff downs since he likes java - charlie(2024)

    // Well technically it's doing something - chris(2023)
}

void Auto::autoSelectorInit() {
    autoSelector.SetDefaultOption("Do Nothing", (int)AutoMode::DO_NOTHING);
    autoSelector.SetDefaultOption("TEST",       (int)AutoMode::TEST);
}

void Auto::sendFeedback() {
    int desiredAutoMode = autoSelector.GetSelected();
    if (desiredAutoMode < (int)autoModeNames.size() && desiredAutoMode >= 0) {
        mode = static_cast<AutoMode>(desiredAutoMode);
    }
    else {
        mode = AutoMode::DO_NOTHING;
    }

    frc::SmartDashboard::PutData("Auto Modes", &autoSelector);
    

    frc::SmartDashboard::PutNumber("Autonomous_Step", step);
    frc::SmartDashboard::PutBoolean("Autonomous_DriveFinished", drive->isFinished());
    frc::SmartDashboard::PutString("Autonomous_ModeName", autoModeNames.at(mode));

    std::string buffer = "";

    auto handleDashboardString = [&](AutoMode mode, const char* description) {
        int mode_index = static_cast<int>(mode);
        // Put mode index in buffer.
        buffer += fmt::format(",{}", mode_index);
        // Send description.
        frc::SmartDashboard::PutString(fmt::format("thunderdashboard_auto_{}", mode_index), description);
    };

    for (auto [mode, name] : autoModeNames) {
        handleDashboardString(mode, name);
    }

    frc::SmartDashboard::PutString("thunderdashboard_auto_list", buffer);
}
