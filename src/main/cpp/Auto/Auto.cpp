#include "Auto/Auto.h"

Auto::Auto(Drive *drive_, Limelight *limelight_)
    : drive(drive_),
      limelight(limelight_) {
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
        case _TEST:
            test();
            break;
        case _SQUARE:
            square();
            break;
        case LEAVE:
            leave();
            break;
        case LEAVE_GO_TO_UPPER_CORAL:
            leaveUpperCoral();
            break;
        case LEAVE_GO_TO_LOWER_CORAL:
            leaveLowerCoral();
            break;
        case L1:
            L1Coral1();
            break;
    }
}

void Auto::test() {
    if (step == 0) {
        drive->setupInitialTrajectoryPosition(&paths->at(Path::_TEST));
        drive->runTrajectory(&paths->at(Path::_TEST), actions);
        step++;
    } else if (step == 1 && drive->isFinished()) {
        printf("Finished Drive!\n");
        step++;
    }
}

void Auto::leave() {
    limelight->setFunctioningState(false);
    if (step == 0) {
        drive->setupInitialTrajectoryPosition(&paths->at(Path::LEAVE));
        drive->runTrajectory(&paths->at(Path::LEAVE), actions);
        step++;
    } else if (step == 1 && drive->isFinished()) {
        printf("Finished Drive!\n");
        step++;
    }
}

void Auto::leaveUpperCoral() {
    if (step == 0) {
        drive->setupInitialTrajectoryPosition(&paths->at(Path::LEAVE_GO_TO_UPPER_CORAL));
        drive->runTrajectory(&paths->at(Path::LEAVE_GO_TO_UPPER_CORAL), actions);
        step++;
    } else if (step == 1 && drive->isFinished()) {
        printf("Finished Drive!\n");
        step++;
    }
}

void Auto::leaveLowerCoral() {
    if (step == 0) {
        drive->setupInitialTrajectoryPosition(&paths->at(Path::LEAVE_GO_TO_LOWER_CORAL));
        drive->runTrajectory(&paths->at(Path::LEAVE_GO_TO_LOWER_CORAL), actions);
        step++;
    } else if (step == 1 && drive->isFinished()) {
        printf("Finished Drive!\n");
        step++;
    }
}

void Auto::L1Coral1() {
    if (step == 0) {
        drive->setupInitialTrajectoryPosition(&paths->at(Path::L1));
        drive->runTrajectory(&paths->at(Path::_SQUARE), actions);
        step++;
    } else if (step == 1 && drive->isFinished()) {
        step++;
    }
}

void Auto::square() {
    if (step == 0) {
        drive->setupInitialTrajectoryPosition(&paths->at(Path::_SQUARE));
        drive->runTrajectory(&paths->at(Path::_SQUARE), actions);
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
    autoSelector.AddOption("Do Nothing",                       (int)AutoMode::DO_NOTHING);
    autoSelector.AddOption("zzz_TEST",                         (int)AutoMode::_TEST);
    autoSelector.AddOption("zzz_SQUARE",                       (int)AutoMode::_SQUARE);
    autoSelector.AddOption("leave",                            (int)AutoMode::LEAVE);
    autoSelector.AddOption("leave to far coral station",       (int)AutoMode::LEAVE_GO_TO_UPPER_CORAL);
    autoSelector.AddOption("leave to processor coral station", (int)AutoMode::LEAVE_GO_TO_LOWER_CORAL);
    autoSelector.AddOption("1 Coral to L1",                    (int)AutoMode::L1);

    autoSelector.SetDefaultOption("Do Nothing",                (int)AutoMode::DO_NOTHING);
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