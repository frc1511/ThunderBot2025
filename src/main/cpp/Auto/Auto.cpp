#include "Auto/Auto.h"

Auto::Auto(Drive *drive_, Limelight *limelight_, Gamepiece *gamepiece_)
: drive(drive_),
  limelight(limelight_),
  gamepiece(gamepiece_),
  toTransit(gamepiece_, Gamepiece::Preset::kTRANSIT),
  toL1(gamepiece_, Gamepiece::Preset::kL1),
  toL2(gamepiece_, Gamepiece::Preset::kL2),
  toL3(gamepiece_, Gamepiece::Preset::kL3),
  toL4(gamepiece_, Gamepiece::Preset::kL4),
  toBarge(gamepiece_, Gamepiece::Preset::kNET),
  toCoralStation(gamepiece_, Gamepiece::Preset::kCORAL_STATION),
  toReefLow(gamepiece_, Gamepiece::Preset::kREEF_LOW),
  toReefHigh(gamepiece_, Gamepiece::Preset::kREEF_HIGH),
  toProcessor(gamepiece_, Gamepiece::Preset::kPROCESSOR),
  startToCoralStation(gamepiece_, Gamepiece::Preset::kCORAL_STATION),
  shootCoral(gamepiece_),
  intakeCoral(gamepiece_, Calgae::GamepieceState::kCORAL),
  intakeAlgae(gamepiece_, Calgae::GamepieceState::kALGAE),
  autoAlignLeftNormal(drive_, true, false),
  autoAlignRightNormal(drive_, false, false),
  autoAlignLeftL4(drive_, true, true),
  autoAlignRightL4(drive_, false, true),
  startAlgaeIntake(gamepiece_, Calgae::GamepieceState::kALGAE),
  stopAlgaeIntake(gamepiece_)
{}

void Auto::autoImportAutoAutos() {
    std::vector<std::filesystem::path> pathFileLocations {};
    for (const auto & entry : std::filesystem::directory_iterator(DEPLOY_DIR)) {
        if (!entry.exists()) continue;
        if (entry.is_directory()) continue;
        if (entry.path().extension() != ".csv") continue;
        pathFileLocations.push_back(entry.path());
    }

    int currentId = 2;

    int addedAutos = 0;
    int totalAutos = pathFileLocations.size();

    for (auto pathLocation : pathFileLocations) {
        bool isNotTrash = false;
        std::string fileName = pathLocation.filename().string().substr(0, pathLocation.filename().string().size() - 4); // Remove the .csv
        if (fileName == LEAVE_ID) {
            bluePaths.insert({LEAVE_ID, CSVTrajectory(pathLocation, false, StartPosition::kSPECIAL, ScorePosition::kSPECIAL)});
            redPaths.insert({LEAVE_ID, CSVTrajectory(pathLocation, true, StartPosition::kSPECIAL, ScorePosition::kSPECIAL)});
            continue;
        }
        
        //! First Part
        size_t matchedTextLength = 0;
        ScorePosition firstScorePosition = ScorePosition::kL1;
        for (auto entry : ScoreMap) {
            if (fileName.starts_with(entry.first)) {
                firstScorePosition = entry.second;
                matchedTextLength = entry.first.size();
                isNotTrash = true;
                break;
            }
        }
        if (!isNotTrash) { // If it's trash
            printf("Auto path `%s` was rejected for not meeting inital score location criteria\n", pathLocation.c_str());
            continue; 
        }
        if (fileName.size() - matchedTextLength <= 0) {
            printf("Auto path `%s` was rejected because it would end the string at score location criteria!\n", pathLocation.c_str());
            continue;
        }
        fileName = fileName.substr(matchedTextLength, fileName.size() - matchedTextLength);

        //! Second Part
        matchedTextLength = 0;
        StartPosition startPosition = StartPosition::kCENTER;
        for (auto entry : StartMap) {
            if (fileName.starts_with(entry.first)) {
                startPosition = entry.second;
                matchedTextLength = entry.first.size();
                isNotTrash = true;
                break;
            }
        }
        if (!isNotTrash) { // If it's trash
            printf("Auto path `%s` was rejected for not meeting start position criteria\n", pathLocation.c_str());
            continue; 
        }
        if (fileName.size() - matchedTextLength <= 0) {
            AutoID id = pathLocation.filename().string();
            bluePaths.insert({id, CSVTrajectory(pathLocation, false, startPosition, firstScorePosition)});
            redPaths.insert({id, CSVTrajectory(pathLocation, true, startPosition, firstScorePosition)});
            convertTable.insert({id, currentId++});
            addedAutos++;
            continue;
        }
        fileName = fileName.substr(matchedTextLength, fileName.size() - matchedTextLength);

        //! ThenStation / Algae
        matchedTextLength = 0;
        bool isAlgae = false;
        if (fileName.starts_with("Algae")) {
            isAlgae = true;
            matchedTextLength = std::string("Algae").size();
            isNotTrash = true;
        } else if (fileName.starts_with("ThenStation")) {
            isAlgae = false;
            matchedTextLength = std::string("ThenStation").size();
            isNotTrash = true;
        }
        if (!isNotTrash) { // If it's trash
            printf("Auto path `%s` was rejected for not being ThenStation or Algae after start position\n", pathLocation.c_str());
            continue; 
        }
        if (fileName.size() - matchedTextLength - 1 <= 0) {
            printf("Auto path `%s` was rejected because it would end the string at ThenStation or Algae without a side or height!\n", pathLocation.c_str());
            continue;
        }
        fileName = fileName.substr(matchedTextLength, fileName.size() - matchedTextLength);

        //! Algae
        if (isAlgae) {
            matchedTextLength = 0;
            AlgaePosition algaePosition = AlgaePosition::kLOW;
            for (auto entry : AlgaeMap) {
                if (fileName.starts_with(entry.first)) {
                    algaePosition = entry.second;
                    matchedTextLength = entry.first.size();
                    isNotTrash = true;
                    break;
                }
            }
            if (!isNotTrash) { // If it's trash
                printf("Auto path `%s` was rejected for not meeting algae score location criteria\n", pathLocation.c_str());
                continue; 
            }
            if (fileName.size() - matchedTextLength <= 0) {
                AutoID id = pathLocation.filename().string();
                bluePaths.insert({id, CSVTrajectory(pathLocation, false, startPosition, firstScorePosition, algaePosition)});
                redPaths.insert({id, CSVTrajectory(pathLocation, true, startPosition, firstScorePosition, algaePosition)});
                convertTable.insert({id, currentId++});
                addedAutos++;
                continue;    
            }
            fileName = fileName.substr(matchedTextLength, fileName.size() - matchedTextLength);
        }
        //! Coral Station
        if (!isAlgae) {
            matchedTextLength = 0;
            CoralStationPosition coralStationPosition = CoralStationPosition::kNONE;
            for (auto entry : CoralStationMap) {
                if (fileName.starts_with(entry.first)) {
                    coralStationPosition = entry.second;
                    matchedTextLength = entry.first.size();
                    isNotTrash = true;
                    break;
                }
            }
            if (!isNotTrash) { // If it's trash
                printf("Auto path `%s` was rejected for not meeting coral station location criteria\n", pathLocation.c_str());
                continue; 
            }
            if (fileName.size() - matchedTextLength <= 0) {
                printf("Auto path `%s` was rejected because it would end the string at ThenStation without where to go!\n", pathLocation.c_str());
                continue;
            }
            fileName = fileName.substr(matchedTextLength, fileName.size() - matchedTextLength);

            //! Remove the ThenScore component
            matchedTextLength = std::string("ThenScore").size();
            if (fileName.size() - matchedTextLength <= 0) {
                printf("Auto path `%s` was rejected because it does not contain the `ThenScore` after coral station\n", pathLocation.c_str());
                continue;
            }
            fileName = fileName.substr(matchedTextLength, fileName.size() - matchedTextLength);

            //! Get the next scoring location
            size_t matchedTextLength = 0;
            ScorePosition secondScorePosition = ScorePosition::kL1;
            for (auto entry : ScoreMap) {
                if (fileName.starts_with(entry.first)) {
                    secondScorePosition = entry.second;
                    matchedTextLength = entry.first.size();
                    isNotTrash = true;
                    break;
                }
            }
            if (!isNotTrash) { // If it's trash
                printf("Auto path `%s` was rejected for not having a score location after coral station position criteria\n", pathLocation.c_str());
                continue; 
            }
            if (fileName.size() - matchedTextLength <= 0) {
                AutoID id = pathLocation.filename().string();
                bluePaths.insert({id, CSVTrajectory(pathLocation, false, startPosition, firstScorePosition, AlgaePosition::kNONE, coralStationPosition, secondScorePosition)});
                redPaths.insert({id, CSVTrajectory(pathLocation, true, startPosition, firstScorePosition, AlgaePosition::kNONE, coralStationPosition, secondScorePosition)});
                convertTable.insert({id, currentId++});
                addedAutos++;
                continue;
            }
            fileName = fileName.substr(matchedTextLength, fileName.size() - matchedTextLength);
        }

        printf("Auto path `%s` was rejected for being to complex and not being able to be parsed at this time. :)\n", pathLocation.c_str());
    }
    printf("Autos Parsing Info: %d/%d were accepted\n", addedAutos, totalAutos);
}

void Auto::resetToMatchMode(MatchMode priorMode, MatchMode mode) {
    if (mode == MatchMode::AUTO) {
        isAuto = true;
        step = 0;
        drive->calibrateIMU();
            /// Separate Red and Blue paths are not required
        if (auto ally = frc::DriverStation::GetAlliance())
        {
            if (ally.value() == frc::DriverStation::Alliance::kRed)
            {
                printf("Red Alliance\n");
                paths = &redPaths;    
            }
            if (ally.value() == frc::DriverStation::Alliance::kBlue)
            {
                printf("Blue Alliance\n");
                paths = &bluePaths;    
            }
        }
        else
        {
            printf("No alliance selected\n");
        }
    } else {
        isAuto = false;
    }
}
void Auto::process() { //called during auto
    if (!isAuto)
        return;
    if (autoId == DO_NOTHING_ID) {
        doNothing();
    } else if (autoId == LEAVE_ID) {
        leave();
    } else {
        runPath();
    }
}

void Auto::leave() {
    limelight->setFunctioningState(false);
    if (step == 0) {
        drive->setupInitialTrajectoryPosition(&paths->at(LEAVE_ID));
        drive->runTrajectory(&paths->at(LEAVE_ID), actions);
        step++;
    } else if (step == 1 && drive->isFinished()) {
        printf("Finished Drive!\n");
        step++;
    }
}

void Auto::runPath() {
    if (step == 0) {
        auto pathPtr = &paths->at(autoId);
        drive->setupInitialTrajectoryPosition(pathPtr);
        drive->runTrajectory(pathPtr, actions);
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
        //Very necessary function! ~Gracie

    // Good function.
    // Very good function. - jeff downs
    // Very bad function. - jeff ups
    // I agree with jeff downs since he likes java - charlie(2024)

    // Well technically it's doing something - chris(2023)
    // It's a paradox! I learned this in english class! - mysteriousPerson(2025)
    // If nothing is doing something, then there is no nothing because its always something (its a paradox) - eleanor(2025)
}

void Auto::autoSelectorInit() {
    for (auto i : convertTable) {
        if (redPaths.find(i.first) == redPaths.end()) {
            autoSelector.AddOption(i.first, (int)i.second); // Other Special Things
            continue;
        }
        if (i.first == "Leave") {
            autoSelector.AddOption("LEAVE | Moving the waste of space slightly forward to get an RP", (int)i.second); // Other Special Things
            continue;
        }
        CSVTrajectory trajectory = redPaths.at(i.first);

        std::string startLocation = StartMapBack[trajectory.startPosition];
        std::string firstScore = ScoreMapBack[trajectory.firstScorePosition];
        std::string optionName = "Start at " + startLocation + ", score on " + firstScore;
        
        if (trajectory.algaePosition != AlgaePosition::kNONE) {
            optionName += ", grab algae from " + AlgaeMapBack[trajectory.algaePosition] + "er holder";
        }
        
        if (trajectory.secondScorePosition != ScorePosition::kSPECIAL) {
            optionName += ", grab from coral station " + CoralStationMapBack[trajectory.coralStationPosition] + ", score on " + ScoreMapBack[trajectory.secondScorePosition];
        }

        autoSelector.AddOption(optionName, (int)i.second);
    }

    autoSelector.SetDefaultOption("Do Nothing", convertTable.at(DO_NOTHING_ID));
}

void Auto::sendFeedback() {
    int desiredAutoMode = autoSelector.GetSelected();
    autoId = DO_NOTHING_ID;
    for (const auto & [key, value] : convertTable) {
        if (value == desiredAutoMode) {
            autoId = static_cast<AutoID>(key);
        }
    }

    frc::SmartDashboard::PutData("Auto Modes", &autoSelector);

    frc::SmartDashboard::PutNumber("Autonomous_Step", step);
    frc::SmartDashboard::PutBoolean("Autonomous_DriveFinished", drive->isFinished());
    frc::SmartDashboard::PutString("Autonomous_ModeName", autoId);
}