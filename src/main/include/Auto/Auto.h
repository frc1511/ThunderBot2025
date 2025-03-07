#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Drive/CSVTrajectory.h"
#include "Basic/Component.h"
#include "Auto/Action.h"
#include "Gamepiece/Gamepiece.h"

#include "Drive/Drive.h"

#define DEPLOY_DIR "/home/lvuser/deploy/"

class Auto : public Component {

public:
    Auto(Drive *drive_, Limelight *limelight_, Gamepiece *gamepiece_);

    void process() override;
    void sendFeedback() override;
    void resetToMatchMode(MatchMode priorMode, MatchMode mode) override;

    frc::SendableChooser<int> autoSelector;
    void autoSelectorInit();

private:
    enum class AutoMode {
        DO_NOTHING,
        _TEST,
        _SQUARE,
        LEAVE,
        LEAVE_GO_TO_LEFT_CORAL,
        LEAVE_GO_TO_RIGHT_CORAL,
        L1Center,
        L1Left,
        L1Right,
    };
    Drive *drive;
    Limelight *limelight;
    Gamepiece *gamepiece;

    Auto::AutoMode mode = AutoMode::DO_NOTHING;

    // Match Autos
    void doNothing();
    void leave();
    void runPath();

    const std::map<AutoMode, const char*> autoModeNames {
        { AutoMode::DO_NOTHING,              "Do Nothing"},
        { AutoMode::_TEST,                   "zzz_Test"},
        { AutoMode::_SQUARE,                 "zzz_Square"},
        { AutoMode::LEAVE,                   "Leave"},
        { AutoMode::LEAVE_GO_TO_LEFT_CORAL,  "Leave go to LEFT coral station"},
        { AutoMode::LEAVE_GO_TO_RIGHT_CORAL, "Leave go to RIGHT coral station"},
        { AutoMode::L1Center,                "1 Coral L1 Center"},
        { AutoMode::L1Left,                  "1 Coral L1 Left"},
        { AutoMode::L1Right,                 "1 Coral L1 Right"},
    };
    int step = 0;
    enum class Path {
        _TEST,
        _SQUARE,
        LEAVE,
        LEAVE_GO_TO_LEFT_CORAL,
        LEAVE_GO_TO_RIGHT_CORAL,
        L1Center,
        L1Left,
        L1Right,
    };
    const std::map<AutoMode, Path> autoModePaths {
        {AutoMode::_TEST, Path::_TEST},
        {AutoMode::_SQUARE, Path::_SQUARE},
        {AutoMode::LEAVE, Path::LEAVE},
        {AutoMode::LEAVE_GO_TO_LEFT_CORAL, Path::LEAVE_GO_TO_LEFT_CORAL},
        {AutoMode::LEAVE_GO_TO_RIGHT_CORAL, Path::LEAVE_GO_TO_RIGHT_CORAL},
        {AutoMode::L1Center, Path::L1Center},
        {AutoMode::L1Left, Path::L1Left},
        {AutoMode::L1Right, Path::L1Right},
    };
    const std::map<Path, CSVTrajectory> bluePaths {
        { Path::_TEST,                   CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv", false } },
        { Path::_SQUARE,                 CSVTrajectory{ DEPLOY_DIR "square.csv",             false } },
        { Path:: LEAVE,                  CSVTrajectory{ DEPLOY_DIR "Leave.csv",              false } },
        { Path::LEAVE_GO_TO_LEFT_CORAL,  CSVTrajectory{ DEPLOY_DIR "GoToCoralStationLeft.csv",  false } },
        { Path::LEAVE_GO_TO_RIGHT_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStationRight.csv",  false } },
        { Path::L1Center,                CSVTrajectory{ DEPLOY_DIR "L1Center.csv",           false } },
        { Path::L1Left,                  CSVTrajectory{ DEPLOY_DIR "L1Left.csv",             false } },
        { Path::L1Right,                 CSVTrajectory{ DEPLOY_DIR "L1Right.csv",            false } },

    };
    const std::map<Path, CSVTrajectory> redPaths {
        { Path::_TEST,                   CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv", true  } },
        { Path::_SQUARE,                 CSVTrajectory{ DEPLOY_DIR "square.csv",             true  } },
        { Path::LEAVE,                   CSVTrajectory{ DEPLOY_DIR "Leave.csv",              true  } },
        { Path::LEAVE_GO_TO_LEFT_CORAL,  CSVTrajectory{ DEPLOY_DIR "GoToCoralStationLeft.csv",  true  } },
        { Path::LEAVE_GO_TO_RIGHT_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStationRight.csv",  true  } },
        { Path::L1Center,                CSVTrajectory{ DEPLOY_DIR "L1Center.csv",           true } },
        { Path::L1Left,                  CSVTrajectory{ DEPLOY_DIR "L1Left.csv",             true } },
        { Path::L1Right,                 CSVTrajectory{ DEPLOY_DIR "L1Right.csv",            true } },
    };
    const std::map<Path, CSVTrajectory>* paths = nullptr;




    class ToGPPreset : public Action {
      public:
        ToGPPreset(Gamepiece *gamepiece_, Gamepiece::Preset preset_) : gamepiece(gamepiece_), preset(preset_) {};
        Gamepiece *gamepiece;
        Gamepiece::Preset preset;
        Action::Result process() override {
            gamepiece->moveToPreset(preset);
            return gamepiece->isAtPreset() ? Action::Result::DONE : Action::Result::WORKING;
        };
    };
    ToGPPreset toTransit;
    ToGPPreset toL1;
    ToGPPreset toL2;
    ToGPPreset toL3;
    ToGPPreset toL4;
    ToGPPreset toCoralStation;
    ToGPPreset toReefLow;
    ToGPPreset toReefHigh;

    class ShootCoral : public Action {
      public:
        ShootCoral(Gamepiece *gamepiece_) : gamepiece(gamepiece_) {};
        Gamepiece *gamepiece;
        Action::Result process() override {
            if (gamepiece->calgae == nullptr) 
                return Action::Result::DONE;
                
            if (!gamepiece->calgae->isAutoShooting)
                gamepiece->calgae->autoShoot();
            return gamepiece->calgae->isShootDone() ? Action::Result::DONE : Action::Result::WORKING;
        };
    };

    ShootCoral shootCoral;

    class Intake : public Action {
      public:
        Intake(Gamepiece *gamepiece_, Calgae::GamepieceState gp_) : gamepiece(gamepiece_), gp(gp_) {};
        Gamepiece *gamepiece;
        Calgae::GamepieceState gp;
        Action::Result process() override {
            if (gamepiece->calgae == nullptr) 
                return Action::Result::DONE;
                
            if (!gamepiece->calgae->isAutoIntaking)
                gamepiece->calgae->autoIntake(gp);
            return gamepiece->calgae->hasGamepiece() ? Action::Result::DONE : Action::Result::WORKING;
        };
    };
    Intake intakeCoral;
    Intake intakeAlgae;


    class AutoAlign : public Action {
      public:
        AutoAlign(Drive *drive_, bool isLeft_, bool isL4_) : drive(drive_), isLeft(isLeft_), isL4(isL4_) {};
        Drive *drive;
        bool isLeft;
        bool isL4;

        Action::Result process() override {
            drive->beginLineup(isLeft, isL4);
            return drive->isFinished() ? Action::Result::DONE : Action::Result::WORKING;
        };
    }; 
    AutoAlign autoAlignLeftNormal;
    AutoAlign autoAlignRightNormal;
    AutoAlign autoAlignLeftL4;
    AutoAlign autoAlignRightL4;

    std::map<u_int32_t, Action*> actions {
        {1 << 0,  &autoAlignLeftNormal},    // Normal Left Align
        {1 << 1,  &autoAlignRightNormal},   // Normal Right Align
        {1 << 2,  &autoAlignLeftL4},        // L4 Left Align
        {1 << 3,  &autoAlignRightL4},       // L4 Right Align
        {1 << 4,  &toL1},                   // L1
        {1 << 5,  &toL2},                   // L2
        {1 << 6,  &toL3},                   // L3
        {1 << 7,  &toL4},                   // L4
        {1 << 8,  &toCoralStation},         // Coral Station
        {1 << 9, &toReefLow},               // Reef Low
        {1 << 10, &toReefHigh},             // Reef High
        {1 << 11, &shootCoral},             // Shoot Coral
        {1 << 12, &intakeCoral},            // Intake Coral
        {1 << 13, &intakeAlgae}             // Intake Algae
        {1 << 14,  &toTransit},             // Transit
    };

    std::string autoSelected;

    bool isAuto = false;
};