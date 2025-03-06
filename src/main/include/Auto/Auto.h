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
        DO_NOTHING                 = 0,
        _TEST                      = 1,
        _SQUARE                    = 2,
        LEAVE                      = 3,
        LEAVE_GO_TO_LEFT_CORAL     = 4,
        LEAVE_GO_TO_RIGHT_CORAL    = 5,
        L1Center                   = 6,
        L1Left                     = 7,
        L1Right                    = 8,
    };
    Drive *drive;
    Limelight *limelight;
    Gamepiece *gamepiece;

    Auto::AutoMode mode = AutoMode::DO_NOTHING;

    // Match Autos
    void doNothing();
    void leave();
    void leaveLeftCoral();
    void leaveRightCoral();
    void L1CoralCenter();
    void L1CoralLeft();
    void L1CoralRight();

    // Debug Autos
    void test();
    void square();

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
    const std::map<Path, CSVTrajectory> bluePaths {
        // { Path::_TEST,                   CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv", false } },
        // { Path::_SQUARE,                 CSVTrajectory{ DEPLOY_DIR "square.csv",             false } },
        // { Path:: LEAVE,                  CSVTrajectory{ DEPLOY_DIR "Leave.csv",              false } },
        // { Path::LEAVE_GO_TO_LEFT_CORAL,  CSVTrajectory{ DEPLOY_DIR "GoToCoralStation1.csv",  false } },
        // { Path::LEAVE_GO_TO_RIGHT_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation2.csv",  false } },
        // { Path::L1Center,                CSVTrajectory{ DEPLOY_DIR "L1Center.csv",           false } },
        // { Path::L1Left,                  CSVTrajectory{ DEPLOY_DIR "L1Left.csv",             false } },
        // { Path::L1Right,                 CSVTrajectory{ DEPLOY_DIR "L1Right.csv",            false } },

    };
    const std::map<Path, CSVTrajectory> redPaths {
        // { Path::_TEST,                   CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv", true  } },
        // { Path::_SQUARE,                 CSVTrajectory{ DEPLOY_DIR "square.csv",             true  } },
        // { Path::LEAVE,                   CSVTrajectory{ DEPLOY_DIR "Leave.csv",              true  } },
        // { Path::LEAVE_GO_TO_LEFT_CORAL,  CSVTrajectory{ DEPLOY_DIR "GoToCoralStation1.csv",  true  } },
        // { Path::LEAVE_GO_TO_RIGHT_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation2.csv",  true  } },
        // { Path::L1Center,                CSVTrajectory{ DEPLOY_DIR "L1Center.csv",           true } },
        // { Path::L1Left,                  CSVTrajectory{ DEPLOY_DIR "L1Left.csv",             true } },
        // { Path::L1Right,                 CSVTrajectory{ DEPLOY_DIR "L1Right.csv",            true } },
    };
    const std::map<Path, CSVTrajectory>* paths = nullptr;




    class ToL1 : public Action {
      public:
        ToL1(Gamepiece *gamepiece_) : gamepiece(gamepiece_) {};
        Gamepiece *gamepiece;
        Action::Result process() override {
            gamepiece->moveToPreset(Gamepiece::Preset::kL1);
            return gamepiece->isAtPreset() ? Action::Result::DONE : Action::Result::WORKING;
        };
    };
    ToL1 toL1;

    class ShootCoral : public Action {
      public:
        ShootCoral(Gamepiece *gamepiece_) : gamepiece(gamepiece_) {};
        Gamepiece *gamepiece;
        Action::Result process() override {
            gamepiece->calgae->autoShoot();
            return gamepiece->calgae->isShootDone() ? Action::Result::DONE : Action::Result::WORKING;
        };
    };
    ShootCoral shootCoral;

    std::map<unsigned int, Action*> actions {
        {1 << 0, &toL1}, // Ground
        {1 << 1, &toL1}, // L1
        {1 << 2, &toL1}, // L2
        {1 << 3, &toL1}, // L3
        {1 << 4, &toL1}, // L4
        {1 << 5, &shootCoral}
    };

    std::string autoSelected;

    bool isAuto = false;
};