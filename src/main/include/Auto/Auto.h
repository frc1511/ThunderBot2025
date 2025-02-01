#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Drive/CSVTrajectory.h"
#include "Drive/Drive.h"
#include "Basic/Component.h"
#include "Auto/Action.h"

#define DEPLOY_DIR "/home/lvuser/deploy/"

class Auto : public Component {

public:
    Auto(Drive *drive);

    void process() override;
    void sendFeedback() override;
    void resetToMatchMode(MatchMode priorMode, MatchMode mode) override;

    frc::SendableChooser<int> autoSelector;
    void autoSelectorInit();

private:
    enum class AutoMode
    {
        DO_NOTHING                 = 0,
        _TEST                      = 1,
        _SQUARE                    = 2,
        LEAVE                      = 3,
        LEAVE_GO_TO_UPPER_CORAL    = 4,
        LEAVE_GO_TO_LOWER_CORAL    = 5,
    };
    Drive *drive;

    Auto::AutoMode mode = AutoMode::DO_NOTHING;

    void doNothing();
    void test();
    void square();
    void leave_upper_coral();
    void leave_lower_coral();
    void leave();

    const std::map<AutoMode, const char*> autoModeNames {
        { AutoMode::DO_NOTHING,                "Do Nothing"},
        { AutoMode::_TEST,                     "zzz_Test"},
        { AutoMode::_SQUARE,                   "zzz_Square"},
        { AutoMode::LEAVE_GO_TO_UPPER_CORAL, "Leave go to UPPER coral station"},
        { AutoMode::LEAVE_GO_TO_LOWER_CORAL, "Leave go to LOWER coral station"},
        { AutoMode::LEAVE,      "Leave"},
    };
    int step = 0;

    enum class Path {
        _TEST,
        _SQUARE,
        LEAVE_GO_TO_UPPER_CORAL,
        LEAVE_GO_TO_LOWER_CORAL,
        LEAVE,
    };
    const std::map<Path, CSVTrajectory> bluePaths {
        { Path::_TEST, CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv",                   false } },
        { Path::_SQUARE, CSVTrajectory{ DEPLOY_DIR "square.csv",                             false } },
        { Path::LEAVE_GO_TO_UPPER_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation1.csv",  false } },
        { Path::LEAVE_GO_TO_LOWER_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation2.csv",  false } },
        { Path:: LEAVE, CSVTrajectory{ DEPLOY_DIR "leave.csv",                               false } },
    };
    const std::map<Path, CSVTrajectory> redPaths {
        { Path::_TEST, CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv",                   true  } },
        { Path::_SQUARE, CSVTrajectory{ DEPLOY_DIR "square.csv",                             true  } },
        { Path::LEAVE_GO_TO_UPPER_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation1.csv",  true  } },
        { Path::LEAVE_GO_TO_LOWER_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation2.csv",  true  } },
        { Path:: LEAVE, CSVTrajectory{ DEPLOY_DIR "leave.csv",                               true  } },
    };
    const std::map<Path, CSVTrajectory>* paths = nullptr;

    std::map<u_int32_t, Action*> actions {

    };

    std::string autoSelected;
};