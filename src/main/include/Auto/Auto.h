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
    void resetToMatchMode(MatchMode mode) override;

    frc::SendableChooser<int> autoSelector;
    void autoSelectorInit();

private:
    enum class AutoMode
    {
        DO_NOTHING   = 0,
        TEST         = 1,
    };
    Drive *drive;

    Auto::AutoMode mode = AutoMode::DO_NOTHING;

    void doNothing();
    void test();

    const std::map<AutoMode, const char*> autoModeNames {
        { AutoMode::DO_NOTHING, "Do Nothing"},
        { AutoMode::TEST,       "Test"},
    };
    int step = 0;

    enum class Path {
        TEST,
    };
    const std::map<Path, CSVTrajectory> bluePaths {
        { Path::TEST, CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv",          false } },

    };
    const std::map<Path, CSVTrajectory> redPaths {
        { Path::TEST, CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv",          true  } },
    };
    const std::map<Path, CSVTrajectory>* paths = nullptr;

    std::map<u_int32_t, Action*> actions {

    };

    std::string autoSelected;
};