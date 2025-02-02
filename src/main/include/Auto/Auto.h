#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Drive/CSVTrajectory.h"
#include "Basic/Component.h"
#include "Auto/Action.h"

#include "Drive/Drive.h"

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
    enum class AutoMode {
        DO_NOTHING                 = 0,
        _TEST                      = 1,
        _SQUARE                    = 2,
        LEAVE                      = 3,
        LEAVE_GO_TO_UPPER_CORAL    = 4,
        LEAVE_GO_TO_LOWER_CORAL    = 5,
        L1           = 6,
    };
    Drive *drive;

    Auto::AutoMode mode = AutoMode::DO_NOTHING;

    // Match Autos
    void doNothing();
    void leave();
    void leaveUpperCoral();
    void leaveLowerCoral();
    void L1Coral1();

    // Debug Autos
    void test();
    void square();

    const std::map<AutoMode, const char*> autoModeNames {
        { AutoMode::DO_NOTHING,                "Do Nothing"},
        { AutoMode::_TEST,                     "zzz_Test"},
        { AutoMode::_SQUARE,                   "zzz_Square"},
        { AutoMode::LEAVE,      "Leave"},
        { AutoMode::LEAVE_GO_TO_UPPER_CORAL, "Leave go to UPPER coral station"},
        { AutoMode::LEAVE_GO_TO_LOWER_CORAL, "Leave go to LOWER coral station"},
        { AutoMode::L1,         "1 Coral L1"},
    };
    int step = 0;

    enum class Path {
        _TEST,
        _SQUARE,
        LEAVE,
        LEAVE_GO_TO_UPPER_CORAL,
        LEAVE_GO_TO_LOWER_CORAL,
        L1,
    };
    const std::map<Path, CSVTrajectory> bluePaths {
        { Path::_TEST, CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv",                   false } },
        { Path::_SQUARE, CSVTrajectory{ DEPLOY_DIR "square.csv",                             false } },
        { Path:: LEAVE, CSVTrajectory{ DEPLOY_DIR "leave.csv",                               false } },
        { Path::LEAVE_GO_TO_UPPER_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation1.csv",  false } },
        { Path::LEAVE_GO_TO_LOWER_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation2.csv",  false } },
        { Path::L1, CSVTrajectory{ DEPLOY_DIR "L1.csv",                                      false } },

    };
    const std::map<Path, CSVTrajectory> redPaths {
        { Path::_TEST, CSVTrajectory{ DEPLOY_DIR "match_winning_auto.csv",                   true  } },
        { Path::_SQUARE, CSVTrajectory{ DEPLOY_DIR "square.csv",                             true  } },
        { Path:: LEAVE, CSVTrajectory{ DEPLOY_DIR "leave.csv",                               true  } },
        { Path::LEAVE_GO_TO_UPPER_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation1.csv",  true  } },
        { Path::LEAVE_GO_TO_LOWER_CORAL, CSVTrajectory{ DEPLOY_DIR "GoToCoralStation2.csv",  true  } },
        { Path::L1, CSVTrajectory{ DEPLOY_DIR "L1.csv",                                      true  } },
    };
    const std::map<Path, CSVTrajectory>* paths = nullptr;

    class ElevatorToL1 : public Action {
      public:
        ElevatorToL1() {}
        Action::Result process() override {
            // Elevator->gotToPreset();
            // Action::Result atPosition = Action::Result::WORKING;
            // if (Elevator->getAtTarget()) {
            //     atPosition = Action::Result::DONE;
            // }
            // return atPosition;
        }
    };
    ElevatorToL1 elevatorToL1;

    class OuttakeCoral : public Action {
      public:
        OuttakeCoral() {}
        Action::Result process() override {
            // calgae->setMotorMode(Calgae::MotorModes::kSHOOT);
            // Action::Result shootDone = Action::Result::WORKING;
            // if (calgae->shootDone()) {
            //     shootDone = Action::Result::DONE;
            //     calgae->setMotorMode(Calgae::MotorModes::kDONE_SHOOTING); //TODO: Make sure this actually stops the motors
            // }
            // return shootDone;
        }
    };
    OuttakeCoral outtakeCoral;

    std::map<u_int32_t, Action*> actions {}; // Actions are in the constructor

    std::string autoSelected;
};