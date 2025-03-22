#pragma once

#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <iostream>
#include <filesystem>

#include "Drive/CSVTrajectory.h"
#include "Basic/Component.h"
#include "Auto/Action.h"
#include "Gamepiece/Gamepiece.h"

#include "Drive/Drive.h"

#define DEPLOY_DIR "/home/lvuser/deploy/"

#define LEAVE_ID "Leave"
#define DO_NOTHING_ID "DO_NOTHING.nothingShouldHaveThisFileName"

typedef std::string AutoID;

class Auto : public Component {

public:
    Auto(Drive *drive_, Limelight *limelight_, Gamepiece *gamepiece_);

    void autoImportAutoAutos();

    void process() override;
    void sendFeedback() override;
    void resetToMatchMode(MatchMode priorMode, MatchMode mode) override;

    frc::SendableChooser<int> autoSelector;
    void autoSelectorInit();

private:
    Drive *drive;
    Limelight *limelight;
    Gamepiece *gamepiece;

    // Match Autos
    void doNothing();
    void leave();
    void runPath();

    AutoID autoId = DO_NOTHING_ID;

    std::map<AutoID, int> convertTable {
        {DO_NOTHING_ID, 0},
        {LEAVE_ID, 1},
    };

    int step = 0;
    std::map<AutoID, CSVTrajectory> redPaths {};
    std::map<AutoID, CSVTrajectory> bluePaths {};
    const std::map<AutoID, CSVTrajectory>* paths = nullptr;

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
    ToGPPreset toBarge;
    ToGPPreset toCoralStation;
    ToGPPreset toReefLow;
    ToGPPreset toReefHigh;
    ToGPPreset toProcessor;

    class ShootCoral : public Action {
      public:
        ShootCoral(Gamepiece *gamepiece_) : gamepiece(gamepiece_) {};
        Gamepiece *gamepiece;
        Action::Result process() override {
            if (gamepiece->calgae == nullptr) 
                return Action::Result::DONE;
                
            if (!gamepiece->calgae->isAutoShooting) {
                gamepiece->calgae->autoShoot();
            }
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
            if (gamepiece->calgae == nullptr) {
                return Action::Result::DONE;
            }

            if (!gamepiece->calgae->isAutoIntaking) {
                gamepiece->calgae->autoIntake(gp,true);
            }
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
            if (!drive->isLineUpDone()) {
                drive->beginLineup(isLeft, isL4);
            }
            return drive->isLineUpDone() ? Action::Result::DONE : Action::Result::WORKING;
        };
    };
    AutoAlign autoAlignLeftNormal;
    AutoAlign autoAlignRightNormal;
    AutoAlign autoAlignLeftL4;
    AutoAlign autoAlignRightL4;
    class StartAlgaeIntake : public Action {
      public:
        StartAlgaeIntake(Gamepiece *gamepiece_, Calgae::GamepieceState gp_) : gamepiece(gamepiece_), gp(gp_) {};
        Gamepiece *gamepiece;
        Calgae::GamepieceState gp;
        Action::Result process() override {
            if (gamepiece->calgae == nullptr) {
                return Action::Result::DONE;
            }

            if (!gamepiece->calgae->isAutoIntaking) {
                gamepiece->calgae->autoIntake(gp,false);
            }
            return Action::Result::DONE;
        };            
    };
    StartAlgaeIntake startAlgaeIntake;
    class StopAlgaeIntake : public Action{
        public:
            StopAlgaeIntake(Gamepiece *gamepiece_) : gamepiece(gamepiece_) {};
            Gamepiece *gamepiece;
            Action::Result process() override {
                if (gamepiece->calgae == nullptr) {
                    return Action::Result::DONE;
                }
                    
                if (gamepiece->calgae->isAutoIntaking) {
                    gamepiece->calgae->stopAutoIntake();
                }
                return Action::Result::DONE;
            };
    };
    StopAlgaeIntake stopAlgaeIntake;
    std::map<u_int32_t, Action*> actions {
        {1 << 0,  &autoAlignLeftNormal},    // Normal Left Align
        {1 << 1,  &autoAlignRightNormal},   // Normal Right Align
        {1 << 2,  &autoAlignLeftL4},        // L4 Left Align
        {1 << 3,  &autoAlignRightL4},       // L4 Right Align
        {1 << 4,  &toL1},                   // L1
        {1 << 5,  &toL2},                   // L2
        {1 << 6,  &toL3},                   // L3
        {1 << 7,  &toL4},                   // L4
        {1 << 8,  &toBarge},                // Barge
        {1 << 9,  &toCoralStation},         // Coral Station
        {1 << 10, &toReefLow},              // Reef Low
        {1 << 11, &toReefHigh},             // Reef High
        {1 << 12, &shootCoral},             // Shoot Coral
        {1 << 13, &intakeCoral},            // Intake Coral
        {1 << 14, &intakeAlgae},            // Intake Algae
        {1 << 15, &toTransit},              // Transit
        {1 << 16, &toProcessor},            // Processor
        {1 << 17, &startAlgaeIntake},       // Start Algae Intake
        {1 << 18, &stopAlgaeIntake}         // Stop Algae Intake
    };

    std::string autoSelected;

    bool isAuto = false;
};