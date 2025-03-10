#pragma once

/// NOTE: This was a desendent of Trajectory.h last year (2024) but it was combined this year (2025)

#include <filesystem>
#include <frc/geometry/Pose2d.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <units/math.h>
#include <map>
#include "Preferences.h"

enum class StartPosition {
    kLEFT,
    kCENTER,
    kRIGHT,
    kSPECIAL
};

enum class ScorePosition {
    kL1,
    kL2,
    kL3,
    kL4,
    kSPECIAL,
};

enum class AlgaePosition {
    kLOW,
    kHIGH,
    kNONE
};

enum class CoralStationPosition {
    kLEFT,
    kRIGHT,
    kNONE,
};

static std::map<std::string, ScorePosition> ScoreMap {
    {"L1", ScorePosition::kL1},
    {"L2", ScorePosition::kL2},
    {"L3", ScorePosition::kL3},
    {"L4", ScorePosition::kL4},
};

static std::map<ScorePosition, std::string> ScoreMapBack {
    {ScorePosition::kL1, "L1"},
    {ScorePosition::kL2, "L2"},
    {ScorePosition::kL3, "L3"},
    {ScorePosition::kL4, "L4"},
};




static std::map<std::string, AlgaePosition> AlgaeMap {
    {"Low", AlgaePosition::kLOW},
    {"High", AlgaePosition::kHIGH},
};

static std::map<AlgaePosition, std::string> AlgaeMapBack {
    {AlgaePosition::kLOW, "Low"},
    {AlgaePosition::kHIGH, "High"},
};



static std::map<std::string, StartPosition> StartMap {
    {"Left", StartPosition::kLEFT},
    {"Center", StartPosition::kCENTER},
    {"Right", StartPosition::kRIGHT},
};


static std::map<StartPosition, std::string> StartMapBack {
    {StartPosition::kLEFT, "Left"},
    {StartPosition::kCENTER, "Center"},
    {StartPosition::kRIGHT, "Right"},
};



static std::map<std::string, CoralStationPosition> CoralStationMap {
    {"Left", CoralStationPosition::kLEFT},
    {"Right", CoralStationPosition::kRIGHT},
};


static std::map<CoralStationPosition, std::string> CoralStationMapBack {
    {CoralStationPosition::kLEFT, "Left"},
    {CoralStationPosition::kRIGHT, "Right"},
};

/**
 * Represents a ThunderAuto-style CSV trajectory for the robot to follow.
 */
class CSVTrajectory {
public:
    CSVTrajectory(std::filesystem::path path, bool inverted, StartPosition startPos, ScorePosition firstScorePos, AlgaePosition algaePos = AlgaePosition::kNONE, CoralStationPosition coralStationPos = CoralStationPosition::kNONE, ScorePosition secondScorePos = ScorePosition::kSPECIAL);
    ~CSVTrajectory();

    /**
     * Represents a single point in a trajectory.
     */
    struct State {
        // The target pose of the robot.
        frc::Pose2d pose;

        // The target velocity of the robot.
        units::meters_per_second_t velocity;
    };

    /**
     * Samples the trajectory at a specified time.
     */
    State sample(units::second_t time) const;

    /**
     * Returns the duration in seconds of the trajectory.
     */
    units::second_t getDuration() const;

    /**
     * Returns the initial position of the robot.
     */
    frc::Pose2d getInitialPose() const;

    /**
     * Returns the final position of the robot.
    */
   frc::Pose2d getFinalPose() const;

    /**
     * Returns the actions with their attributed timestamps.
     */
    inline const std::map<units::second_t, u_int32_t>& getActions() const { return actions; }

    StartPosition startPosition = StartPosition::kCENTER;
    ScorePosition firstScorePosition = ScorePosition::kL1;    
    ScorePosition secondScorePosition = ScorePosition::kL1;
    AlgaePosition algaePosition = AlgaePosition::kNONE;
    CoralStationPosition coralStationPosition = CoralStationPosition::kNONE;

private:
    std::map<units::second_t, State> states;
    std::map<units::second_t, u_int32_t> actions;
};