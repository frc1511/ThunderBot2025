#pragma once

/// NOTE: This was a desendent of Trajectory.h last year (2024) but it was combined this year

#include <filesystem>
#include <frc/geometry/Pose2d.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <units/math.h>
#include <map>
#include "Preferences.h"

/**
 * Represents a ThunderAuto-style CSV trajectory for the robot to follow.
 */
class CSVTrajectory {
public:
    CSVTrajectory(std::filesystem::path path, bool inverted = false);
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

private:
    std::map<units::second_t, State> states;
    std::map<units::second_t, u_int32_t> actions;
};