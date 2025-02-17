#include "Drive/CSVTrajectory.h"
#include "Drive/Parser.h"

CSVTrajectory::CSVTrajectory(std::filesystem::path path, bool inverted) {
    std::string fileStr = Parser::getFile(path);
    if (fileStr.empty()) exit(1);

    Parser::Iter currIter = fileStr.cbegin();

    // Skip the CSV header.
    Parser::parseUntil(currIter, fileStr.cend(), "\n");
    ++currIter;

    // Read each line of the file.
    while (currIter != fileStr.cend()) {
        units::second_t time(Parser::parseNumber(currIter, fileStr.cend())); ++currIter;
        units::meter_t xPos(Parser::parseNumber(currIter, fileStr.cend())); ++currIter;
        units::meter_t yPos(Parser::parseNumber(currIter, fileStr.cend())); ++currIter;
        units::meters_per_second_t velocity(Parser::parseNumber(currIter, fileStr.cend())); ++currIter;
        frc::Rotation2d rotation = units::radian_t(Parser::parseNumber(currIter, fileStr.cend())); ++currIter;
        u_int32_t action = static_cast<u_int32_t>(Parser::parseNumber(currIter, fileStr.cend())); ++currIter;

        if (inverted) {
            xPos = TRAJECTORY_PREFERENCE.FIELD_X - xPos;
            yPos = TRAJECTORY_PREFERENCE.FIELD_Y - yPos;            
            rotation = frc::Rotation2d(rotation.Degrees() + 180_deg);
        }

        frc::Pose2d pose(xPos, yPos, rotation);

        // Add the point to the trajectory.
        states.emplace(time, State{ pose, velocity });
        
        if (action) {
          actions.emplace(time, action);
        }
    }
}

CSVTrajectory::~CSVTrajectory() { }

CSVTrajectory::State CSVTrajectory::sample(units::second_t time) const {
    decltype(states)::const_iterator upperBound = states.upper_bound(time),
                                     lowerBound = states.lower_bound(time);
    
    bool noUpper = (upperBound == states.cend()),
         noLower = (lowerBound == states.cbegin());

    // No defined states D:
    if (noUpper && noLower) {
        return State{ frc::Pose2d(), 0_mps };
    }
    // Return the highest defined state if there is no upper bound.
    else if (noUpper) {
        State s(lowerBound->second);
        s.velocity = 0_mps;
        return s;
    }
    // Return the lowest defined state if there is no lower bound.
    else if (noLower) {
        State s(upperBound->second);
        s.velocity = 0_mps;
        return s;
    }

    auto [upperTime, upperState] = *upperBound;
    auto [lowerTime, lowerState] = *lowerBound;

    // Return the next state if is's close enough.
    if (units::math::abs(upperTime - lowerTime) < 50_ms) {
        return upperState;
    }

    // Linear interpolation.
    double t = (time.value() - lowerTime.value()) / (upperTime.value() - lowerTime.value());
    frc::Pose2d pose = lowerState.pose.Exp(lowerState.pose.Log(upperState.pose) * t);
    units::meters_per_second_t velocity = ((upperState.velocity - lowerState.velocity) * t) + lowerState.velocity;

    return State{ pose, velocity };
}

units::second_t CSVTrajectory::getDuration() const {
    // Reverse iterator to get the last element.
    return states.rbegin()->first;
}

frc::Pose2d CSVTrajectory::getInitialPose() const {
    const State& state(states.cbegin()->second);

    return state.pose;
}

frc::Pose2d CSVTrajectory::getFinalPose() const {
    const State& state(states.cend()->second);

    return state.pose;
}