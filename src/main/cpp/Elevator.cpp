#include "Elevator.h" 

void Elevator::process() {
    switch (controlMode) {
        case kMANUAL:
            double speed = 0.1;
            break;
        case kPRESET:
            currentDistanceToTarget = ElevatorPositionValues[targetPosition] - getPosition();
            currentDistanceToTarget = std::clamp(currentDistanceToTarget, minHeight, maxHeight);
            currentAbsoluteDistanceToTarget = abs(currentDistanceToTarget);
            if (currentAbsoluteDistanceToTarget >= 0.01 && !getAtMinHeight()) {
                if (!getAtMaxHeight && currentDistanceToTarget > 0) {
                    double speed = 0.1;
                    setSpeed(speed);
                } else if (!getAtMinHeight()  && currentDistanceToTarget < 0) {
                    double speed = -0.1;
                    setSpeed(speed);
                }
            }
            break;
        case kSTOP:
            double speed = 0.0;
            break;
        default:
            break;
    }
}

void Elevator::getDistanceTraveled() {
    
}

bool Elevator::getAtMaxHeight() {
    return getPosition() >= maxHeight ? true : false;
}

bool Elevator::getAtMinHeight() {
    return getPosition() <= minHeight ? true : false;
}

double Elevator::getPosition() {
    return ((leftEncoder.GetPosition() + rightEncoder.GetPosition())/2);
}

void Elevator::goToPosition(ElevatorPositions target) {
    controlMode = ElevatorControlModes::kPRESET;
    targetPosition = target;
    initialDistanceToTarget = abs(ElevatorPositionValues[targetPosition] - getPosition());
}

void Elevator::setSpeed(double speed) {
    leftSparkMax.Set(speed);
    rightSparkMax.Set(speed);
}