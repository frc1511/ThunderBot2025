#pragma once

#include "Basic/Settings.h"

#include <frc/DriverStation.h>

class Robot;

class Component {
  public:

    virtual ~Component() = default; // Fixes a warning

    enum class MatchMode {
        DISABLED,
        AUTO,
        TELEOP,
        TEST,
    };

    /**
     * Component should do calibrations and reset/configure hardware for correct power-on usage
     * This should save things in a non-volatile/persistent configuration
     * This will always be followed by resetToMatchMode() to restore correct runtime operations for that specific mode
     */
    virtual inline void doPersistentConfiguration() {};

    /**
     * Resets the component to run in the given configuration
     * This should reset internal states to their operational points (i.e. ready to go for the mode)
     * This should also abandon any safely cancellable actions (i.e. Drive)
     */
    virtual void resetToMatchMode(MatchMode priorMode, MatchMode mode) {};

    /**
     * Send operational and/or diagnostic info/feedback to the SmartDashboard
     * Should be called periodically in all modes, other than when manually disabled
     */
    virtual void sendFeedback() {};

    /**
     * The periodic function. Should be employed for basic, TeleOperated and Autonomonous operations
     */
    virtual void process() {};

  protected:
    static Settings settings;


};