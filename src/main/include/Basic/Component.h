#pragma once

#include <Basic/Settings.h>

#include <frc/DriverStation.h>
#include "units/current.h"

class Robot;

class Component {
  public:
    virtual ~Component();

    enum class MatchMode {
        DISABLED,
        AUTO,
        TELEOP,
        TEST,
    };
    /**
     * Component should do calibrations and reset/configure hardware for correct power-on usage
     * This should save things in a non-volatile/persistant configuration
     * This will always be followed by resetToMatchMode() to restore correct runtime operations for that specific mode
     */
    virtual void doPersistantConfiguration();

    /**
     * Resets the component to run in the given configuration
     * This should reset internal states to their operational points (i.e. ready to go for the mode)
     * This should also abandon any safely cancellable actions (i.e. Drive)
     */
    virtual void resetToMatchMode(MatchMode mode);

    /**
     * Send operational and/or diagnostic info/feedback to the SmartDashboard
     * Should be called periodically in all modes, other than when manually disabled
     */
    virtual void sendFeedback();

    /**
     * The teleop periodic function. Should be employed for basic, TeleOperated operations
     */
    virtual void teleOpProcess();

    /**
     * The auto periodic function. Should be employed for basic, Autonomonous operations
     */
    virtual void autoProcess();

    /**
     * Should return the total current draw of the mechanism 
     */
    virtual units::ampere_t getCurrent();

  protected:
    MatchMode getMatchMode();

    MatchMode getLastMode();
    
    static Settings settings;
  private:
    MatchMode lastMode;

    void callResetToMode(Component::MatchMode lastMode);

    friend class Robot;
};