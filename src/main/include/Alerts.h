#pragma once

#include "Libraries/elasticlib.h"
#include "Basic/IOMap.h"

#include "frc/Timer.h"

class Alert {
  public:

// MARK: Component Disable Alerts

    inline static const elastic::Notification driveAlert =        {.level = elastic::NotificationLevel::INFO,
                                                                   .title = "Drive Disabled",
                                                                   .description = "Drive Disabled",
                                                                   .displayTime = 2_s,};
    inline static const elastic::Notification calgaeAlert =       {.level = elastic::NotificationLevel::INFO,
                                                                   .title = "Calgae Disabled",
                                                                   .description = "Calgae Disabled",
                                                                   .displayTime = 2_s,};
    inline static const elastic::Notification wristAlert =        {.level = elastic::NotificationLevel::INFO,
                                                                   .title = "Wrist Disabled",
                                                                   .description = "Wrist Disabled",
                                                                   .displayTime = 2_s,};
    inline static const elastic::Notification elevatorAlert =     {.level = elastic::NotificationLevel::INFO,
                                                                   .title = "Elevator Disabled",
                                                                   .description = "Elevator Disabled",
                                                                   .displayTime = 2_s,};
    inline static const elastic::Notification autoAlert =         {.level = elastic::NotificationLevel::INFO,
                                                                   .title = "Autonomous Disabled",
                                                                   .description = "Autonomous Disabled",
                                                                   .displayTime = 2_s,};
    inline static const elastic::Notification blinkyBlinkyAlert = {.level = elastic::NotificationLevel::INFO,
                                                                   .title = "Blinky Blinky Disabled",
                                                                   .description = "Blinky Blinky Disabled",
                                                                   .displayTime = 2_s,};

// MARK: Controller Alerts

    inline static const elastic::Notification driveDisabledAlert             = {.level = elastic::NotificationLevel::WARNING,
                                                                                .title = "Drive Disabled",
                                                                                .description = "Drive Disabled or Drive Controller Disconnected",
                                                                                .displayTime = 2_s,};
    inline static const elastic::Notification auxDisabledAlert               = {.level = elastic::NotificationLevel::WARNING,
                                                                                .title = "Aux Disabled",
                                                                                .description = "Aux Disabled or Aux Controller Disconnected",
                                                                                .displayTime = 2_s,};
    inline static const elastic::Notification switchBoardDisconnectedAlert   = {.level = elastic::NotificationLevel::WARNING,
                                                                                .title = "Switchboard Disconnected",
                                                                                .description = "Switchboard Disconnected",
                                                                                .displayTime = 2_s,};

    inline static bool driveDisabled = false;
    inline static bool auxDisabled = false;
    inline static bool switchBoardDisconnected = false;

    static void reAllowControllerAlerts() {
        driveDisabled = false;
        auxDisabled = false;
        switchBoardDisconnected = false;
    }

    static void sendDisconnectAndDisableStates(bool auxConnected, bool driveConnected, bool switchBoardConnected) {
        if (!auxDisabled && !auxConnected) {
            auxDisabled = true;
        }

        if (!driveDisabled && !driveConnected) {
            driveDisabled = true;
        }

        if (!switchBoardDisconnected && !switchBoardConnected) {
            switchBoardDisconnected = true;
        }
    }

    inline static frc::Timer alertTimer {};

    static void startTimer() {
        alertTimer.Start();
    }

    static void sendFeedback() {
        double currentTime = alertTimer.Get().value();
        static int numberOfAlertsTried = 0;

        if (currentTime >= 18.0 && numberOfAlertsTried == 9) {
            alertTimer.Restart();
            numberOfAlertsTried = 0;
        } else if (currentTime >= 16.0 && numberOfAlertsTried == 8) {
            numberOfAlertsTried++;
            #ifndef ENABLE_BLINKY_BLINKY
                elastic::SendNotification(blinkyBlinkyAlert);
            #endif
        } else if (currentTime >= 14.0 && numberOfAlertsTried == 7) {
            numberOfAlertsTried++;
            #ifndef ENABLE_AUTO
                elastic::SendNotification(autoAlert);
            #endif
        } else if (currentTime >= 12.0 && numberOfAlertsTried == 6) {
            numberOfAlertsTried++;
            #ifndef ENABLE_ELEVATOR
                elastic::SendNotification(elevatorAlert);
            #endif
        } else if (currentTime >= 10.0 && numberOfAlertsTried == 5) {
            numberOfAlertsTried++;
            #ifndef ENABLE_WRIST
                elastic::SendNotification(wristAlert);
            #endif
        } else if (currentTime >= 8.0 && numberOfAlertsTried == 4) {
            numberOfAlertsTried++;
            #ifndef ENABLE_CALGAE
                elastic::SendNotification(calgaeAlert);
            #endif

        } else if (currentTime >= 6.0 && numberOfAlertsTried == 3) {
            numberOfAlertsTried++;
            #ifndef ENABLE_DRIVE
                elastic::SendNotification(driveAlert);
            #endif
        } else if (currentTime >= 4.0 && numberOfAlertsTried == 2) {
            numberOfAlertsTried++;
            if (switchBoardDisconnected) {
                elastic::SendNotification(switchBoardDisconnectedAlert);
            }
        } else if (currentTime >= 2.0 && numberOfAlertsTried == 1) {
            numberOfAlertsTried++;
            if (auxDisabled) {
                elastic::SendNotification(auxDisabledAlert);
            }
        } else if (currentTime >= 0.0 && numberOfAlertsTried == 0) {
            numberOfAlertsTried++;
            if (driveDisabled) {
                elastic::SendNotification(driveDisabledAlert);
            }
        }

    }
};