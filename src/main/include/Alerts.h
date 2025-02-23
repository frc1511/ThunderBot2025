#pragma once

#include "Libraries/elasticlib.h"
#include "Basic/IOMap.h"

class Alert {
  public:

// MARK: Component Disable Alerts

    inline static const elastic::Notification driveAlert = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Drive Disabled",
                                                .description = "Drive Disabled",
                                                .displayTime = 3_s,};
    inline static const elastic::Notification calgaeAlert = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Calgae Disabled",
                                                .description = "Calgae Disabled",
                                                .displayTime = 6_s,};
    inline static const elastic::Notification wristAlert = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Wrist Disabled",
                                                .description = "Wrist Disabled",
                                                .displayTime = 9_s,};
    inline static const elastic::Notification elevatorAlert = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Elevator Disabled",
                                                .description = "Elevator Disabled",
                                                .displayTime = 12_s,};
    inline static const elastic::Notification controlsAlert = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Controls Disabled",
                                                .description = "Controls Disabled",
                                                .displayTime = 15_s,};
    inline static const elastic::Notification autoAlert = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Autonomous Disabled",
                                                .description = "Autonomous Disabled",
                                                .displayTime = 18_s,};
    inline static const elastic::Notification blinkyBlinkyAlert = {.level = elastic::NotificationLevel::WARNING,
                                                .title = "Blinky Blinky Disabled",
                                                .description = "Blinky Blinky Disabled",
                                                .displayTime = 21_s,};

    static void sendComponentDisableAlerts() {
        // Check IOMap for these bad boys
        #ifndef ENABLE_DRIVE
            elastic::SendNotification(driveAlert);
        #endif
        #ifndef ENABLE_CALGAE
            elastic::SendNotification(calgaeAlert);
        #endif
        #ifndef ENABLE_WRIST
            elastic::SendNotification(wristAlert);
        #endif
        #ifndef ENABLE_ELEVATOR
            elastic::SendNotification(elevatorAlert);
        #endif
        #ifndef ENABLE_CONTROLS
            elastic::SendNotification(controlsAlert);
        #endif
        #ifndef ENABLE_AUTO
            elastic::SendNotification(autoAlert);
        #endif
        #ifndef ENABLE_BLINKY_BLINKY
            elastic::SendNotification(blinkyBlinkyAlert);
        #endif
    }

// MARK: Controller Alerts

    inline static const elastic::Notification driveDisabledAlert = {.level = elastic::NotificationLevel::WARNING,
                                                                    .title = "Drive Disabled",
                                                                    .description = "Drive Disabled or Drive Controller Disconnected",
                                                                    .displayTime = 10_s,};
    inline static const elastic::Notification auxDisabledAlert   = {.level = elastic::NotificationLevel::WARNING,
                                                                    .title = "Aux Disabled",
                                                                    .description = "Aux Disabled or Aux Controller Disconnected",
                                                                    .displayTime = 10_s,};

    inline static bool driveDisableAlertShown = false;
    inline static bool auxDisableAlertShown = false;

    static void reAllowControllerAlerts() {
        driveDisableAlertShown = false;
        auxDisableAlertShown = false;
    }

    static void sendControllerDisableAndDisconnectedAlerts(bool auxConnected, bool driveConnected) {
        if (!auxDisableAlertShown && !auxConnected) {
            elastic::SendNotification(auxDisabledAlert);
            auxDisableAlertShown = true;
        }

        if (!driveDisableAlertShown && !driveConnected) {
            elastic::SendNotification(driveDisabledAlert);
            driveDisableAlertShown = true;
        }
    }
};