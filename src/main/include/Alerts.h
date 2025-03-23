#pragma once

#include "Libraries/elasticlib.h"
#include "Basic/IOMap.h"

#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

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
    inline static const elastic::Notification driveDisconnectedAlert         = {.level = elastic::NotificationLevel::WARNING,
                                                                                .title = "Drive Disconnected",
                                                                                .description = "Drive Disconnected",
                                                                                .displayTime = 2_s,};
    inline static const elastic::Notification auxDisconnectedAlert           = {.level = elastic::NotificationLevel::WARNING,
                                                                                .title = "Aux Disconnected",
                                                                                .description = "Aux Disconnected",
                                                                                .displayTime = 2_s,};
    inline static const elastic::Notification switchBoardDisconnectedAlert   = {.level = elastic::NotificationLevel::WARNING,
                                                                                .title = "Switchboard Disconnected",
                                                                                .description = "Switchboard Disconnected",
                                                                                .displayTime = 2_s,};

    // MARK: Other Errors
    inline static const elastic::Notification wristSensor =  {.level = elastic::NotificationLevel::ERROR,
                                                                .title = "Wrist Disconnected",
                                                                .description = "Wrist Disconnected",
                                                                .displayTime = 1_s,};
    inline static const elastic::Notification batteryAlert =  {.level = elastic::NotificationLevel::WARNING,
                                                                .title = "Battery Voltage Low",
                                                                .description = "Battery Voltage dipped below 9 volts",
                                                                .displayTime = 1_s,};

    inline static bool auxDisconnected = false;
    inline static bool driveDisconnected = false;
    inline static bool switchBoardDisconnected = false;

    static void sendDisconnectAndDisableStates(bool auxConnected, bool driveConnected, bool switchBoardConnected) {
        auxDisconnected = !auxConnected;
        driveDisconnected = !driveConnected;
        switchBoardDisconnected = !driveConnected;
    }

    inline static frc::Timer alertTimer {};

    static void startTimer() {
        alertTimer.Start();
    }

    static void displayAlert(elastic::Notification alert) {
        elastic::SendNotification(alert);
    }

    static void sendFeedback() {
        double currentTime = alertTimer.Get().value();
        static int numberOfAlertsTried = 0;

        static bool lowBattery = false;
        auto voltage = frc::DriverStation::GetBatteryVoltage();
        if (voltage && voltage < 9.0) {
                lowBattery = true;
        }

        frc::SmartDashboard::PutBoolean("Alerts_Low_Battery_Triggered", lowBattery);

        if (currentTime >= 20.0 && numberOfAlertsTried == 10) {
            alertTimer.Restart();
            numberOfAlertsTried = 0;
        } else if (currentTime >= 18.0 && numberOfAlertsTried == 9) {
            numberOfAlertsTried++;
            if (lowBattery) {
                displayAlert(batteryAlert);
            }
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
            if (auxDisconnected) {
                elastic::SendNotification(auxDisconnectedAlert);
            }
        } else if (currentTime >= 0.0 && numberOfAlertsTried == 0) {
            numberOfAlertsTried++;
            if (driveDisconnected) {
                elastic::SendNotification(driveDisconnectedAlert);
            }
        }
    }
};