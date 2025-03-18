#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>

struct ThunderLogger {
    ThunderLogger() {}

  private:
    inline static bool loggerActive = false;

  public:
    static void StartLogging() {
        if (!loggerActive) {
            loggerActive = true;
            frc::DataLogManager::Start();
            frc::DataLogManager::LogNetworkTables(true);
            frc::DataLogManager::LogConsoleOutput(true);
            frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
        }
    }

    static void StopLogging() {
        if (loggerActive) {
            frc::DataLogManager::Stop();
            loggerActive = false;
        }
    }
};