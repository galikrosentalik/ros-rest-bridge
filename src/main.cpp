#include "BridgeManager.hpp"
#include "Logger.hpp"
#include "MainFrame.hpp"

int main(int argc, char **argv)
{
    loggerUtility::configure({ {"type", "file"}, {"file_path", "/tmp"}, {"file_name", "Ros-Rest-bridge"}, {"reopen_interval", "1"} });
    EndOfWorldAnnouncer::Init();
    loggerUtility::writeLog(BWR_LOG_INFO, "Ros-Rest-bridge application starts");
    BridgeManager obj;
    obj.Routine();
    loggerUtility::writeLog(BWR_LOG_FATAL, "main()");
    return 0;
}