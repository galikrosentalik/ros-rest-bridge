#include "BridgeManager.hpp"
#include "Logger.hpp"

int main(int argc, char **argv)
{
    loggerUtility::configure({ {"type", "file"}, {"file_path", "/tmp"}, {"file_name", "Ros-Rest-bridge"}, {"reopen_interval", "1"} });
    loggerUtility::writeLog(BWR_LOG_INFO, "Ros-Rest-bridge application starts");
    BridgeManager obj;
    obj.Routine();
    return 0;
}