#include <signal.h> //signal handler
#include "Logger.hpp"
#include "MainFrame.hpp"

bool EndOfWorldAnnouncer::m_isEndOfWorldArrive = false;
bool EndOfWorldAnnouncer::m_initAlready = false;
int EndOfWorldAnnouncer::m_numOfSignals = 0;

static void SignalHandler(int _sig)
{
    EndOfWorldAnnouncer::IncrementNumOfSignals();
    if (EndOfWorldAnnouncer::GetNumOfSignals() >= 5)
    {
        loggerUtility::writeLog(BWR_LOG_FATAL,"EndOfWorldAnnouncer::SignalHandler(), GOT TOO MANY SIGNALS, BRUTALLY CLOSING APPLICATION");
        exit(1);
    }
    std::string signalName;
    switch (_sig)
    {
        case SIGINT:
        {
            signalName = "SIGINT";
            break;
        }
        case SIGTERM:
        {
            signalName = "SIGTERM";
            break;
        }
        case SIGQUIT:
        {
            signalName = "SIGQUIT";
            break;
        }
        case SIGTSTP:
        {
            signalName = "SIGTSTP";
            break;
        }
        case SIGHUP:
        {
            signalName = "SIGHUP";
            break;
        }
        case SIGCONT:
        {
            signalName = "SIGCONT";
            break;
        }
        case SIGKILL:
        {
            signalName = "SIGKILL";
            break;
        }
        case SIGABRT:
        {
            signalName = "SIGABRT";
            break;
        }
        case SIGSEGV:
        {
            signalName = "SIGSEGV";
            break;
        }
        default:
        {
            signalName = "UNFAMILIAR SIGNAL=" + std::to_string(_sig);
            break;
        }
    }
    loggerUtility::writeLog(loggerUtility::log_level::FATAL,"EndOfWorldAnnouncer::SignalHandler(), GOT %s SIGNAL, %d", signalName.c_str(), EndOfWorldAnnouncer::GetNumOfSignals());
    EndOfWorldAnnouncer::AnnounceToAllEndOfWorldArrive();
}

void EndOfWorldAnnouncer::Init()
{
    if (m_initAlready)
    {
        return;
    }
    loggerUtility::writeLog(loggerUtility::log_level::TRACE,"EndOfWorldAnnouncer::init()");
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    signal(SIGQUIT, SignalHandler);
    signal(SIGTSTP, SignalHandler);
    signal(SIGHUP, SignalHandler);
    signal(SIGCONT, SignalHandler);
    signal(SIGKILL, SignalHandler);
    signal(SIGABRT, SignalHandler);
    signal(SIGSEGV, SignalHandler);
    m_initAlready = true;
}

