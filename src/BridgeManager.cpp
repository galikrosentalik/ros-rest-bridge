#include "BridgeManager.hpp"
#include "Logger.hpp"
#include "MainFrame.hpp"

BridgeManager::BridgeManager()
: m_numOfPostMsgs(0)
, m_numOfGetMsgs(0)
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "BridgeManager::BridgeManager()");
    m_rosHandler = std::make_shared<RosHandler>();
    m_restHandler = std::make_shared<RestHandler>("http://0.0.0.0:8080/RosBridge", this);
}

BridgeManager::~BridgeManager()
{
    loggerUtility::writeLog(BWR_LOG_FATAL, "BridgeManager::~BridgeManager()");
}

json::value BridgeManager::HandleGet(const std::string& _topic)
{
    ++m_numOfGetMsgs;
    json::value pose = m_rosHandler->GetLatestMsg(_topic);
    json::value retValue;
    retValue["topic"] = json::value::string(_topic);
    retValue["data"] = pose;
    return retValue;
}

bool BridgeManager::CheckFieldExistInBody(const std::string& _field, json::value& _body)
{
    if (_body.has_field(_field))
    {
        return true;
    }
    else
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "BridgeManager::CheckFieldExistInBody(), MISSING FIELD, %s", _field.c_str());
        return false;
    }
}

void BridgeManager::HandlePost(const std::string _topic, json::value& _body)
{
    ++m_numOfPostMsgs;
    double x, y, z;
    try
    {
        if(CheckFieldExistInBody("x", _body) && CheckFieldExistInBody("y", _body) && CheckFieldExistInBody("z", _body))
        {
            x = _body["x"].as_number().to_double();
            y = _body["y"].as_number().to_double();
            z = _body["z"].as_number().to_double();
        }
        else
        {
            return;
        }
    }
    catch (const web::json::json_exception& e)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "BridgeManager::HandlePost(), EXCEPTION CAUGHT, %s", e.what());
        // Handle the error as needed
    }
    m_rosHandler->PublishTopic(_topic, x, y, z);

}

void BridgeManager::Routine()
{
    while(ros::ok() && !EndOfWorldAnnouncer::IsEndOfWorldArrive())
    {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        loggerUtility::writeLog(BWR_LOG_INFO, "BridgeManager::Routine(), HANDLED %d POST MSG, %d GET MSG", m_numOfPostMsgs, m_numOfGetMsgs);
        m_numOfPostMsgs = 0;
        m_numOfGetMsgs = 0;
    }
    loggerUtility::writeLog(BWR_LOG_FATAL, "BridgeManager::Routine(), EXITING");
}
