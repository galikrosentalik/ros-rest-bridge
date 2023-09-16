#include "BridgeManager.hpp"
#include "Logger.hpp"

BridgeManager::BridgeManager()
: m_numOfPostMsgs(0)
, m_numOfGetMsgs(0)
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "BridgeManager::BridgeManager()");
    m_rosHandler = std::make_shared<RosHandler>();
    m_restHandler = std::make_shared<RestHandler>("http://localhost:8080/RosBridge", this);
}

BridgeManager::~BridgeManager(){}

json::value BridgeManager::handle_get(const std::string& _topic)
{
    ++m_numOfGetMsgs;
    geometry_msgs::Pose pose = m_rosHandler->GetLatestMsg(_topic);
    json::value retValue;
    json::value data;
    data["x"] = json::value::number(pose.position.x);
    data["y"] = json::value::number(pose.position.y);
    data["z"] = json::value::number(pose.position.z);
    retValue["topic"] = json::value::string(_topic);
    retValue["data"] = data;
    return retValue;
}

void BridgeManager::handle_post(const std::string _topic, const json::value& _body)
{
    ++m_numOfPostMsgs;
    double x, y, z;
    std::cout << _topic << std::endl;
    std::cout << _body << std::endl;
    if (_body.has_field("x"))
    {
        x = _body.get("x").as_double();
    }
    else
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "BridgeManager::handle_post(), MISSING FIELD, X");
        return;
    }
    if (_body.has_field("y"))
    {
        x = _body.get("y").as_double();
    }
    else
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "BridgeManager::handle_post(), MISSING FIELD, Y");
        return;
    }
    if (_body.has_field("z"))
    {
        x = _body.get("z").as_double();
    }
    else
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "BridgeManager::handle_post(), MISSING FIELD, Z");
        return;
    }
    loggerUtility::writeLog(BWR_LOG_FATAL, "22");
    m_rosHandler->PublishTopic(_topic, x, y, z);
}

void BridgeManager::Routine()
{
    while(ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        loggerUtility::writeLog(BWR_LOG_INFO, "BridgeManager::Routine(), HANDLED %d POST MSG, %d GET MSG", m_numOfPostMsgs, m_numOfGetMsgs);
        m_numOfPostMsgs = 0;
        m_numOfGetMsgs = 0;
    }
}
