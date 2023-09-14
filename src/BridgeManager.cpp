#include "BridgeManager.hpp"
#include "Logger.hpp"

BridgeManager::BridgeManager()
: m_numOfPostMsgs(0)
, m_numOfGetMsgs(0)
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "BridgeManager::BridgeManager()");
    m_rosHandler = std::make_shared<RosHandler>();
    m_restHandler = std::make_shared<RestHandler>("http://localhost:8080/api/hello", this);
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
    geometry_msgs::Pose pose;
    if (_body.has_field("x"))
    {
        pose.position.x = _body.get("x").as_double();
    }
    if (_body.has_field("y"))
    {
        pose.position.x = _body.get("y").as_double();
    }
    if (_body.has_field("z"))
    {
        pose.position.x = _body.get("z").as_double();
    }
    m_rosHandler->PublishTopic(_topic, pose);
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
