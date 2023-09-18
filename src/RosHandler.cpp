#include <thread>

#include "RosHandler.hpp"
#include "Logger.hpp"

RosHandler::RosHandler()
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "RosHandler::RosHandler()");
    static int dummy;
    std::string name = "ros_rest_relay";
    ros::init(dummy, nullptr, name);
    m_node = std::make_shared<ros::NodeHandle>();
    loggerUtility::writeLog(BWR_LOG_DEBUG, "RosHandler::RosHandler(), ROS INIT DONE");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    SubscribeTopic("/first_topic");
    SubscribeTopic("/another_topic");
    SubscribeTopic("/also_a_topic");
    m_trd = std::make_shared<std::thread>(&RosHandler::Routine, this);
}

RosHandler::~RosHandler()
{
    loggerUtility::writeLog(BWR_LOG_FATAL, "RosHandler::~RosHandler()");
    m_trd->join();
}

void RosHandler::Routine()
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "RosHandler::Routine(), STARTING ROUTINE");
    ros::spin();
    loggerUtility::writeLog(BWR_LOG_FATAL, "RosHandler::Routine(), EXITING ROUTINE");
}

void RosHandler::SubscribeTopic(const std::string& _topic)
{
    m_subscribersMtx.lock();
    if(m_subscribers.find(_topic) == m_subscribers.end())
    {
        loggerUtility::writeLog(BWR_LOG_INFO, "RosHandler::SubscribeTopic(), SUBSCRIBING TO NEW TOPIC, %s", _topic.c_str());
        m_subscribers[_topic] = std::make_shared<CallBackHandler>(_topic, m_node);
    }
    m_subscribersMtx.unlock();
}

web::json::value RosHandler::GetLatestMsg(const std::string& _topic)
{
    if(m_subscribers.find(_topic) == m_subscribers.end())
    {
        SubscribeTopic(_topic);
        //sleep? conditional variable ? a way to verify the call back is already called
    }
    geometry_msgs::Pose stat = m_subscribers[_topic]->GetLatestMsg();
    web::json::value retValue;
    retValue["x"] = web::json::value::number(stat.position.x);
    retValue["y"] = web::json::value::number(stat.position.y);
    retValue["z"] = web::json::value::number(stat.position.z);
    return retValue;
}

void RosHandler::PublishTopic(const std::string& _topic, double _x, double _y, double _z)
{
    if(m_publishers.find(_topic) == m_publishers.end())
    {
        loggerUtility::writeLog(BWR_LOG_INFO, "RosHandler::PublishTopic(), CREATING NEW PUBLISHER, %s", _topic.c_str());
        m_publishers[_topic] = std::make_pair<ros::Publisher, std::shared_ptr<std::mutex>>(m_node->advertise<geometry_msgs::Pose>(_topic, 1), std::make_shared<std::mutex>());
    }
    geometry_msgs::Pose msg;
    msg.position.x = _x;
    msg.position.y = _y;
    msg.position.z = _z;
    m_publishers[_topic].second->lock();
    m_publishers[_topic].first.publish(msg);
    m_publishers[_topic].second->unlock();
}
