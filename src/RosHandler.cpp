#include <thread>

#include "RosHandler.hpp"
#include "Logger.hpp"

RosHandler::RosHandler()
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "RosHandler::RosHandler()");
    static int dummy;
    std::string name = "RestBridge";
    ros::init(dummy, nullptr, name);
    m_node = std::make_shared<ros::NodeHandle>();
    loggerUtility::writeLog(BWR_LOG_DEBUG, "RosHandler::RosHandler(), ROS INIT DONE");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    SubscribeTopic("/first_topic");
    SubscribeTopic("/another_topic");
    SubscribeTopic("/also_a_topic");
}

RosHandler::~RosHandler(){}

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

geometry_msgs::Pose RosHandler::GetLatestMsg(const std::string& _topic)
{
    if(m_subscribers.find(_topic) == m_subscribers.end())
    {
        SubscribeTopic(_topic);
        //sleep? conditional variable ? a way to verify the call back is already called
    }
    return m_subscribers[_topic]->GetLatestMsg();
}

void RosHandler::PublishTopic(const std::string& _topic, geometry_msgs::Pose _msg)
{
    if(m_publishers.find(_topic) == m_publishers.end())
    {
        m_publishers[_topic] = std::make_pair<ros::Publisher, std::shared_ptr<std::mutex>>(m_node->advertise<geometry_msgs::Pose>(_topic, 1), std::make_shared<std::mutex>());
    }
    m_publishers[_topic].second->lock();
    m_publishers[_topic].first.publish(_msg);
    m_publishers[_topic].second->unlock();
}
