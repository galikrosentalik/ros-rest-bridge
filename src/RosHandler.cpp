#include "RosHandler.hpp"

RosHandler::RosHandler()
{
    static int dummy;
    std::string name = "RestBridge";
    ros::init(dummy, nullptr, name);
    m_node = std::make_shared<ros::NodeHandle>();
}

RosHandler::~RosHandler(){}

void RosHandler::SubscribeTopic(const std::string& _topic)
{
    m_subscribersMtx.lock();
    if(m_subscribers.find(_topic) == m_subscribers.end())
    {
        m_subscribers[_topic] = CallBackHandler(_topic, m_node);
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
    return m_subscribers[_topic].GetLatestMsg();
}

void RosHandler::PublishTopic(const std::string& _topic, geometry_msgs::Pose _msg)
{

}