//
// Created by Gal on 3/8/20.
//

#ifndef ROS_HANDLER_HPP
#define ROS_HANDLER_HPP

#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <cpprest/json.h>

class CallBackHandler
{
public:
    CallBackHandler(const std::string& _topic, std::shared_ptr<ros::NodeHandle> _node)
    : m_topic(_topic)
    {
        m_subscriber = _node->subscribe(m_topic, 10, &CallBackHandler::CallBack, this, ros::TransportHints().tcpNoDelay(true));
        m_lastMsg.position.x = std::numeric_limits<double>::quiet_NaN();
        m_lastMsg.position.y = std::numeric_limits<double>::quiet_NaN();
        m_lastMsg.position.z = std::numeric_limits<double>::quiet_NaN();
    }
    geometry_msgs::Pose GetLatestMsg() { return m_lastMsg; }

private:
    void CallBack(geometry_msgs::Pose _msg)
    {
        m_lastMsg = _msg;
    }

private:
    std::string m_topic;
    geometry_msgs::Pose m_lastMsg;
    ros::Subscriber m_subscriber;
};

class RosHandler
{
public:
    RosHandler();
    ~RosHandler();
    web::json::value GetLatestMsg(const std::string& _topic); //in case the topic is not subscirbed yet, will subscribe and block untill the topic is published
    void PublishTopic(const std::string& _topic, double _x, double _y, double _z);

private:
    void Routine();
    void SubscribeTopic(const std::string& _topic);
    bool CreateNewPublisher(const std::string& _topic);

private:
    std::mutex m_publishersMtx;
    std::map<std::string, std::pair<ros::Publisher, std::shared_ptr<std::mutex> > > m_publishers;
    std::mutex m_subscribersMtx;
    std::map<std::string, std::shared_ptr<CallBackHandler> > m_subscribers;
    std::shared_ptr<ros::NodeHandle> m_node;
    std::shared_ptr<std::thread> m_trd;
};

#endif //ROS_HANDLER_HPP
