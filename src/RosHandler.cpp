#include <thread>
#include <limits>
#include <cmath>

#include "MainFrame.hpp"
#include "RosHandler.hpp"
#include "Logger.hpp"

RosHandler::RosHandler()
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "RosHandler::RosHandler()");
    static int dummy;
    std::string name = "ros_rest_relay";
    try
    {
        ros::init(dummy, nullptr, name);
        m_node = std::make_shared<ros::NodeHandle>();
        loggerUtility::writeLog(BWR_LOG_DEBUG, "RosHandler::RosHandler(), ROS INIT DONE");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        SubscribeTopic("/first_topic");
        SubscribeTopic("/another_topic");
        SubscribeTopic("/also_a_topic");
        m_trd = std::make_shared<std::thread>(&RosHandler::Routine, this);
    }
    catch (const ros::Exception& e)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::RosHandler(), ROS EXCEPTION: %s", e.what());
        EndOfWorldAnnouncer::AnnounceToAllEndOfWorldArrive();
    }
    catch (...)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::RosHandler(), AN UNKNOWN EXCEPTION OCCURRED");
        EndOfWorldAnnouncer::AnnounceToAllEndOfWorldArrive();
    }
}

RosHandler::~RosHandler()
{
    loggerUtility::writeLog(BWR_LOG_FATAL, "RosHandler::~RosHandler()");
    m_trd->join();
}

void RosHandler::Routine()
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "RosHandler::Routine(), STARTING ROUTINE");
    try
    {
        ros::spin();
    }
    catch (const ros::Exception& e)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::Routine(), ROS EXCEPTION: %s", e.what());
        EndOfWorldAnnouncer::AnnounceToAllEndOfWorldArrive();
    }
    catch (...)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::Routine(), AN UNKNOWN EXCEPTION OCCURRED");
        EndOfWorldAnnouncer::AnnounceToAllEndOfWorldArrive();
    }
    loggerUtility::writeLog(BWR_LOG_FATAL, "RosHandler::Routine(), EXITING ROUTINE");
}

void RosHandler::SubscribeTopic(const std::string& _topic)
{
    m_subscribersMtx.lock();
    if(m_subscribers.find(_topic) == m_subscribers.end())
    {
        loggerUtility::writeLog(BWR_LOG_INFO, "RosHandler::SubscribeTopic(), SUBSCRIBING TO NEW TOPIC, %s", _topic.c_str());
        try
        {
            m_subscribers[_topic] = std::make_shared<CallBackHandler>(_topic, m_node);
        }
        catch (const ros::Exception& e)
        {
            loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::SubscribeTopic(), ROS EXCEPTION: %s", e.what());
        }
        catch (...)
        {
            loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::SubscribeTopic(), AN UNKNOWN EXCEPTION OCCURRED");
        }
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
    web::json::value retValue;
    try
    {
        geometry_msgs::Pose stat = m_subscribers[_topic]->GetLatestMsg();
        retValue["x"] = web::json::value::number(stat.position.x);
        retValue["y"] = web::json::value::number(stat.position.y);
        retValue["z"] = web::json::value::number(stat.position.z);
    }
    catch (...)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::GetLatestMsg(), AN UNKNOWN EXCEPTION OCCURRED");
        retValue["x"] = std::numeric_limits<double>::quiet_NaN();
        retValue["y"] = std::numeric_limits<double>::quiet_NaN();
        retValue["z"] = std::numeric_limits<double>::quiet_NaN();
    }
    return retValue;
}

void RosHandler::PublishTopic(const std::string& _topic, double _x, double _y, double _z)
{
    if(m_publishers.find(_topic) == m_publishers.end())
    {
        loggerUtility::writeLog(BWR_LOG_INFO, "RosHandler::PublishTopic(), CREATING NEW PUBLISHER, %s", _topic.c_str());
        m_publishers[_topic] = std::make_pair<ros::Publisher, std::shared_ptr<std::mutex>>(m_node->advertise<geometry_msgs::Pose>(_topic, 1), std::make_shared<std::mutex>());
    }
    try
    {
        geometry_msgs::Pose msg;
        msg.position.x = _x;
        msg.position.y = _y;
        msg.position.z = _z;
        m_publishers[_topic].second->lock();
        m_publishers[_topic].first.publish(msg);
        m_publishers[_topic].second->unlock();
    }
    catch (const ros::Exception& e)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::PublishTopic(), ROS EXCEPTION: %s", e.what());
    }
    catch (...)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "RosHandler::PublishTopic(), AN UNKNOWN EXCEPTION OCCURRED");
    }
}
