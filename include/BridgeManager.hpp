//
// Created by Gal on 3/8/20.
//
#ifndef __BRIDGE_MANAGER_HPP__
#define __BRIDGE_MANAGER_HPP__

#include "RosHandler.hpp"
#include "RestHandler.hpp"

class BridgeManager
{
public:
    BridgeManager();
    ~BridgeManager();
    void Routine();
    json::value handle_get(const std::string& _topic);
    void handle_post(const std::string _topic, const json::value& _message);
private:

private:
    std::shared_ptr<RosHandler> m_rosHandler;
    std::shared_ptr<RestHandler> m_restHandler;
    int m_numOfPostMsgs;
    int m_numOfGetMsgs;
};

#endif //__BRIDGE_MANAGER_HPP__
