//
// Created by Gal on 3/8/20.
//
#ifndef REST_HANDLER_HPP
#define REST_HANDLER_HPP

#include <cpprest/http_listener.h>
#include <cpprest/json.h>
#include <iostream>
#include <thread>
#include <string>
#include <memory>

using namespace web;
using namespace http;
using namespace http::experimental::listener;

class BridgeManager;

class RestHandler
{
public:
    RestHandler(const std::string& _url, BridgeManager* _manager);
    ~RestHandler();

private:
    void HandleGet(http_request message);
    void HandlePost(http_request message);

private:
    std::shared_ptr<http_listener> m_listener;
    BridgeManager* m_manager;
};

#endif //REST_HANDLER_HPP
