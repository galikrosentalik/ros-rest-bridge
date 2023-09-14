
#include "BridgeManager.hpp"
#include "RestHandler.hpp"
#include "Logger.hpp"

RestHandler::RestHandler(const std::string& _url, BridgeManager* _manager)
: m_manager(_manager)
{
    m_listener = std::make_shared<http_listener>(_url);
    m_listener->support(methods::GET, std::bind(&RestHandler::handle_get, this, std::placeholders::_1));
    m_listener->support(methods::POST, std::bind(&RestHandler::handle_post, this, std::placeholders::_1));
    m_listener->open().wait();
}

RestHandler::~RestHandler()
{

}

void RestHandler::handle_get(http_request message)
{
    auto query_parameters = uri::split_query(message.request_uri().query());
    json::value retValue;
    if (query_parameters.find("topic") != query_parameters.end())
    {
//        retValue = m_manager->handle_get()
    }
    else
    {
        loggerUtility::writeLog(BWR_LOG_WARN, "RestHandler::handle_get(), GET REQUEST IS MISSING TOPIC");
    }
    json::value response;
    response["data"] = retValue;
    message.reply(status_codes::OK, response);
}

void RestHandler::handle_post(http_request message)
{
//    m_manager->handle_post()
    auto remote_address = message.remote_address();
    std::cout << "Received POST request from IP: " << remote_address << std::endl;

    json::value response;
    response[U("message")] = json::value::string(U("Received POST request!"));

    message.reply(status_codes::OK, response);
}
