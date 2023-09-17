
#include "BridgeManager.hpp"
#include "RestHandler.hpp"
#include "Logger.hpp"

RestHandler::RestHandler(const std::string& _url, BridgeManager* _manager)
: m_manager(_manager)
{
    loggerUtility::writeLog(BWR_LOG_DEBUG, "RestHandler::RestHandler()");
    m_listener = std::make_shared<http_listener>(_url);
    m_listener->support(methods::GET, std::bind(&RestHandler::HandleGet, this, std::placeholders::_1));
    m_listener->support(methods::POST, std::bind(&RestHandler::HandlePost, this, std::placeholders::_1));
    m_listener->open().wait();
}

RestHandler::~RestHandler()
{
    loggerUtility::writeLog(BWR_LOG_FATAL, "RestHandler::~RestHandler()");
}

void RestHandler::HandleGet(http_request message)
{
    auto query_parameters = uri::split_query(message.request_uri().query());
    json::value retValue;
    if (query_parameters.find("topic") != query_parameters.end())
    {
        std::string topic = query_parameters["topic"];
        retValue = m_manager->HandleGet(topic);
    }
    else
    {
        loggerUtility::writeLog(BWR_LOG_WARN, "RestHandler::HandleGet(), GET REQUEST IS MISSING TOPIC");
    }
    json::value response;
    response["data"] = retValue;
    message.reply(status_codes::OK, response);
}

void RestHandler::HandlePost(http_request message)
{
    json::value response;
    try
    {
        message.extract_json().then([=](json::value body)
        {
            std::string topic;
            json::value data;
            if (body.has_field("topic") && body.has_field("data"))
            {
                topic = body["topic"].as_string();
                data = body["data"];
                m_manager->HandlePost(topic, data);
                message.reply(status_codes::OK, body);
            }
            else
            {
                loggerUtility::writeLog(BWR_LOG_ERROR, "RestHandler::HandlePost(), BED POST REQUEST, MISSING TOPIC OR DATA");
                message.reply(status_codes::BadRequest);
            }
        });
    }
    catch(...)
    {
        loggerUtility::writeLog(BWR_LOG_ERROR, "RestHandler::HandlePost(), EXCEPTION CAUGHT");
        message.reply(status_codes::InternalError);
    }
}
