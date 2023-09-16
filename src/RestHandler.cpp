
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
        std::string topic = query_parameters["topic"];
        retValue = m_manager->handle_get(topic);
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
    loggerUtility::writeLog(BWR_LOG_FATAL, "1");
    json::value response;
    try
    {
        message.extract_json().then([=](json::value body)
        {
            std::string topic;
            json::value data;
            // Check if the "topic" and "data" fields are present
            if (body.has_field("topic") && body.has_field("data"))
            {
                topic = utility::conversions::to_utf8string(body["topic"].as_string()); // Convert to std::string
                data = body["data"];
                //response = *(const_cast<json::value>(&body));
                // Now you can work with the "topic" and "data" fields as std::string and json::value, respectively
                std::cout << "Topic: " << topic << std::endl;
                std::cout << "Data - x: " << data[U("x")].as_double() << ", y: " << data[U("y")].as_double() << ", z: " << data[U("z")].as_double() << std::endl;
            }
            else
            {
                // Handle the case where required fields are missing
                // You may want to return an error response
                // or take appropriate action based on your application logic
                message.reply(status_codes::OK);
            }
            m_manager->handle_post(topic, data);
            // Now you can use the 'topic' variable as a std::string
        });
        message.reply(status_codes::OK, response);
    }
    catch(...)
    {
        loggerUtility::writeLog(BWR_LOG_WARN, "RestHandler::handle_post(), EXCEPTION CAUGHT");
    }
    loggerUtility::writeLog(BWR_LOG_FATAL, "3");
}
