#include <cpprest/http_listener.h>
#include <cpprest/json.h>
#include <iostream>
#include <thread>
#include <string>

using namespace web;
using namespace http;
using namespace http::experimental::listener;

int main() 
{
    const std::wstring url = L"http://localhost:8080/api/hello";

    std::string utf8_url(url.begin(), url.end());
    uri uri_url(utf8_url);

    http_listener listener(uri_url);

    listener.support(methods::GET, [](http_request message) 
    {
        // Get client details
        auto remote_address = message.remote_address();
        
        // Convert the client's address (which is a narrow string) to a wide string
        std::wstring remote_address_wide(remote_address.begin(), remote_address.end());
        
        // Print client details to std::wcout (wide string stream)
        std::wcout << L"Received GET request from IP: " << remote_address_wide << std::endl;

        // Create a JSON response
        json::value response;
        response[U("message")] = json::value::string(U("Hello, World!"));

        // Send the response
        message.reply(status_codes::OK, response);
    });

    try 
    {
        listener.open().wait();
        std::wcout << L"Listening for requests at: " << url << std::endl;

        std::this_thread::sleep_for(std::chrono::hours(1));

        listener.close().wait();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}

