#pragma once
#include <mrpt/web/CWebSocket.h>
#include <mrpt/web/json_config.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <memory>

namespace mrpt::web
{
using ConnectionPointer = std::shared_ptr<WsServer::Connection>;
//pure abstract class
class CPubSubManagerBase
{
    // Publisher side work:
    // Contains methods for adding client_id to set published_topics["top"]
    // and for removing client_id from set published_topics["top"]

    // Subscription side work:
    // Contains methods for adding client_id to set subscribed_topics["top"]
    // and for removing client_id from set subscribed_topics["top"]
public:
    virtual void addPublisherToTopic(const std::string& topic, const std::string& type, const bool latch = false, const int queue_size = 100) = 0;
    virtual void removePublisherFromTopic(const std::string& topic) = 0;
    virtual int addSubscriptionToTopic(const std::string& topic, ConnectionPointer _conn) = 0;
    virtual void removeSubscriptionFromTopic(const std::string& topic, ConnectionPointer _conn) = 0;
    virtual void publishMessageToTopic(const std::string& topic,const Json::Value& msg) = 0;
    virtual const std::vector<ConnectionPointer> getSubscribedConnections(const std::string& topic) = 0;
    virtual bool checkConnectionLive(ConnectionPointer _conn)
    {
        throw std::logic_error("Connection Check is not set up.");
        return true;
    }
    CPubSubManagerBase() {}
    ~CPubSubManagerBase() {}
};
}
