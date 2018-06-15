#pragma once
#include <mrpt/web/CPubSubManagerBase.h>
#include "CWebSocketServer.h"
#include <iostream>
#include <thread>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <functional>

namespace mrpt::web
{
    using ConnectionPointer  = std::shared_ptr<WsServer::Connection>;
    //class  derived from pure abstract class CPubSubManagerBase
    template<typename SERVER>
    class CPubSubManager: public CPubSubManagerBase
    {
    private:
    class TopicDataStore
    {
    public:
        TopicDataStore(const std::string& topic_) : m_topic(topic_) {}
        inline std::vector<ConnectionPointer> getConnections()
        {
            return std::vector<ConnectionPointer>(m_connections.begin(), m_connections.end());
        }
        inline void addConnection(ConnectionPointer _conn)
        {
            m_connections.insert(_conn);
        }
        inline void removeConnection(ConnectionPointer _conn)
        {
            m_connections.erase(_conn);
        }
    private:
        std::string m_topic;
        std::unordered_set<ConnectionPointer> m_connections;
    };
    public:
    CPubSubManager(SERVER* connector_) : m_connector(connector_) {}
    inline virtual int addSubscriptionToTopic(const std::string& topic, ConnectionPointer _conn)
    {
        if(m_subscriptions.find(topic) == m_subscriptions.end())
        {
            m_subscriptions[topic] = std::make_shared<TopicDataStore>(topic);
        }
        auto &store = m_subscriptions[topic];
        store->addConnection(_conn); 
    }
    inline virtual void removeSubscriptionFromTopic(const std::string& topic, ConnectionPointer _conn)
    {
        if(m_subscriptions.find(topic) == m_subscriptions.end()) return;
        auto &store = m_subscriptions[topic];
        store->removeConnection(_conn);
    }
    inline virtual void publishMessageToTopic(const std::string& topic,const Json::Value& msg)
    {
        return;
    }
    inline virtual const std::vector<ConnectionPointer> getSubscribedConnections(const std::string& topic)
    {
        if(m_subscriptions.find(topic) == m_subscriptions.end()) return std::vector<ConnectionPointer>();
        auto &store = m_subscriptions[topic];
        return store->getConnections();
    }
    inline virtual bool checkConnectionLive(ConnectionPointer _conn)
    {
        return m_connector->checkConnectionLive(_conn);
    }
    private:
        std::unordered_map<std::string, std::shared_ptr<TopicDataStore>> m_subscriptions;
        // std::unordered_map<std::string, std::shared_ptr<Publisher<>> m_publishers;
        SERVER* m_connector;
    };
}