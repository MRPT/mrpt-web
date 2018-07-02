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
    class PublisherMetaData
    {
    public:
      PublisherMetaData(const std::string& topic, const std::string& type, const bool& latch, const int queue_size):
        m_topic(topic), m_type(type), m_latch(latch), m_queue_size(queue_size) {}
      std::string getTopic() { return m_topic; }
      std::string getType() { return m_type; }
      bool getLatch() { return m_latch; }
      int getQueueSize() { return m_queue_size; }
    private:
      const std::string m_topic;
      const std::string m_type;
      const bool m_latch;
      const int m_queue_size;
    };
    public:
    CPubSubManager(SERVER* connector_) : m_connector(connector_) {}
    inline virtual void addPublisherToTopic(const std::string& topic,const std::string& type,const bool latch = false, const int queue_size = 100)
    {
      // std::cout << "New incoming publisher on topic" << topic << std::endl;
    }
    inline virtual void removePublisherFromTopic(const std::string& topic)
    {
      // std::cout << "Removing the topic "<< topic << std::endl;
    }
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
        std::unordered_map<std::string, std::shared_ptr<PublisherMetaData>> m_publishers;
        SERVER* m_connector;
    };
}
