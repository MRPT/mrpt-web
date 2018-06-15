#pragma once
#include <mrpt/web/CWebSocket.h>
#include <mrpt/web/CPubSubManagerBase.h>
#include <mrpt/web/json_config.h>

#include <iostream>
#include <thread>
#include <atomic>
#include <memory>

namespace mrpt::web
{   
using ConnectionPointer = std::shared_ptr<WsServer::Connection>;
/** template class for callback of certain MRPT data structures */
template <typename MESSAGE_TYPE>
class CallBackSub
{
private:
    virtual Json::Value serialize(MESSAGE_TYPE& msg)
    {
        /** Serialize standard MRPT data structures */
        Json::Value val;
        CSchemeArchive output(val);
        output = msg;
        return val;
    }
public:
    CallBackSub(std::string topic_, CPubSubManagerBase* base_ = nullptr) : m_topic(topic_), m_base(base_)
    {
        m_builder.settings_["indentation"] = "";
    }
    void operator()(MESSAGE_TYPE& msg)
    {
        if(m_base == nullptr) return;
        /** serialize the message
         * serialized message is stored in output_json
         */
        Json::Value output_json = serialize(msg);

        const std::string& output_str = Json::writeString(builder, output_json); 
        for(auto &connection : m_base->getSubscribedConnections(m_topic))
        {
            /** check if the connection is still live */
            if(!(m_base->checkConnectionLive(connection))) continue;
            /** if still live, send the message
             * to connection asynchronously
             */
            auto send_stream = std::make_shared<WsServer::SendStream>();
            *send_stream << output_str;

            /** connection->send is an asynchronous function */
            connection->send(send_stream, [](const mrpt::web::error_code &ec){
                if(ec) {
                    std::cout << "Server: Error sending message. " <<
                    "Error" << ec << ", error_message: " << ec.message() << std::endl;
                }
            });
        }
    }
    inline void setBase(CPubSubManagerBase* base_) { m_base = base_; }
private:
    std::string m_topic;
    CPubSubManagerBase* m_base;
    static Json::StreamWriterBuilder m_builder;
};
}