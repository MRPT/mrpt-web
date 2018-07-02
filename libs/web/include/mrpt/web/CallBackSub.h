#pragma once
#include <mrpt/web/CWebSocket.h>
#include <mrpt/web/CPubSubManagerBase.h>
#include <mrpt/web/json_config.h>
#include <mrpt/web/CSchemeArchive.h>

#include <mrpt/serialization/CSchemeArchiveBase.h>

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
  #define VERSION_CODE "1.0"
  #define OP_CODE "publish"
private:
    virtual Json::Value serialize(MESSAGE_TYPE& msg)
    {
        /** Serialize standard MRPT data structures */
        Json::Value val;
        mrpt::serialization::CSchemeArchiveBase output(std::make_unique<CSchemeArchive<Json::Value>>(val));
        output = msg;
        return val;
    }
public:
    CallBackSub(std::string topic_, CPubSubManagerBase* base_ = nullptr) : m_topic(topic_), m_base(base_) {}
    void operator()(MESSAGE_TYPE& msg)
    {
        if(m_base == nullptr) return;
        /** serialize the message
         * serialized message is stored in output_json
         */
        Json::Value output_json;
        output_json["msg"] = serialize(msg);
        output_json["v"] = VERSION_CODE;
        output_json["op"] = OP_CODE;
        output_json["topic"] = m_topic;
        Json::StreamWriterBuilder m_builder;
        m_builder.settings_["indentation"] = "";

        const std::string output_str = Json::writeString(m_builder, output_json);
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
};
}
