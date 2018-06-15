#pragma once
#include <mrpt/web/CProcedure.h>
#include <mrpt/web/json_config.h>
#include <mrpt/web/CWebSocket.h>

#include <boost/asio.hpp>
namespace mrpt::web
{
using ConnectionPointer = std::shared_ptr<WsServer::Connection>;
class CProcedureInvokationBase
{
public:
    virtual void handleMethodCall(CProcedure& _proc, Json::Value const& _input, Json::Value& _output) = 0;
    virtual void handlePushMethodCall(CProcedure& _proc, Json::Value const& _input, Json::Value& _output, ConnectionPointer _conn) = 0;
    virtual ~CProcedureInvokationBase() {}
};
} //namespace mrpt::web
