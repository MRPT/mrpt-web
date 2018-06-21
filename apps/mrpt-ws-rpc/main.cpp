#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3DQuat.h>

#include <mrpt/web/CModularServer.h>
#include <mrpt/web/CSchemeArchive.h>

#include "CRPCPubSubFace.h"
#include "CPubSubManager.h"
#include "CWebSocketServer.h"

#include <cstdlib>
#include <functional>
#include <thread>
#include <string>
#include <mutex>
#include <memory>
#include <iostream>

using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::web;
using namespace mrpt::serialization;

class CRPCPubSub : public CRPCPubSubAbstract
{
public:
  CRPCPubSub(std::shared_ptr<CPubSubManagerBase> manager_) : m_manager(manager_) {}
  Json::Value Publisher_Advertise(const Json::Value& request, ConnectionPointer _conn) override
  {
    Json::Value ret;
    const std::string topic = request.get("topic", "").asString();
    const std::string type = request.get("type", "").asString();
    const bool latch = request.get("latch",false).asBool();
    const int queue_size = request.get("queue_size",100).asInt();
    m_manager->addPublisherToTopic(topic,type);
    ret["success"] = true;
    return ret;
  }
  Json::Value Publish(const Json::Value& request) override
  {
    Json::Value ret;
    const std::string topic = request.get("topic","").asString();
    const auto msg = request["message"];
    m_manager->publishMessageToTopic(topic, msg);
    ret["success"] = true;
    return ret;
  }
  Json::Value Publisher_Unadvertise(const Json::Value& request, ConnectionPointer _conn) override
  {
    Json::Value ret;
    const std::string topic = request.get("topic", "").asString();
    m_manager->removePublisherFromTopic(topic);
    ret["success"] = true;
    return ret;
  }
  Json::Value Subscriber_subscribe(const Json::Value& request, ConnectionPointer _conn) override
  {
    Json::Value ret;

    return ret;
  }
  Json::Value Subscriber_unsubscribe(const Json::Value& request, ConnectionPointer _conn) override
  {
    Json::Value ret;

    return ret;
  }
  Json::Value add_three_ints(const Json::Value& request) override
  {
    Json::Value req = request;
    Json::Value ret;
    int a = req.get("a",0).asInt();
    int b = req.get("b",0).asInt();
    int c = req.get("c",0).asInt();
    ret["sum"] = (int)(a+b+c);
    return ret;
  }
private:
  std::shared_ptr<CPubSubManagerBase> m_manager;
};
int main(int argc,char* argv[])
{
  //Check the command line arguments,
  if (argc != 3)
  {
      std::cerr <<
          "Usage: mrpt-ws-rpc <address> <port>\n" <<
          "Example:\n" <<
          "    mrpt-ws-rpc 127.0.0.1 8080\n";
      return EXIT_FAILURE;
  }
  auto const address = boost::asio::ip::make_address(argv[1]);
  auto const port = static_cast<unsigned short>(std::atoi(argv[2]));
  try{
    std::unique_ptr<CModularServer<>> jsonrpcIpcServer;
    using FullServer = CModularServer< CRPCPubSub>;
    //Server without ConnectionStore and SubscriberManager
    // auto server = new CWebSocketServer(port);
    //Server with ConnectionStore and SubscriberManager
    auto server = new CWebSocketServer(port);

    // Managers for Publishers and Subscibers
    auto manager = std::make_shared<CPubSubManager<CWebSocketServer>>(server);

    jsonrpcIpcServer.reset(new FullServer(
      new CRPCPubSub(manager)   // RPC stub for PubSub activity
    ));

    jsonrpcIpcServer->addConnector(server);

    // Server starts listening
    jsonrpcIpcServer->StartListening();
    getchar();
    // Server stops listening to connections
    jsonrpcIpcServer->StopListening();
  }
  catch(const std::exception& e)
  {
    std::cerr <<"Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
