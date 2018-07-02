#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3DQuat.h>

#include <mrpt/web/CModularServer.h>
#include <mrpt/web/CSchemeArchive.h>
#include <mrpt/web/CallBackSub.h>
#include <mrpt/web/json_config.h>

#include <mrpt/serialization/CSchemeArchiveBase.h>

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
    const std::string topic = request.get("topic","").asString();
    const std::string type = request.get("type","").asString();
    const int throttle_rate = request.get("throttle_rate", 0).asInt();
    const int queue_length = request.get("queue_length", 0).asInt();
    m_manager->addSubscriptionToTopic(topic, _conn);
    ret["success"] = true;
    return ret;
  }
  Json::Value Subscriber_unsubscribe(const Json::Value& request, ConnectionPointer _conn) override
  {
    Json::Value ret;
    const std::string topic = request.get("topic","").asString();
    m_manager->removeSubscriptionFromTopic(topic, _conn);
    ret["success"] = true;
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
  Json::Value val;
  CSchemeArchiveBase out(std::make_unique<CSchemeArchive<Json::Value>>(val));
  CPoint2D pt;
  out = pt;
  std::cout << val << std::endl;
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
    //add dummy publisher on topic '/path' for testing

    std::thread pub_thread1([&](){
      std::string topic = "/path";
      CallBackSub<CPoint2D> point_sub(topic, manager.get() );
      while(1) {
      CPoint2D pt;
      point_sub(pt);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }
    });

    std::thread pub_thread2([&](){
      std::string topic = "/pose";
      CallBackSub<CPose3DQuat> pose_sub(topic, manager.get() );
      while(1) {
        CPose3DQuat ps;
        pose_sub(ps);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      }
    });
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
