#pragma once
#include <mrpt/web/CModularServer.h>
#include <mrpt/web/CProcedure.h>

namespace mrpt::web
{
class CRPCPubSubAbstract : public ServerInterface<CRPCPubSubAbstract>
{
    public:
        CRPCPubSubAbstract()
        {
            this->bindAndAddPushMethod(mrpt::web::CProcedure("Publisher.advertise", procedure::PUSHMETHOD), &CRPCPubSubAbstract::Publisher_AdvertiseI);
            this->bindAndAddMethod(mrpt::web::CProcedure("Publish", procedure::METHOD), &CRPCPubSubAbstract::PublishI);
            this->bindAndAddPushMethod(mrpt::web::CProcedure("Publisher.unadvertise",procedure::PUSHMETHOD), &CRPCPubSubAbstract::Publisher_UnadvertiseI);
            this->bindAndAddPushMethod(mrpt::web::CProcedure("Subscriber.subscribe", procedure::PUSHMETHOD), &CRPCPubSubAbstract::Subscriber_subscribeI);
            this->bindAndAddPushMethod(mrpt::web::CProcedure("Subscriber.unsubscribe", procedure::PUSHMETHOD), &CRPCPubSubAbstract::Subscriber_unsubscribeI);
        }

        inline virtual void Publisher_AdvertiseI(const Json::Value &request, Json::Value &response, mrpt::web::ConnectionPointer _conn)
        {
            response = this->Publisher_Advertise(request, _conn);
        }
        inline virtual void PublishI(const Json::Value &request, Json::Value &response)
        {
            response = this->Publish(request);
        }
        inline virtual void Publisher_UnadvertiseI(const Json::Value &request, Json::Value &response,  mrpt::web::ConnectionPointer _conn)
        {
            response = this->Publisher_Unadvertise(request, _conn);
        }
        inline virtual void Subscriber_subscribeI(const Json::Value &request, Json::Value &response,  mrpt::web::ConnectionPointer _conn)
        {
            response = this->Subscriber_subscribe(request, _conn);
        }
        inline virtual void Subscriber_unsubscribeI(const Json::Value &request, Json::Value &response,  mrpt::web::ConnectionPointer _conn)
        {
            response = this->Subscriber_unsubscribe(request, _conn);
        }
        virtual Json::Value Publisher_Advertise(const Json::Value& request, mrpt::web::ConnectionPointer _conn) = 0;
        virtual Json::Value Publish(const Json::Value& request) = 0;
        virtual Json::Value Publisher_Unadvertise(const Json::Value& request, mrpt::web::ConnectionPointer _conn) = 0;
        virtual Json::Value Subscriber_subscribe(const Json::Value& request, mrpt::web::ConnectionPointer _conn) = 0;
        virtual Json::Value Subscriber_unsubscribe(const Json::Value& request, mrpt::web::ConnectionPointer _conn) = 0;
};
}