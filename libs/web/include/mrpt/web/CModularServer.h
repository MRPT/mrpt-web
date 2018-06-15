#pragma once
#include <mrpt/web/CAbstractServerConnector.h>
#include <mrpt/web/CWebSocket.h>
#include <mrpt/web/CProcedure.h>
#include <mrpt/web/CRequestHandler.h>
#include <mrpt/web/json_config.h>

#include <unordered_map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace mrpt::web
{
    using WsServer = SocketServer<WS>;
    using ConnectionPointer = std::shared_ptr<WsServer::Connection>;
    template <class I> using AbstractMethodPointer = void(I::*)(Json::Value const& _parameter, Json::Value& _result);
    template <class I> using AbstractPushMethodPointer = void(I::*)(Json::Value const& _parameter, Json::Value& _result, ConnectionPointer _conn);
    template<class I>
    class ServerInterface
    {
        public:
        using MethodPointer = AbstractMethodPointer<I>;
        using PushMethodPointer = AbstractPushMethodPointer<I>;
        
        using MethodBinding = std::tuple<CProcedure, AbstractMethodPointer<I>>;
        using PushMethodBinding = std::tuple<CProcedure, AbstractPushMethodPointer<I>>;
        using Methods = std::vector<MethodBinding>;
        using PushMethods = std::vector<PushMethodBinding>;

        virtual ~ServerInterface() {}
        Methods const& methods() const { return m_methods; }
        PushMethods const& pushmethods() const { return m_pushmethods; }
        protected:
        void bindAndAddMethod(CProcedure const& _proc, MethodPointer _pointer) {m_methods.emplace_back(_proc, _pointer);}
        void bindAndAddPushMethod(CProcedure const& _proc, PushMethodPointer _pointer) { m_pushmethods.emplace_back(_proc, _pointer);}
        private:
        Methods m_methods;
        PushMethods m_pushmethods;
    };


    template <class... Is>
    class CModularServer: public CProcedureInvokationBase
    {
    public:
        CModularServer()
        : m_handler(new CRequestHandler(*this))
        {
            //May be add some standard procedures
        }
        
        virtual ~CModularServer() { StopListening(); }

        virtual void StartListening()
        {
            for(auto const& connector: m_connectors)
                connector->StartListening();
        }
        virtual void StopListening()
        {
            for(auto const& connector: m_connectors)
                connector->StopListening();
        }
        unsigned addConnector(CAbstractServerConnector* _connector)
        {
            m_connectors.emplace_back(_connector);
            _connector->SetHandler(m_handler.get());
            return m_connectors.size() - 1;
        }

        CAbstractServerConnector* connector(unsigned _i) const
        {
            return m_connectors.at(_i).get();
        }
    protected:
        std::vector<std::unique_ptr<CAbstractServerConnector>> m_connectors;
        std::unique_ptr<CRequestHandler> m_handler;
    };

    template <class I, class... Is>
    class CModularServer<I, Is...> : public CModularServer<Is...>
    {
    public:
        using MethodPointer = AbstractMethodPointer<I>;
        using PushMethodPointer = AbstractPushMethodPointer<I>;
        CModularServer<I, Is...>(I* _i, Is*... _is):CModularServer<Is...>(_is...), m_interface(_i)
        {
            if(!m_interface)
                return;
            for (auto const& method: m_interface->methods())
            {
                m_methods[std::get<0>(method).GetProcedureName()] = std::get<1>(method);
                this->m_handler->AddProcedure(std::get<0>(method));
            }

            for(auto const& pushmethod: m_interface->pushmethods())
            {
                m_pushmethods[std::get<0>(pushmethod).GetProcedureName()] = std::get<1>(pushmethod);
                this->m_handler->AddProcedure(std::get<0>(pushmethod));
            }
        }

        virtual void handleMethodCall(CProcedure& _proc, Json::Value const& _input, Json::Value& _output) override
        {
            auto pointer = m_methods.find(_proc.GetProcedureName());
            if(pointer != m_methods.end())
            {
                try
                {
                    (m_interface.get()->*(pointer->second))(_input, _output);
                }
                catch(Json::Exception const& ex)
                {
                    throw std::logic_error(ex.what());
                }
            }
        }

        virtual void handlePushMethodCall(CProcedure& _proc, Json::Value const& _input, Json::Value& _output, ConnectionPointer _conn) override
        {
            auto pointer = m_pushmethods.find(_proc.GetProcedureName());
            if(pointer != m_pushmethods.end())
            {
                try
                {
                    (m_interface.get()->*(pointer->second))(_input, _output, _conn);
                }
                catch(Json::Exception const& ex)
                {
                    throw std::logic_error(ex.what());
                }
            }
        }
    private:
        std::unique_ptr<I> m_interface;
        std::map<std::string, MethodPointer> m_methods;
        std::map<std::string, PushMethodPointer> m_pushmethods;
    };
}
