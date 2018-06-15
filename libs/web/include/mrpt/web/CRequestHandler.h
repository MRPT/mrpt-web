#pragma once
#include <mrpt/web/CProcedureInvokationBase.h>
namespace mrpt::web
{
class CRequestHandler
{
    const std::string OP_FIELD_ABSENT = "OP_FIELD_ABSENT";
    const std::string OP_ABSENT = "OP_ABSENT";
public:
    CRequestHandler(CProcedureInvokationBase& base): m_outter(base) {}
    void handleRequest(const std::string &request, std::string &response, ConnectionPointer _conn)
    {
        /** The procedure name is stored in
         *  proc_name = req["op"]
         *
        */
        Json::Value req, _input, _output, res;
        //Parse request to Json::Value req and extracting op
        std::stringstream ss(request);
        ss >> req;
        const std::string proc_name = req.get("op",OP_FIELD_ABSENT).asString();
        //Prepare base result Json::Value res with the id and version
        res["id"] = req["id"];  //Add checks for presence of id in req
        res["v"] = req["v"]; 
        if(proc_name == OP_FIELD_ABSENT)
        {
            //Return some error code
            return;
        }
        if(m_procedures.find(proc_name) == m_procedures.end())
        {
            //Return some other error code
            return;
        }
        auto _proc = m_procedures[proc_name];
        _input = req["params"];
        if(_proc.getProcedureType() == procedure::METHOD)
        {
            m_outter.handleMethodCall(_proc, _input, _output);
        }
        else if(_proc.getProcedureType() == procedure::PUSHMETHOD)
        {
            m_outter.handlePushMethodCall(_proc, _input, _output, _conn);
        }
        // Add the output to Json::Object res
        res["result"] = _output;

        //parse the Json::Object to string response
        Json::StreamWriterBuilder builder;
        builder.settings_["indentation"] = "";
        response = Json::writeString(builder, res);

    }
    void AddProcedure(const CProcedure& _proc)
    {
        m_procedures[_proc.GetProcedureName()] = _proc;
    } 
private:
    CProcedureInvokationBase& m_outter;
    std::unordered_map<std::string, CProcedure> m_procedures;
};
} // namespace mrpt::web
