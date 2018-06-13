#pragma once
#include <string>

namespace mrpt::web
{
    enum class procedure{METHOD, PUSHMETHOD};
    class CProcedure
    {
    public:
        CProcedure() {}
        CProcedure(std::string name, procedure type): procedureName(name), procedureType(type) {}
        const std::string& GetProcedureName() const { return procedureName; }
        procedure GetProcedureType() const { return procedureType; }
    private:
        std::string procedureName;
        procedure procedureType;    
    };
}
