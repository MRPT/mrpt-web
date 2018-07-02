#pragma once
#include <mrpt/web/CModularServer.h>
#include <mrpt/web/CProcedure.h>

namespace mrpt::web
{
class StubsAbstract : public ServerInterface<StubsAbstract>
{
  public:
  StubsAbstract()
  {
    this->bindAndAddMethod(mrpt::web::CProcedure("LoadRawlog", procedure::METHOD), &StubsAbstract::LoadRawlogI);
    this->bindAndAddMethod(mrpt::web::CProcedure("GetRawlogTree", procedure::METHOD), &StubsAbstract::GetRawlogTreeI);
    this->bindAndAddMethod(mrpt::web::CProcedure("GetRawlogDataFromIndex", procedure::METHOD), &StubsAbstract::GetRawlogDataFromIndexI);
  }

  inline virtual void LoadRawlogI(const Json::Value &request, Json::Value &response)
  {
    response = this->LoadRawlog(request);
  }

  inline virtual void GetRawlogTreeI(const Json::Value &request, Json::Value &response)
  {
    response = this->GetRawlogTree(request);
  }

  inline virtual void GetRawlogDataFromIndexI(const Json::Value &request, Json::Value &response)
  {
    response = this->GetRawlogDataFromIndex(request);
  }

  virtual Json::Value LoadRawlog(const Json::Value& request) = 0;
  virtual Json::Value GetRawlogTree(const Json::Value& request) = 0;
  virtual Json::Value GetRawlogDataFromIndex(const Json::Value& request) = 0;
};
}
