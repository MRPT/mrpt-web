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
    this->bindAndAddMethod(mrpt::web::CProcedure("LoadMotionModel", procedure::METHOD), &StubsAbstract::LoadMotionModelI);
    this->bindAndAddMethod(mrpt::web::CProcedure("GetMapAndPath", procedure::METHOD), &StubsAbstract::GetMapAndPathI);
    this->bindAndAddMethod(mrpt::web::CProcedure("GetMapFromRTK", procedure::METHOD), &StubsAbstract::GetMapFromRTKI);
    this->bindAndAddMethod(mrpt::web::CProcedure("GetRandomPaths", procedure::METHOD), &StubsAbstract::GetRandomPathsI);
    this->bindAndAddMethod(mrpt::web::CProcedure("DrawRandomSamples", procedure::METHOD), &StubsAbstract::DrawRandomSamplesI);
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

  inline virtual void LoadMotionModelI(const Json::Value &request, Json::Value &response)
  {
    response = this->LoadMotionModel(request);
  }

  inline virtual void GetMapAndPathI(const Json::Value &request, Json::Value &response)
  {
    response = this->GetMapAndPath(request);
  }

  inline virtual void GetMapFromRTKI(const Json::Value &request, Json::Value &response)
  {
    response = this->GetMapFromRTK(request);
  }

  inline virtual void GetRandomPathsI(const Json::Value &request, Json::Value &response)
  {
    response = this->GetRandomPaths(request);
  }

  inline virtual void DrawRandomSamplesI(const Json::Value &request, Json::Value &response)
  {
    response = this->DrawRandomSamples(request);
  }

  virtual Json::Value LoadRawlog(const Json::Value& request) = 0;
  virtual Json::Value GetRawlogTree(const Json::Value& request) = 0;
  virtual Json::Value GetRawlogDataFromIndex(const Json::Value& request) = 0;
  virtual Json::Value LoadMotionModel(const Json::Value& request) = 0;
  virtual Json::Value GetMapAndPath(const Json::Value &request) = 0;
  virtual Json::Value GetMapFromRTK(const Json::Value &request) = 0;
  virtual Json::Value GetRandomPaths(const Json::Value &request) = 0;
  virtual Json::Value DrawRandomSamples(const Json::Value &request) = 0;
};
}
