#pragma once
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CRawlog.h>

#include <mrpt/web/json_config.h>
class CFormMotionModel
{
  public:
  CFormMotionModel();
  virtual ~CFormMotionModel();

  void loadFromGaussian(const Json::Value& in);
  void loadFromThrun(const Json::Value& in);

  void showOptionsInDialog(Json::Value& opt);
  void applyToLoadedRawlog(mrpt::obs::CRawlog& rawlog, size_t first, size_t last);

  void drawRandomSamples(float _x, float _y, float _phi, Json::Value& ret);
  private:
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions options;
};
