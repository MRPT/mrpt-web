#pragma once

#include <memory>
#include <string>

#include <mrpt/obs/CRawlog.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/topography.h>
#include <mrpt/poses/CPosePDFParticles.h>

#include <mrpt/web/json_config.h>

class CFormRawMap
{
  public:
    CFormRawMap(mrpt::obs::CRawlog& rawlog);
    virtual ~CFormRawMap();
  public:
    void OnbtnGenerateClick(size_t first_idx_, size_t last_idx_, size_t decimate_, Json::Value& ret);
    void OnbtnGeneratePathsClick(size_t first_idx_, size_t last_idx_, size_t decimate_, Json::Value& ret);
    void OnGenerateFromRTK(size_t first_idx_, size_t last_idx_, size_t decimate_, Json::Value& ret);
  private:
  // The rawlog to process
  mrpt::obs::CRawlog& m_rawlog;
  // The map built from laser & odometry
  mrpt::maps::CMultiMetricMap theMap;
  mrpt::poses::CPose3DInterpolator robot_path;
  mrpt::topography::TPathFromRTKInfo rtk_path_info;
};
