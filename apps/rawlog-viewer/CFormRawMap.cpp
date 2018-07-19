#include "CFormRawMap.h"
// General global variables:
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/math/geometry.h>
#include <mrpt/topography.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::config;
using namespace mrpt::math;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::poses;
using namespace std;

string config_str = "; ====================================================\n; "
		  "MULTIMETRIC MAP CONFIGURATION\n; "
		  "====================================================\n[map]\n; "
		  "Creation of maps:\noccupancyGrid_count = 0\ngasGrid_count = "
		  "0\nlandmarksMap_count = 0\nbeaconMap_count = 0\npointsMap_count = "
		  "1\nheightMap_count = 0\ncolourPointsMap_count=0\n\n; "
		  "====================================================\n; MULTIMETRIC "
		  "MAP: PointsMap #00\n; "
		  "====================================================\n; Insertion "
		  "Options for PointsMap "
		  "00:\n[map_pointsMap_00_insertOpts]\nminDistBetweenLaserPoints = "
		  "0.05\nisPlanarMap = 0\nalso_interpolate = 0\n\n; "
		  "====================================================\n; MULTIMETRIC "
		  "MAP: HeightMap #00\n; "
		  "====================================================\n; Creation "
		  "Options for HeightMap "
		  "00:\n[map_heightGrid_00_creationOpts]\nmapType = 0 \t\t; See "
		  "CHeightGridMap2D::CHeightGridMap2D\nmin_x = -10\nmax_x = 10\nmin_y "
		  "= -10\nmax_y = 10\nresolution = 0.10\n\n; "
		  "====================================================\n; MULTIMETRIC "
		  "MAP: HeightMap #00\n; "
		  "====================================================\n; Insertion "
		  "Options for HeightMap "
		  "00:\n[map_heightGrid_00_insertOpts]\nfilterByHeight = 0 ; 0/1: Do "
		  "not/do filter.\nz_min =-0.10\nz_max = 0.10\n\n; "
		  "====================================================\n; MULTIMETRIC "
		  "MAP: ColourPointsMap #00\n; "
		  "====================================================\n; Insertion "
		  "Options for ColourPointsMap "
		  "00:\n[map_colourPointsMap_00_insertOpts]\nminDistBetweenLaserPoints "
		  "= 0.05\nisPlanarMap = 0\nalso_interpolate = 0\n\n; Additional "
		  "options for use in RTK GPS-based map "
		  "building\n[RTK_MAP]\ndisableGPSInterp = 0 // If set to 1, disable "
		  "GPS interpolation for sequences of JAVAD readings\nsmooth_filter = "
		  "3 // Smooth pitch & roll angles by averaging each value with a "
		  "window of N previous and next elements (=0: Disable)\n";
CFormRawMap::CFormRawMap(mrpt::obs::CRawlog& rawlog): m_rawlog(rawlog) {}

CFormRawMap::~CFormRawMap() {}

void CFormRawMap::OnbtnGenerateClick(size_t first_idx_, size_t last_idx_, size_t decimate_, Json::Value& ret) {
  // Go generate the map:
  size_t i;
  CPose2D curPose(0, 0, 0);

  size_t first  = first_idx_;
  size_t last = min(last_idx_, m_rawlog.size() - 1);
  size_t decimate = decimate_;
  std::cout << first_idx_ << last_idx_ << m_rawlog.size() - 1 <<std::endl;
  // Create a memory "ini file" with the text in window
  CConfigFileMemory configSrc(config_str);
  TSetOfMetricMapInitializers lstMaps;
	lstMaps.loadFromConfigFile(configSrc, "map"); // Error here
  theMap.setListOfMaps(&lstMaps);

  CPointsMap::Ptr thePntsMap;
  if (!theMap.m_pointsMaps.empty())
  {
    CSimplePointsMap::Ptr sMap;
    sMap = theMap.m_pointsMaps[0];
    thePntsMap = std::dynamic_pointer_cast<CPointsMap>(sMap);
  }
  else if (theMap.m_colourPointsMap)
  {
    CColouredPointsMap::Ptr colorMap;
    colorMap = theMap.m_colourPointsMap;
    thePntsMap = std::dynamic_pointer_cast<CPointsMap>(colorMap);
  }

  size_t count = 0;
  vector<float> pathX, pathY;
  bool abort = false;

  robot_path.clear();

  // An (approx) estimate of the final size of the map (great improve in
  // speed!)
  if (thePntsMap) thePntsMap->reserve((last - first + 1) * 800);

  TTimeStamp last_tim = INVALID_TIMESTAMP;

	for (i = first; !abort && i <= last; i++)
	{
		bool addNewPathEntry = false;

		switch (m_rawlog.getType(i))
		{
			case CRawlog::etActionCollection:
			{
				CActionCollection::Ptr acts = m_rawlog.getAsAction(i);
				CPose2D poseIncrement;
				bool poseIncrementLoaded = false;

				for (size_t j = 0; j < acts->size(); j++)
				{
					CAction::Ptr act = acts->get(j);
					if (act->GetRuntimeClass() ==
						CLASS_ID(CActionRobotMovement2D))
					{
						CActionRobotMovement2D::Ptr mov =
							std::dynamic_pointer_cast<CActionRobotMovement2D>(
								act);

						// Load an odometry estimation, but only if it is the
						// only movement
						//  estimation source: any other may be a better one:
						if (!poseIncrementLoaded ||
							mov->estimationMethod !=
								CActionRobotMovement2D::emOdometry)
						{
							poseIncrementLoaded = true;
							mov->poseChange->getMean(poseIncrement);
						}
					}
				}

				if (!poseIncrementLoaded && i < last)
					THROW_EXCEPTION_FMT(
						"ERROR: Odometry not found at step %d!", (int)i);

				curPose = curPose + poseIncrement;
				addNewPathEntry = true;
			}
			break;
			case CRawlog::etSensoryFrame:
			{
				if (((i >> 1) % decimate) == 0)
				{
					CPose3D dumPose(curPose);
					m_rawlog.getAsObservations(i)->insertObservationsInto(
						&theMap, &dumPose);
				}
				addNewPathEntry = true;
			}
			break;
			case CRawlog::etObservation:
			{
				// Always, process odometry:
				const CObservation* obs = m_rawlog.getAsObservation(i).get();
				if (IS_CLASS(obs, CObservationOdometry))
				{
					const CObservationOdometry* obsOdo =
						static_cast<const CObservationOdometry*>(obs);
					curPose = obsOdo->odometry;
				}

				if (((i >> 1) % decimate) == 0)
				{
					CPose3D dumPose(curPose);
					theMap.insertObservation(
						m_rawlog.getAsObservation(i).get(), &dumPose);
					last_tim = m_rawlog.getAsObservation(i)->timestamp;
				}
				addNewPathEntry = true;
			}
			break;
			default:
				break;
		};  // end switch

		if (addNewPathEntry)
		{
			pathX.push_back(curPose.x());
			pathY.push_back(curPose.y());
			if (last_tim != INVALID_TIMESTAMP)
				robot_path.insert(last_tim, CPose3D(curPose));
		}

		// if ((count++ % 50) == 0)
		// {
		// 	if (!progDia.Update((int)(i - first))) abort = true;
		// 	wxTheApp->Yield();
		// }
	}  // end for i
  Json::Value lyPoints, lyPath;
  // Load into the graphs:
  // ----------------------------
  /**
   * Add the path to return serialized return message
   * store in vector pathX , pathY
   */
  for (uint32_t k = 0; k < pathX.size(); k++)
  {
    lyPath["x"][k] = pathX[k];
  }

  for (uint32_t k = 0; k < pathY.size(); k++)
  {
    lyPath["y"][k] = pathY[k];
  }
  ret["path"] = lyPath;
  if (thePntsMap)
  {
    size_t nPts = thePntsMap->size();
    size_t decimation = 1;

    if (nPts > 100000)
    {
      decimation = nPts / 100000;
    }

    vector<float> Xs, Ys;
    thePntsMap->getAllPoints(Xs, Ys, decimation);
    // Set the points data Xs, Ys to lyPoints
    for (uint32_t k = 0; k < Xs.size(); k++)
    {
      lyPoints["xs"][k] = Xs[k];
    }
    for (uint32_t k = 0; k < Ys.size(); k++)
    {
      lyPoints["ys"][k] = Ys[k];
    }
    // if (decimation > 1)
    //     lbCount->SetLabel(wxString::Format(
    //     _("Point count=%u\n(Decimation=%u)"), unsigned(Xs.size()),
    //     unsigned(decimation)));
    // else
    //   lbCount->SetLabel(wxString::Format(
    //   _("Point count=%u\n(No decimation)"), unsigned(Xs.size())));
    ret["points"] = lyPoints;
  }
  ret["textData"] = config_str;
}

void CFormRawMap::OnbtnGeneratePathsClick(size_t first_idx_, size_t last_idx_, size_t decimate_, Json::Value& ret) {
  ret["err"] = "Not implemented yet";
}

void CFormRawMap::OnGenerateFromRTK(size_t first_idx_, size_t last_idx_, size_t decimate_, Json::Value& ret) {
  // Not implemented yet
  ret["err"] = "Not implemented yet";
}
