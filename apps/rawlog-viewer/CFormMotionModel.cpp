#include "CFormMotionModel.h"

#include <mrpt/serialization/CArchive.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/obs/CSensoryFrame.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::serialization;
using namespace std;

// The new options:
CFormMotionModel::CFormMotionModel()
{

}

CFormMotionModel::~CFormMotionModel()
{

}

void CFormMotionModel::loadFromGaussian(const Json::Value& in)
{
  // Gaussian selected:
  options.modelSelection = CActionRobotMovement2D::mmGaussian;
  options.gaussianModel.a1 = (in["edG_A1"].asFloat());
  options.gaussianModel.a2 = (in["edG_A2"].asFloat()) * 180 / M_PIf;
  options.gaussianModel.a3 = (in["edG_A3"].asFloat()) * M_PIf / 180;
  options.gaussianModel.a4 = (in["edG_A4"].asFloat());
  options.gaussianModel.minStdXY = (in["edMinStdXY"].asFloat());
  options.gaussianModel.minStdPHI =
    DEG2RAD((in["edMinStdPHI"].asFloat()));
}

void CFormMotionModel::loadFromThrun(const Json::Value& in)
{
  // Thrun selected:
  options.modelSelection = CActionRobotMovement2D::mmThrun;
  options.thrunModel.nParticlesCount = (in["edNumParts"].asInt());
  options.thrunModel.alfa1_rot_rot = (in["edA1"].asFloat());
  options.thrunModel.alfa2_rot_trans =
    DEG2RAD((in["edA2"].asFloat()));
  options.thrunModel.alfa3_trans_trans = (in["edA3"].asFloat());
  options.thrunModel.alfa4_trans_rot = RAD2DEG((in["edA4"].asFloat()));
  options.thrunModel.additional_std_XY = (in["edAddXY"].asFloat());
  options.thrunModel.additional_std_phi =
    DEG2RAD((in["edAddPhi"].asFloat()));
}

void CFormMotionModel::showOptionsInDialog(Json::Value& opt)
{
  char str[1000];
  if (CActionRobotMovement2D::mmGaussian == options.modelSelection)
  {
    opt["edG_A1"] = options.gaussianModel.a1;
    opt["edG_A2"] = options.gaussianModel.a2 * (M_PIf / 180);
    opt["edG_A3"] = options.gaussianModel.a3 * (180 / M_PIf);
    opt["edG_A4"] = options.gaussianModel.a4;
    opt["edMinStdXY"] = options.gaussianModel.minStdXY;
    opt["edMinStdPHI"] = options.gaussianModel.minStdPHI;
  }
  else
  {
    opt["edA1"] = options.thrunModel.alfa1_rot_rot;
    opt["edA2"] = options.thrunModel.alfa2_rot_trans * (180 / M_PIf);
    opt["edA3"] = options.thrunModel.alfa3_trans_trans;
    opt["edA4"] = options.thrunModel.alfa4_trans_rot * (M_PIf / 180);
    opt["edAddXY"] = options.thrunModel.additional_std_XY;
    opt["edAddPhi"] = options.thrunModel.additional_std_phi;
    opt["edNumParts"] = options.thrunModel.nParticlesCount;
  }
}
void CFormMotionModel::applyToLoadedRawlog(CRawlog& rawlog, size_t first, size_t last)
{
  // Apply changes:
  size_t changes = 0;
  {
    for (size_t i = first; i <= last; i++)
    {
      last = min(last, rawlog.size() - 1);
      // Check type:
      if (rawlog.getType(i) == CRawlog::etActionCollection)
      {
        // This is an action:
        CActionCollection::Ptr acts = rawlog.getAsAction(i);

        CActionRobotMovement2D::Ptr firstActionMov =
          acts->getActionByClass<CActionRobotMovement2D>();

        if (firstActionMov)
        {
          if (firstActionMov->estimationMethod ==
            CActionRobotMovement2D::emOdometry)
          {
            // Use the kinematics motion model to estimate a pose
            // change gaussian approximation:
            firstActionMov->computeFromOdometry(
              firstActionMov->rawOdometryIncrementReading,
              options);
          }
          else
          {
            // Take the mean value
            firstActionMov->computeFromOdometry(
              firstActionMov->poseChange->getMeanVal(), options);
          }
          changes++;
        }
      } // end is action
    } // end for i
  }
}

void CFormMotionModel::drawRandomSamples(float _x, float _y, float _phi, Json::Value& ret)
{
  // Plot the 2D pose samples:
  unsigned int N = 1000;
  vector<float> xs(N), ys(N), ps(N), dumm(N, 0.1f);

  CActionRobotMovement2D act;
  CPose2D odo(_x, _y, DEG2RAD(_phi));

  // Load in the action:
  act.computeFromOdometry(odo, options);

  // Draw a set of random (x, y, phi) samples :
  // poseChange->draw drawManySamples( N, samples );

  // Pass to vectors and draw them:
  CPose2D tmpPose;
  for (unsigned int i = 0; i < N; i++)
  {
    act.drawSingleSample(tmpPose);
    xs[i] = tmpPose.x();
    ys[i] = tmpPose.y();
    ps[i] = RAD2DEG(tmpPose.phi());
  }
  /** serialize the data to send for plotting
   *
   * Code here
   *
   */

}
