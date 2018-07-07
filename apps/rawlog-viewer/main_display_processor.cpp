#include "CRawlogTreeProcessor.h"

#include <mrpt/system/datetime.h>
#include <mrpt/math/ops_matrices.h>  // << ops
#include <mrpt/math/ops_vectors.h>  // << ops
#include <mrpt/math/wrap2pi.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CPointCloudColoured.h>

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/obs.h>

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/poses/CPosePDFParticles.h>

#include <iomanip>

#define THRESHOLD_CONFIDENCE_IMAGE 1 /** The actual value TBD */
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace mrpt::poses;
using namespace mrpt::rtti;

using namespace std;
using namespace  mrpt::webapps;
namespace mrpt::webapps
{
class ControlSignals
{
  public:
    ControlSignals(): m_tab_selection(0) {}
    void changeSelection(uint8_t x) { m_tab_selection = x; }
    Json::Value serializeTo() {
      Json::Value ret;
      ret["tab"] = m_tab_selection;
      return ret;
    }
  private:
    uint8_t m_tab_selection;
};
void mainDisplayProcessor(const mrpt::serialization::CSerializable::Ptr& sel_obj,	const mrpt::obs::CRawlog* rawlog, Json::Value& ret)
{
  /**
   * redirect console log output to the web app
  */
  std::stringstream ss;
  /** Control Signals like tab number, other contextual parameters */
  ControlSignals cs;
  if (sel_obj)
  {

    /**Get runtime class */
    const TRuntimeClassId* classID = sel_obj->GetRuntimeClass();
    // default tab is 0
    cs.changeSelection(0);
    // Common data:
    if (classID->derivedFrom(CLASS_ID(CObservation)))
    {
      CObservation::Ptr obs(
        std::dynamic_pointer_cast<CObservation>(sel_obj));
      obs->load();
      obs->getDescriptionAsText(ss);
      // Special cases:
      if (IS_CLASS(sel_obj, CObservation2DRangeScan))
      {
        CObservation2DRangeScan::Ptr obs_scan2d =
          std::dynamic_pointer_cast<CObservation2DRangeScan>(
            sel_obj);

        mrpt::maps::CSimplePointsMap pts;
        pts.insertionOptions.minDistBetweenLaserPoints = .0;

        pts.loadFromRangeScan(*obs_scan2d);

        ss << "2D coordinates of valid points (wrt to "
            "robot/vehicle frame, "
            << pts.size() << " points)\n";
        ss << "pts=[";
        const auto& xs = pts.getPointsBufferRef_x();
        const auto& ys = pts.getPointsBufferRef_y();
        for (size_t i = 0; i < xs.size(); i++)
          ss << format("%7.04f %7.04f;", xs[i], ys[i]);
        ss << "]\n\n";
      }

      // curSelectedObservation =
      //   std::dynamic_pointer_cast<CObservation>(sel_obj);
    }
    if (classID->derivedFrom(CLASS_ID(CAction)))
    {
      CAction::Ptr obs(std::dynamic_pointer_cast<CAction>(sel_obj));
      ss << "Timestamp (UTC): " << dateTimeToString(obs->timestamp)
          << endl;
    }

    // Specific data:
    if (classID == CLASS_ID(CObservation2DRangeScan))
    {
      // ----------------------------------------------------------------------
      //              CObservation2DRangeScan
      // ----------------------------------------------------------------------
      cs.changeSelection(2);
      CObservation2DRangeScan::Ptr obs =
        std::dynamic_pointer_cast<CObservation2DRangeScan>(sel_obj);
        // The plot:
        mrpt::maps::CSimplePointsMap dummMap;
				dummMap.insertionOptions.minDistBetweenLaserPoints = 0;
				dummMap.insertObservation(obs.get());

				vector<float> Xs, Ys;
				dummMap.getAllPoints(Xs, Ys);
        /** Serialize CSimplePointsMap to schema
         *
         * Code Here
         *
         */
    }

    if (classID == CLASS_ID(CObservationImage))
    {
      // ----------------------------------------------------------------------
      //              CObservationImage
      // ----------------------------------------------------------------------
      cs.changeSelection(3);
      CObservationImage::Ptr obs =
        std::dynamic_pointer_cast<CObservationImage>(sel_obj);

      // wxGUI code :// Get bitmap:
      // // ----------------------
      // wxImage* img = mrpt::gui::MRPTImage2wxImage(obs->image);
      // bmpObsImage->SetBitmap(wxBitmap(*img));
      // bmpObsImage->Refresh();
      // delete img;
      /** function to convert image to a schema serializable format
       * Add the image to Json::Value object ret
       *
       * Code Here
       */
      obs->image.unload();  // For externally-stored datasets
    }

    if (classID == CLASS_ID(CObservationStereoImages))
    {
      // ----------------------------------------------------------------------
      //              CObservationStereoImages
      // ----------------------------------------------------------------------
      cs.changeSelection(4);
      CObservationStereoImages::Ptr obs =
        std::dynamic_pointer_cast<CObservationStereoImages>(
          sel_obj);

      // wxGUI code // Images:
      // // ----------------------
      // wxImage* imgLeft = mrpt::gui::MRPTImage2wxImage(obs->imageLeft);
      // bmpObsStereoLeft->SetBitmap(wxBitmap(*imgLeft));
      // bmpObsStereoLeft->Refresh();
      // delete imgLeft;

      // wxImage* imgRight =
      //   mrpt::gui::MRPTImage2wxImage(obs->imageRight);
      // bmpObsStereoRight->SetBitmap(wxBitmap(*imgRight));
      // bmpObsStereoRight->Refresh();
      // delete imgRight;

      // wxImage* imgDisp =
      //   mrpt::gui::MRPTImage2wxImage(obs->imageDisparity);
      // bmpObsStereoDisp->SetBitmap(wxBitmap(*imgDisp));
      // bmpObsStereoDisp->Refresh();
      // delete imgDisp;
      /** function to convert all three images
       * to a schema serializable format
       * Add the image to Json::Value object ret
       *
       * Code Here
       */
    }

    if (classID == CLASS_ID(CActionRobotMovement2D))
    {
      // ----------------------------------------------------------------------
      //              CActionRobotMovement2D
      // ----------------------------------------------------------------------
      cs.changeSelection(1);

      CActionRobotMovement2D::Ptr act =
        std::dynamic_pointer_cast<CActionRobotMovement2D>(sel_obj);

      CPose2D Ap;
      CMatrixDouble33 mat;
      act->poseChange->getCovarianceAndMean(mat, Ap);

      ss << "Robot Movement (as a gaussian pose change):\n";
      ss << " Mean = " << Ap << endl;

      ss << format(" Covariance:     DET=%e\n", mat.det());

      ss << format(
        "      %e %e %e\n", mat(0, 0), mat(0, 1), mat(0, 2));
      ss << format(
        "      %e %e %e\n", mat(1, 0), mat(1, 1), mat(1, 2));
      ss << format(
        "      %e %e %e\n", mat(2, 0), mat(2, 1), mat(2, 2));

      ss << endl;

      ss << " Actual reading from the odometry increment = "
          << act->rawOdometryIncrementReading << endl;

      ss << format(
        "Actual PDF class is: '%s'\n",
        act->poseChange->GetRuntimeClass()->className);

      if (act->poseChange->GetRuntimeClass() ==
        CLASS_ID(CPosePDFParticles))
      {
        CPosePDFParticles::Ptr aux =
          std::dynamic_pointer_cast<CPosePDFParticles>(
            act->poseChange.get_ptr());
        ss << format(
          " (Particle count = %u)\n",
          (unsigned)aux->m_particles.size());
      }
      ss << endl;

      ss << "Estimation method: ";
      switch (act->estimationMethod)
      {
        case CActionRobotMovement2D::emOdometry:
          ss << "emOdometry\n";
          break;
        case CActionRobotMovement2D::emScan2DMatching:
          ss << "emScan2DMatching\n";
          break;
        default:
          ss << "(Unknown ID!)\n";
          break;
      };

      // Additional data:
      if (act->hasEncodersInfo)
      {
        ss << format(
          " Encoder info: deltaL=%i deltaR=%i\n",
          act->encoderLeftTicks, act->encoderRightTicks);
      }
      else
        ss << "Encoder info: Not available!\n";

      if (act->hasVelocities)
      {
        ss << format(
          " Velocity info: v=%s\n",
          act->velocityLocal.asString().c_str());
      }
      else
        ss << "Velocity info: Not available!\n";

      // Plot the 2D pose samples:
      unsigned int N = 1000;
      vector<CVectorDouble> samples(N);
      vector<float> xs(N), ys(N), ps(N), dumm(N, 0.1f);

      // // Draw a set of random (x,y,phi) samples:
      /** Needs to be uncommented once problem is resolved */
      // act->poseChange->drawManySamples(N, samples);

      // Pass to vectors and draw them:
      for (unsigned int i = 0; i < N; i++)
      {
        xs[i] = samples[i][0];
        ys[i] = samples[i][1];
        ps[i] = RAD2DEG(samples[i][2]);
      }

      // lyAction2D_XY->SetData(xs, ys);
      // lyAction2D_PHI->SetData(ps, dumm);

      // plotAct2D_XY->Fit();
      // plotAct2D_PHI->Fit();
      /** Serialize xs, ys, ps, dumm data
       *
       * Code Here
       */
    }

    if (classID == CLASS_ID(CObservationBearingRange))
    {
      // ----------------------------------------------------------------------
      //              CObservationBearingRange
      // ----------------------------------------------------------------------
      cs.changeSelection(8);
      CObservationBearingRange::Ptr obs =
        std::dynamic_pointer_cast<CObservationBearingRange>(
          sel_obj);
      // The plot:
      size_t nPts = obs->sensedData.size();
      vector<float> Xs(nPts), Ys(nPts), Zs(nPts);
      for (size_t k = 0; k < nPts; k++)
      {
        float R = obs->sensedData[k].range;
        float yaw = obs->sensedData[k].yaw;
        float pitch = obs->sensedData[k].pitch;

        CPoint3D local(
          R * cos(yaw) * cos(pitch), R * sin(yaw) * cos(pitch),
          R * sin(pitch));
        CPoint3D relRobot(obs->sensorLocationOnRobot + local);
        Xs[k] = relRobot.x();
        Ys[k] = relRobot.y();
        Zs[k] = relRobot.z();
      }
        // wxGUI code
        // lyRangeBearingLandmarks->SetData(Xs, Ys);
        // plotRangeBearing->LockAspect();
        // plotRangeBearing
        // ->Fit();  // Update the window to show the new data fitted.
      /**Serializa and send xs, ys data, this data will be displayed
       * on the plot
       * Code here
       */
    }

    if (classID == CLASS_ID(CActionRobotMovement3D))
    {
      // ----------------------------------------------------------------------
      //              CActionRobotMovement3D
      // ----------------------------------------------------------------------
      // Notebook1->ChangeSelection( 1 );
      CActionRobotMovement3D::Ptr act =
        std::dynamic_pointer_cast<CActionRobotMovement3D>(sel_obj);
      ss << "Robot Movement (as a gaussian pose change):\n";
      ss << act->poseChange << endl;
    }

    if (classID == CLASS_ID(CObservation3DRangeScan))
    {
      // ----------------------------------------------------------------------
      //              CObservation3DRangeScan
      // ----------------------------------------------------------------------
      cs.changeSelection(9);
      CObservation3DRangeScan::Ptr obs =
        std::dynamic_pointer_cast<CObservation3DRangeScan>(sel_obj);

      obs->load();  // Make sure the 3D pointcloud , etc.. are all
      // loaded in the memory

      const bool generate3Donthefly = true;
      if (generate3Donthefly)
      {
        mrpt::obs::T3DPointsProjectionParams pp;
        pp.takeIntoAccountSensorPoseOnRobot = false;
        obs->project3DPointsFromDepthImageInto(*obs, pp);
      }

      if (generate3Donthefly)
          ss << "NOTICE: The stored observation didn't contain 3D "
        "points, but these have been generated on-the-fly "
        "just for visualization purposes.\n"
        "(You can disable this behavior from the menu "
        "Sensors->3D depth cameras\n\n";

      // Update the 3D view
      if (generate3Donthefly)
      {
				mrpt::opengl::CPointCloudColoured::Ptr pnts =
					mrpt::make_aligned_shared<
						mrpt::opengl::CPointCloudColoured>();
				CColouredPointsMap pointMap;
				pointMap.colorScheme.scheme =
					CColouredPointsMap::cmFromIntensityImage;

				if (obs->hasPoints3D)
				{
					// Assign only those points above a certain threshold:
					const int confThreshold =
						obs->hasConfidenceImage ? THRESHOLD_CONFIDENCE_IMAGE : 0;

					if (confThreshold ==
						0)  // This includes when there is no confidence image.
					{
						pointMap.insertionOptions.minDistBetweenLaserPoints =
							0;  // don't drop any point
						pointMap.insertObservation(obs.get());  // This
						// transform
						// points into
						// vehicle-frame
						pnts->loadFromPointsMap(&pointMap);

						pnts->setPose(mrpt::poses::CPose3D());  // No need to
						// further
						// transform 3D
						// points
					}
					else
					{
						pnts->clear();

						const vector<float>& obs_xs = obs->points3D_x;
						const vector<float>& obs_ys = obs->points3D_y;
						const vector<float>& obs_zs = obs->points3D_z;

						size_t i = 0;

						const size_t W = obs->confidenceImage.getWidth();
						const size_t H = obs->confidenceImage.getHeight();

						ASSERT_(obs->confidenceImage.isColor() == false);
						ASSERT_(obs_xs.size() == H * W);

						for (size_t r = 0; r < H; r++)
						{
							unsigned char const* ptr_lin =
								obs->confidenceImage.get_unsafe(0, r, 0);
							for (size_t c = 0; c < W; c++, i++)
							{
								unsigned char conf = *ptr_lin++;
								if (conf >= confThreshold)
									pnts->push_back(
										obs_xs[i], obs_ys[i], obs_zs[i], 1, 1,
										1);
							}
						}
						// Translate the 3D cloud since sensed points are
						// relative to the camera, but the camera may be
						// translated wrt the robot (our 0,0,0 here):
						pnts->setPose(obs->sensorPose);
					}

					pnts->setPointSize(4.0);
				}

        /** serialize CPointCloudColoured pnts to schema
         * These points will be added to the 3D screen
         *
         * Code here
         */
        // Free memory:
				if (generate3Donthefly)
				{
					obs->hasPoints3D = false;
					obs->resizePoints3DVectors(0);
				}
      }
      else
      {
				// Update intensity image ======
				{
					CImage im;
					if (obs->hasIntensityImage)
						im = obs->intensityImage;
					else
						im.resize(10, 10, CH_GRAY, true);
          // wxGUI code
					// wxImage* img = mrpt::gui::MRPTImage2wxImage(im);
					// if (img->IsOk()) bmp3Dobs_int->SetBitmap(wxBitmap(*img));
					// bmp3Dobs_int->Refresh();
					// delete img;
          /**
           * Serializa CImage im ( it is the intensityImage) and add to ret object
           * Code here
           *
           */
					obs->intensityImage
						.unload();  // For externally-stored datasets
				}
				// Update depth image ======
				{
					CImage auxImg;
					if (obs->hasRangeImage)
					{
						// Convert to range [0,255]
						mrpt::math::CMatrix normalized_range = obs->rangeImage;
						const float max_rang =
							std::max(obs->maxRange, normalized_range.maximum());
						if (max_rang > 0) normalized_range *= 255. / max_rang;
						auxImg.setFromMatrix(
							normalized_range,
							false /* it's in range [0,255] */);
					}
					else
						auxImg.resize(10, 10, CH_GRAY, true);

					// wxImage* img = mrpt::gui::MRPTImage2wxImage(auxImg);
					// if (img->IsOk()) bmp3Dobs_depth->SetBitmap(wxBitmap(*img));
					// bmp3Dobs_depth->Refresh();
          // delete img;
          /**
           * Serializa CImage auxImg ( it is the intensityImage) and add to ret object
           * Code here
           *
           */
				}
				// Update confidence image ======
				{
					// wxImage* img;
					// if (obs->hasConfidenceImage)
					// 	img =
					// 		mrpt::gui::MRPTImage2wxImage(obs->confidenceImage);
					// else
					// {
					// 	mrpt::img::CImage dumm(10, 10);
					// 	img = mrpt::gui::MRPTImage2wxImage(dumm);
					// }
					// if (img->IsOk()) bmp3Dobs_conf->SetBitmap(wxBitmap(*img));
					// bmp3Dobs_conf->Refresh();
					// delete img;
					// obs->confidenceImage
					// 	.unload();  // For externally-stored datasets
          if (obs->hasConfidenceImage)
          {
            /** serialize confidence image to schema*/
          }
          else
          {
            mrpt::img::CImage dumm(10,10);
            /** serialize dumm to schema */
          }
          // add the serialized Image to schema
          obs->confidenceImage.unload();
				}
				obs->unload();
      }
    }

    if (classID == CLASS_ID(CObservationVelodyneScan))
    {
      /** TODO input the code */
    }

    if (classID->derivedFrom(CLASS_ID(CObservation)))
    {
      CObservation::Ptr obs(
        std::dynamic_pointer_cast<CObservation>(sel_obj));
      obs->unload();
    }

  } // scope for sel_obj != NULL
  else
  {
    cs.changeSelection(0);
    {
      // Show the comments of the rawlog:
      string s;
      rawlog->getCommentText(s);

      if (s.empty())
      {
        ss << "(The rawlog has no comments))" << endl;
      }
      else
      {
        ss << s;
      }
    }
  }
  /** Add the control signals to schema */
  ret["cs"] = cs.serializeTo();
  /** Add the stringstream ss to schema */
  ret["text"] = ss.str();
  /** Done*/
}
// add error handling by putting the entire function in a try catch phrase
// using macros
}
