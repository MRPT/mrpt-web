#include <mrpt/web/CModularServer.h>
#include <mrpt/web/CSchemeArchive.h>
#include <mrpt/web/CallBackSub.h>
#include <mrpt/web/CPubSubManagerBase.h>
#include <mrpt/web/json_config.h>

#include <mrpt/vision/CVideoFileWriter.h>
#include <mrpt/containers/stl_containers_utils.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/system/datetime.h>
#include <mrpt/math/ops_matrices.h>  // << ops
#include <mrpt/math/ops_vectors.h>  // << ops
#include <mrpt/math/wrap2pi.h>
#include <mrpt/core/aligned_std_map.h>

#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationRange.h>

#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

#include "CPubSubManager.h"
#include "CWebSocketServer.h"
#include "CRawlogTreeProcessor.h"
#include "CFormMotionModel.h"
#include "CFormRawMap.h"
#include "stubs.h"

#include <cstdlib>
#include <functional>
#include <thread>
#include <string>
#include <mutex>
#include <memory>
#include <iostream>
#include <iomanip>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::config;
using namespace mrpt::img;
using namespace mrpt::poses;
using namespace mrpt::serialization;
using namespace mrpt::rtti;
using namespace mrpt::vision;
using namespace mrpt::io;
// using namespace mrpt::gui;
// using namespace mrpt::opengl;
using namespace mrpt::web;

using namespace std;
struct TInfoPerSensorLabel
{
  TInfoPerSensorLabel()
    : max_ellapsed_tim_between_obs(.0),
      first(INVALID_TIMESTAMP),
      last(INVALID_TIMESTAMP)
  {
  }
  std::vector<double> timOccurs;
  double max_ellapsed_tim_between_obs;
  mrpt::system::TTimeStamp first, last;

  size_t getOccurences() const { return timOccurs.size(); }

  void addOcurrence(
    mrpt::system::TTimeStamp obs_tim,
    mrpt::system::TTimeStamp first_dataset_tim)
  {
    double obs_t = .0;  // 0-based timestamp:
    if (first_dataset_tim != INVALID_TIMESTAMP && obs_tim != INVALID_TIMESTAMP)
      obs_t = mrpt::system::timeDifference(first_dataset_tim, obs_tim);

    double ellapsed_tim = .0;
    if (!timOccurs.empty()) ellapsed_tim = obs_t - timOccurs.back();

    timOccurs.push_back(obs_t);
    if (ellapsed_tim > max_ellapsed_tim_between_obs)
      max_ellapsed_tim_between_obs = ellapsed_tim;
  }
};

class Stubs : public StubsAbstract
{
	public:
		Stubs():m_loaded_file_name("noname.rawlog")
		{
			m_time_to_load = 0;
			m_experiment_length = 0;
      m_tree = std::make_shared<CRawlogTreeProcessor>();
		}
    Json::Value LoadMotionModel(const Json::Value &request)
    {
      Json::Value ret;
      try{
        const Json::Value in = request["sendData"];
        CFormMotionModel motion_model;
        if(in["model"]=="Gaussian")
        {
          motion_model.loadFromGaussian(in);
        }
        else if(in["model"]=="Thrun")
        {
          motion_model.loadFromThrun(in);
        }
        else
        {
          std::cout << "The model should be Gaussian or Thrun." << std::endl;
        }
        size_t first = in.get("first", 0).asInt();
        size_t last = in.get("last", rawlog.size() - 1).asInt();
        motion_model.applyToLoadedRawlog(rawlog, first, last);
        ret["loaded"] = true;
      }
      catch(exception& e)
      {
        ret["err"] = e.what();
        std::cout << e.what() << std::endl;
      }
      return ret;
    }
    Json::Value GetRawlogDataFromIndex(const Json::Value &request)
    {
      const int index = request.get("index", 0).asInt();
      Json::Value ret;
      ret = m_tree->getTreeDataPoint(index);
      return ret;
    }
    Json::Value GetMapAndPath(const Json::Value &request)
    {
      CFormRawMap map_generator(rawlog);
      size_t firstEntry = request.get("firstEntry", 0).asUInt();
      size_t lastEntry = request.get("lastEntry", 0).asUInt();
      size_t decimate = request.get("decimate", 0).asUInt();
      Json::Value ret;
      map_generator.OnbtnGenerateClick(firstEntry, lastEntry, decimate, ret);
      return ret;
    }
    Json::Value GetMapFromRTK(const Json::Value &request)
    {
      CFormRawMap map_generator(rawlog);
      size_t firstEntry = request.get("firstEntry", 0).asUInt();
      size_t lastEntry = request.get("lastEntry", 0).asUInt();
      size_t decimate = request.get("decimate", 0).asUInt();
      Json::Value ret;
      map_generator.OnGenerateFromRTK(firstEntry, lastEntry, decimate, ret);
      return ret;
    }
    Json::Value GetRandomPaths(const Json::Value &request)
    {
      CFormRawMap map_generator(rawlog);
      size_t firstEntry = request.get("firstEntry", 0).asUInt();
      size_t lastEntry = request.get("lastEntry", 0).asUInt();
      size_t decimate = request.get("decimate", 0).asUInt();
      Json::Value ret;
      map_generator.OnbtnGeneratePathsClick(firstEntry, lastEntry, decimate, ret);
      return ret;
    }
    Json::Value GetRawlogTree(const Json::Value &request)
    {
      Json::Value ret;
      m_tree->getTreeIndex(ret);
      return ret;
    }
		Json::Value LoadRawlog(const Json::Value& request)
		{
			// TODO: add something similar to WX_START_TRY
			Json::Value ret;
			ret["err"] = "";
			ret["loaded"] = false;
			// remove the parameters from JSON object
			const std::string path = request.get("path","").asString();
			const int first = request.get("first", 0).asInt();
			const int last = request.get("last", -1).asInt();
			if(!fileExists(path))
			{
				ret["err"] = string("File does not exist");
				ret["loaded"] = false;
				return ret;
			}
			CFileGZInputStream fil(path);
			uint64_t fil_size = fil.getTotalBytesCount();
			m_loaded_file_name = path;
			// let the app process the messages
			/**
			 * code here
			 */
			// clear first
			rawlog.clear();
			m_crono_loading.Tic();

			size_t count_loop = 0;
			int entry_index = 0;
			bool keep_loading = true;
			bool already_warned_too_large_file = false;
			string err_msg;

			while(keep_loading)
			{
				if(count_loop++ % 10 == 0)
				{
					uint64_t fil_pos = fil.getPosition();
					static double last_ratio = -1;
					double ratio = fil_pos / (1.0 * fil_size);

					if(ratio - last_ratio >= 0.006)
					{
						last_ratio = ratio;

						unsigned long memUsg = getMemoryUsage();
						double memUsg_Mb = memUsg / (1024.0 * 1024.0);
						if (memUsg_Mb > 2600 && !already_warned_too_large_file)
						{
							already_warned_too_large_file = true;
							err_msg += string("Memory Usage exceeded 2600 MB.");
							keep_loading = false;
						}
					}
				}
				// Try to load the Object into a serializable Object
				CSerializable::Ptr new_obj;
				try
				{
					archiveFrom(fil) >> new_obj;
					//Check type:
					if (new_obj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
					{
						if (entry_index >= first && (last == -1 || entry_index <= last))
							rawlog.addObservationsMemoryReference(
								std::dynamic_pointer_cast<CSensoryFrame>(new_obj)
								);
						entry_index++;
					}
					else if (new_obj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
					{
						if (entry_index >= first && (last == -1 || entry_index <= last))
							rawlog.addActionsMemoryReference(
								std::dynamic_pointer_cast<CActionCollection>(new_obj)
								);
								entry_index++;
					}
					/* Added in MRPT 0.6.0: The new "observations only" format: */
					else if (new_obj->GetRuntimeClass()->derivedFrom(
								CLASS_ID(CObservation)))
					{
						if (entry_index >= first && (last == -1 || entry_index <= last))
							rawlog.addObservationMemoryReference(
								std::dynamic_pointer_cast<CObservation>(new_obj));
						entry_index++;
					}
					/* FOR BACKWARD COMPATIBILITY: CPose2D was used previously instead
						of an "ActionCollection" object
																						26-JAN-2006	*/
					else if (new_obj->GetRuntimeClass() == CLASS_ID(CPose2D))
					{
						if (entry_index >= first && (last == -1 || entry_index <= last))
						{
							CPose2D::Ptr poseChange =
								std::dynamic_pointer_cast<CPose2D>(new_obj);
							CActionCollection::Ptr temp =
								mrpt::make_aligned_shared<CActionCollection>();
							CActionRobotMovement2D action;
							CActionRobotMovement2D::TMotionModelOptions options;
							action.computeFromOdometry(*poseChange, options);
							temp->insert(action);

							rawlog.addActionsMemoryReference(temp);
						}
						entry_index++;
					}
					else if (new_obj->GetRuntimeClass() == CLASS_ID(CRawlog))
					{
						CRawlog::Ptr rw = std::dynamic_pointer_cast<CRawlog>(new_obj);
						rawlog = std::move(*rw);
					}
					else
					{
						// Unknown class:
						// New in MRPT v1.5.0: Allow loading some other classes:
						rawlog.addGenericObject(new_obj);
					}

					// Passed last?
					if (last != -1 && entry_index > last) keep_loading = false;
				}
				catch (std::bad_alloc&)
				{
					// Probably we're in a 32 bit machine and we rose up to 2Gb of
					// mem... free
					//  some, give a warning and go on.
					if (rawlog.size() > 10000)
					{
						size_t NN = rawlog.size() - 10000;
						while (rawlog.size() > NN) rawlog.remove(NN);
					}
					else
						rawlog.clear();

					string s =
						"OUT OF MEMORY: The last part of the rawlog has been freed "
							"to allow the program to continue.\n";
		#if MRPT_WORD_SIZE == 32
					s +=
						"  This is a 32bit machine, so the maximum memory available is "
						"2Gb despite of the real RAM installed.";
		#endif
					err_msg += s;
					keep_loading = false;
				}
				catch (exception& e)
				{
					err_msg += e.what();
					keep_loading = false;
				}
				catch (...)
				{
					keep_loading = false;
				}
			} // end while keep loading

			m_time_to_load = m_crono_loading.Tac();

			// Build the tree view
			rebuildTreeView();

			///set error messages to return object
			ret["err"] = err_msg;
			ret["loaded"] = true;
			return ret;
			// add some thing similar to WX_END_TRY
		}
	private:
		void rebuildTreeView() {
			string s;
			float totalDistance;
			bool firstSF = true;
			TTimeStamp tim_start = INVALID_TIMESTAMP,
			tim_last = INVALID_TIMESTAMP;
			int count_loop = 0;

      listOfSensorLabels.clear();

      // Refresh the custom tree view
      m_tree->setRawlogName(m_loaded_file_name);
      m_tree->setRawlogSource(&rawlog);
      using TListOfObjectsOccurs = std::map<const TRuntimeClassId*, size_t>;
      TListOfObjectsOccurs listOfObjects;

			// The elements:
			for (unsigned int i = 0; i < rawlog.size(); i++)
			{
				switch (rawlog.getType(i))
				{
					case CRawlog::etActionCollection:
					{
						CActionCollection::Ptr acts = rawlog.getAsAction(i);

						// Distance:
						CPose2D est;
						if (acts->getBestMovementEstimation())
						{
							acts->getBestMovementEstimation()->poseChange->getMean(est);
							totalDistance += est.norm();
						}
					}
					break;

					case CRawlog::etSensoryFrame:
					{
						CSerializable::Ptr obj = rawlog.getAsObservations(i);
						if (CLASS_ID(CSensoryFrame) != obj->GetRuntimeClass())
							THROW_EXCEPTION("Expected an object of class CSensoryFrame!!");

						CSensoryFrame::Ptr sf =
							std::dynamic_pointer_cast<CSensoryFrame>(obj);

						if (firstSF)
						{
							if (sf->size())
							{
								firstSF = false;
								tim_start = (*sf->begin())->timestamp;
							}
						}
						if (sf->size())
							tim_last = (*sf->begin())->timestamp;  // Keep the last one

						size_t j, n = sf->size();

						for (j = 0;j < n; j++)
						{
							CObservation::Ptr obs = sf->getObservationByIndex(j);

							// Stats:
							listOfObjects[obs->GetRuntimeClass()]++;
							TInfoPerSensorLabel& dd =
								listOfSensorLabels[obs->sensorLabel];
							dd.addOcurrence(obs->timestamp, tim_start);
							if (dd.first == INVALID_TIMESTAMP)
								dd.first = obs->timestamp;
							dd.last = obs->timestamp;
						}
					} // end sensory frame
					break;

					case CRawlog::etObservation:
					{
						CObservation::Ptr obs = rawlog.getAsObservation(i);

						if (tim_start == INVALID_TIMESTAMP) tim_start = obs->timestamp;

						tim_last = obs->timestamp; // Keep the last one

						// Stats:
						listOfObjects[obs->GetRuntimeClass()]++;

						// 0-based timestamp
						TInfoPerSensorLabel& dd= listOfSensorLabels[obs->sensorLabel];
						dd.addOcurrence(obs->timestamp, tim_start);
						if (dd.first == INVALID_TIMESTAMP) dd.first = obs->timestamp;
						dd.last = obs->timestamp;

						// For odometry measurements: total distance:
						if (obs->GetRuntimeClass() == CLASS_ID(CObservationOdometry))
						{
							CObservationOdometry::Ptr odoObs =
							std::dynamic_pointer_cast<CObservationOdometry>(obs);

							static CPose2D oldOdo;
							static bool oldOdo_first = true;

							if (oldOdo_first)
							{
								oldOdo_first = false;
								oldOdo = odoObs->odometry;
							}
							else
							{
								CPose2D inc = odoObs->odometry - oldOdo;
								oldOdo = odoObs->odometry;

								totalDistance += inc.norm();
							}
						}

					} // end Observation
					break;
					default:
						break;
				}; // end switch type

			} // end for i
      // Need to create a statistics handler + widget
      // Add Time to load file to statHandler

      // Add number of records loaded

      // Add distance travelled

      // Add info about first time stamp

      // Add data set length as time

      // Add stats of object classes

      // add data about count of different object types and freq

      // add data from listOfSensorLabels
		}

	private:
	std::shared_ptr<CPubSubManagerBase> m_manager;
  std::shared_ptr<CRawlogTreeProcessor> m_tree;
	CRawlog rawlog;
	TTimeStamp rawlog_first_timestamp = INVALID_TIMESTAMP;

	CTicTac m_crono_loading;
	string m_loaded_file_name;
	float m_time_to_load;
	double m_experiment_length;
  private:
  // A list of sensor labels (and the times they appear) in the currently loaded
  // rawlog.
  std::map<std::string, TInfoPerSensorLabel> listOfSensorLabels;

};

int main(int argc,char* argv[])
{
  //Check the command line arguments,
  if (argc != 3)
  {
      std::cerr <<
          "Usage: mrpt-ws-rpc <address> <port>\n" <<
          "Example:\n" <<
          "    mrpt-ws-rpc 127.0.0.1 8080\n";
      return EXIT_FAILURE;
  }
  auto const address = boost::asio::ip::make_address(argv[1]);
  auto const port = static_cast<unsigned short>(std::atoi(argv[2]));
  try{
    std::unique_ptr<CModularServer<>> jsonrpcIpcServer;
    using FullServer = CModularServer<Stubs>;
    //Server without ConnectionStore and SubscriberManager
    // auto server = new CWebSocketServer(port);
    //Server with ConnectionStore and SubscriberManager
    auto server = new CWebSocketServer(port);

    // Managers for Publishers and Subscibers
    auto manager = std::make_shared<CPubSubManager<CWebSocketServer>>(server);

    jsonrpcIpcServer.reset(new FullServer(
      new Stubs   // RPC stub for PubSub activity
    ));

    jsonrpcIpcServer->addConnector(server);


    // Server starts listening
    jsonrpcIpcServer->StartListening();
    getchar();
    // Server stops listening to connections
    jsonrpcIpcServer->StopListening();
  }
  catch(const std::exception& e)
  {
    std::cerr <<"Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
