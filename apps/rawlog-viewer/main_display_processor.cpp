#include "CRawlogTreeProcessor.h"

#include <mrpt/system/datetime.h>
#include <mrpt/obs.h>

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/poses/CPosePDFParticles.h>

#include <iomanip>

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

void mrpt::webapps::mainDisplayProcessor(const mrpt::serialization::CSerializable::Ptr& sel_obj, Json::Value& ret)
{

}
