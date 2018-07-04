#include "CRawlogTreeProcessor.h"
#include <mrpt/system/datetime.h>

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/obs.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace mrpt::obs;
using namespace mrpt::rtti;

using namespace mrpt::webapps;
using namespace std;
/** A tree view that represents efficiently all rawlog's items.
 */
CRawlogTreeProcessor::CRawlogTreeProcessor() {}
void CRawlogTreeProcessor::setRawlogSource(CRawlog* rawlog)
{
  m_rawlog = rawlog;
  reloadFromRawlog(-1);
}

void CRawlogTreeProcessor::getTreeIndex(Json::Value& val)
{
  uint32_t first_item = 0;
  uint32_t last_item = m_tree_nodes.size();
  for(uint32_t i = first_item; i < last_item; i++)
  {
    TNodeData& d = m_tree_nodes[i];
    Json::Value tmp;
    tmp["level"] = d.level;
    tmp["index"] = d.index;
    std::stringstream data;
    int icon = -1;
    if (i == 0)
    {
      // The root node:
      data << "Rawlog: " << m_rawlog_name;
      icon = 3;
    }
    else
    {
      // According to class ID:
      m_rawlog->getAsGeneric(d.index);  // Just to assure its on memory

      if (d.data)
      {
        // Icon:
        icon = iconIndexFromClass(d.data->GetRuntimeClass());

        // Text:
        if (d.level == 1)
        {
          data <<"["<<d.index<<"] ";
        }

        data << string(d.data->GetRuntimeClass()->className);

        // Sensor label:
        if (d.data->GetRuntimeClass()->derivedFrom(
          CLASS_ID(CObservation)))
        {
          CObservation::Ptr obs =
          std::dynamic_pointer_cast<CObservation>(d.data);

          if (!obs->sensorLabel.empty())
            data << " : " << string(obs->sensorLabel.c_str());
        }
      }
    }

    // if (m_rawlog_start != INVALID_TIMESTAMP &&
    //     m_rawlog_last != INVALID_TIMESTAMP && last_tim_y > first_tim_y)
    // {
    //   const double len_tim =
    //     mrpt::system::timeDifference(m_rawlog_start, m_rawlog_last);
    //   for (size_t i = first_time; i < last_time; i++)
    //   {
    //     TNodeData& d = m_tree_nodes[i];

    //     if (d.data)
    //     {
    //       TTimeStamp t_this = INVALID_TIMESTAMP;
    //       if (d.data->GetRuntimeClass()->derivedFrom(
    //         CLASS_ID(CObservation)))
    //       {
    //         CObservation::Ptr obs =
    //         std::dynamic_pointer_cast<CObservation>(d.data);
    //         t_this = obs->timestamp;
    //       }
    //      // More code to be added here
    //     }
    //   }
    // }
    tmp["data"] = data.str();
    tmp["icon"] = icon;
    val[i] = tmp;
  }
}

/**Serializes the data structure at specified index of the rawlog.
 */
Json::Value CRawlogTreeProcessor::getTreeDataPoint(size_t index) const
{
  Json::Value ret;
  if (index < (int)m_tree_nodes.size() && index >=0) {
    mainDisplayProcessor(m_tree_nodes[index].data, m_rawlog, ret);
  }
  return ret;
}

void CRawlogTreeProcessor::reloadFromRawlog(int hint_rawlog_items = -1)
{
  // Recompute the total height of the scroll area:
  //  We also compute a list for each index with:
  //    - Pointer to data
  //	  - level in the hierarchy (0,1,2)
  // --------------------------------------------------------

  if (m_rawlog)
  {
    if (hint_rawlog_items < 0)
      m_tree_nodes.reserve(m_rawlog->size() + 100);
    else
      m_tree_nodes.reserve(hint_rawlog_items + 100);
  }

  // Create a "tree node" for each element in the rawlog:
  // ---------------------------------------------------------
  m_tree_nodes.clear();

  m_rawlog_start = INVALID_TIMESTAMP;
  m_rawlog_last = INVALID_TIMESTAMP;

  // Root:
  m_tree_nodes.push_back(TNodeData());
  TNodeData& d = m_tree_nodes.back();
  d.level = 0;

  //	CVectorDouble	tims;

  if (m_rawlog)
  {
    CRawlog::iterator end_it = m_rawlog->end();
    size_t rawlog_index = 0;
    for (CRawlog::iterator it = m_rawlog->begin(); it != end_it;
      it++, rawlog_index++)
    {
      m_tree_nodes.push_back(TNodeData());
      TNodeData& d = m_tree_nodes.back();
      d.level = 1;
      d.data = (*it);
      d.index = rawlog_index;

      // For containers, go recursively:
      if ((*it)->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
      {
        CSensoryFrame::Ptr sf =
          std::dynamic_pointer_cast<CSensoryFrame>(*it);
        for (CSensoryFrame::iterator o = sf->begin(); o != sf->end();
          ++o)
        {
          m_tree_nodes.push_back(TNodeData());
          TNodeData& d = m_tree_nodes.back();
          d.level = 2;
          d.data = (*o);

          if ((*o)->timestamp != INVALID_TIMESTAMP)
          {
            m_rawlog_last = (*o)->timestamp;
            if (m_rawlog_start == INVALID_TIMESTAMP)
              m_rawlog_start = (*o)->timestamp;
          }
        }
      }
      else if ((*it)->GetRuntimeClass() == CLASS_ID(CActionCollection))
      {
        CActionCollection::Ptr acts =
          std::dynamic_pointer_cast<CActionCollection>(*it);
        for (CActionCollection::iterator a = acts->begin();
          a != acts->end(); ++a)
        {
          m_tree_nodes.push_back(TNodeData());
          TNodeData& d = m_tree_nodes.back();
          d.level = 2;
          d.data = a->get_ptr();

          if ((*a)->timestamp != INVALID_TIMESTAMP)
          {
            m_rawlog_last = (*a)->timestamp;
            if (m_rawlog_start == INVALID_TIMESTAMP)
              m_rawlog_start = (*a)->timestamp;
          }
        }
      }
      else if (
        (*it)->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
      {
        CObservation::Ptr o =
          std::dynamic_pointer_cast<CObservation>(*it);
        if (o->timestamp != INVALID_TIMESTAMP)
        {
          m_rawlog_last = o->timestamp;
          if (m_rawlog_start == INVALID_TIMESTAMP)
            m_rawlog_start = o->timestamp;

          // tims.push_back(
          // mrpt::system::timeDifference(m_rawlog_start,
          // o->timestamp));
        }
      }
    }
  }

  //	mrpt::system::vectorToTextFile(tims,"tims.txt");

  // Set new size:
  int ly = m_tree_nodes.size();
}

/** Returns the time of the first element in the rawlog. */
mrpt::system::TTimeStamp CRawlogTreeProcessor::getFirstTimestamp() const
{
  return m_rawlog_start;
}

int CRawlogTreeProcessor::iconIndexFromClass(const TRuntimeClassId* class_ID)
{
  int iconIndex = -1;

  if (class_ID == CLASS_ID(CObservation2DRangeScan))
    iconIndex = 6;
  else if (class_ID == CLASS_ID(CObservationImage))
    iconIndex = 4;
  else if (class_ID == CLASS_ID(CObservationStereoImages))
    iconIndex = 5;
  else if (class_ID == CLASS_ID(CObservationGPS))
    iconIndex = 7;
  else if (class_ID == CLASS_ID(CObservationGasSensors))
    iconIndex = 8;
  else if (class_ID == CLASS_ID(CObservationWirelessPower))
    iconIndex = 8;
  else if (class_ID == CLASS_ID(CObservationRFID))
    iconIndex = 8;
  else if (class_ID->derivedFrom(CLASS_ID(CObservation)))
    iconIndex = 2;  // Default observation
  else if (class_ID == CLASS_ID(CActionCollection))
    iconIndex = 0;
  else if (class_ID == CLASS_ID(CSensoryFrame))
    iconIndex = 1;
  else if (class_ID->derivedFrom(CLASS_ID(CAction)))
    iconIndex = 2;

  return iconIndex;
}
