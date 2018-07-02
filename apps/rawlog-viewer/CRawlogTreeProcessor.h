#pragma once

#include <mrpt/obs/CRawlog.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/CTicTac.h>

#include <mrpt/web/json_config.h>

namespace mrpt::webapps
{
  struct TNodeData
  {
    public:
    TNodeData() : level(0), data(), index(0) {}
    ~TNodeData() {}
    /** Heirarchy level: 0,1,2 */
    uint8_t level;
    /** The object, or null ptr */
    mrpt::serialization::CSerializable::Ptr data;
    size_t index;
    inline void convertToJson(Json::Value& val) {
      val["level"] = level;
      val["index"] = index;
      val["data"] = std::string(data->GetRuntimeClass()->className);
    }
  };
}
class CRawlogTreeProcessor;

/** A tree view that represents efficiently all rawlog's items.
 */
class CRawlogTreeProcessor
{
	public:
	/** Constructor
	 */
	CRawlogTreeProcessor();
	/** Sets the name of the rawlog file, used for the root item */
	inline void setRawlogName(const std::string& s) { m_rawlog_name = s; }
	/**Sets the rawlog to be rendered in the control (Its kept as a pointer,
	 * so that the original object is not destroyed).
	 * It automatically calls "reloadFromRawlog".
	 *
	 */
	void setRawlogSource(mrpt::obs::CRawlog* rawlog);
	/**
	 * Reloads the rawlog.
	 */
	void reloadFromRawlog(int hint_rawlog_items);

	/** Returns the time of the first element in the rawlog. */
	mrpt::system::TTimeStamp getFirstTimestamp() const;

  /** Get the index of the selected item */
  int GetSelectedItem() const { return m_selected_item; }

  /** Convert the m_tree_nodes data to Json::Value */
  void getTreeIndex(Json::Value& val);

  /** Convert a data point from m_tree_nodes to Json::Value */
  Json::Value getTreeDataPoint(size_t index) const;

	protected:
	mrpt::obs::CRawlog* m_rawlog; /* A reference for the rawlog to be rendered*/
	int m_selected_item;          /** Selected row, or -1 if none */
	std::string m_rawlog_name;    /** File name */
	mrpt::system::TTimeStamp m_rawlog_start;
	mrpt::system::TTimeStamp m_rawlog_last;
	std::vector<mrpt::webapps::TNodeData> m_tree_nodes;

	/**Returns an icon depending on the class of the object in the tree
	 * view
	 */
	static int iconIndexFromClass(const mrpt::rtti::TRuntimeClassId* class_ID);
};
