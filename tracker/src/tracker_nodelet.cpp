#include <pluginlib/class_list_macros.h>
#include <tracker/tracker_nodelet.h>

namespace track
{

void TrackerNodelet::onInit() {
    tracker = new track::Tracker(getNodeHandle());
    NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
}

PLUGINLIB_DECLARE_CLASS(track, TrackerNodelet, track::TrackerNodelet, nodelet::Nodelet);

}
