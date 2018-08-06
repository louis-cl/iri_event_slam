#pragma once
#include <nodelet/nodelet.h>
#include "tracker/tracker.h"

namespace track
{
    
class TrackerNodelet : public nodelet::Nodelet {
public:
    virtual void onInit();

private:
    track::Tracker* tracker;
};

}
