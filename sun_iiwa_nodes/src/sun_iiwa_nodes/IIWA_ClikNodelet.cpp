// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include "sun_iiwa_nodes/IIWA_ClikNodelet.h"

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(sun::iiwa::IIWAClikNodelet, nodelet::Nodelet)
