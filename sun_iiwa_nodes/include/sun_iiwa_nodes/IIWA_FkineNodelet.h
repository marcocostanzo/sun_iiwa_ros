#include "sun_iiwa_nodes/IIWA_FkineNode.h"
#include "sun_robot_ros/FkineNodelet.h"

namespace sun_iiwa_nodes {
class IIWA_FkineNodelet : public sun::FkineNodelet<sun::IIWA_FkineNode> {};
} // namespace sun_iiwa_nodes