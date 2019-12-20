/**
 * @file workspace_representation.cpp
 */

#include "workspace_representation/workspace_representation.h"
#include <kdl/frames_io.hpp>

using namespace std;
using namespace sensor_msgs;
using namespace workspace_representation;

WorkspaceRepresentation::WorkspaceRepresentation(const KDL::Tree& tree, const urdf::Model& model)
  : model_(model)
{
  // walk the tree and add segment to segments_
  addChildren(tree.getRootSegment());

}

void WorkspaceRepresentation::prepareWorkspace()
{

}

// add children to correct maps
void WorkspaceRepresentation::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const string& root = GetTreeElementSegment(segment->second).getName();

  auto children = GetTreeElementChildren(segment->second);
  for (int i = 0; i < children.size(); i++)
  {
    auto child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    string child_joint_name = child.getJoint().getName();
    if (child.getJoint().getType() == KDL::Joint::None)
    {
      if (model_.getJoint(child_joint_name) &&
          model_.getJoint(child_joint_name)->type == urdf::Joint::FLOATING)
      {
        ROS_INFO_STREAM("FLoating joint. Not adding segment from "
            << root << " to " << child.getName()
            <<  ". This joint is not used based on joint_states info");
      }
      else
      {
        segments_fixed_.insert(make_pair(child_joint_name, s));
        ROS_DEBUG("Adding fixed segment from %s, to %s", root.c_str(), child.getName().c_str());
      }
    }
    else
    {
      segments_.insert(make_pair(child_joint_name, s));
      ROS_DEBUG("Adding moveing segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}
