/**
 * @file workspace_representation.h
 * @note A part of this code is inspired by "ros/robot_state_publisher"
 */

#ifndef INCLUDE_WORKSPACE_REPRESENTATION_WORKSPACE_REPRESENTATION_H
#define INCLUDE_WORKSPACE_REPRESENTATION_WORKSPACE_REPRESENTATION_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>

namespace workspace_representation
{

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment,
              const std::string& p_root,
              const std::string& p_tip)
    : segment(p_segment), root(p_root), tip(p_tip)
  {
  }

  KDL::Segment segment;
  std::string root;
  std::string tip;
};

class WorkspaceRepresentation
{
public:
  WorkspaceRepresentation(const KDL::Tree& tree, const urdf::Model& model);
  virtual ~WorkspaceRepresentation();

  /// Represent workspace by sampling configuration randomly
  void prepareWorkspace();

private:
  void addChildren(const KDL::SegmentMap::const_iterator segment);

  //std::string urdf_string_;
  urdf::Model model_;
  std::map<std::string, SegmentPair> segments_, segments_fixed_;
};

}

#endif /* INCLUDE_WORKSPACE_REPRESENTATION_WORKSPACE_REPRESENTATION_H */
