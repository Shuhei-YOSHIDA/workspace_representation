/**
 * @file workspace_representation.h
 */

#ifndef INCLUDE_WORKSPACE_REPRESENTATION_WORKSPACE_REPRESENTATION_H
#define INCLUDE_WORKSPACE_REPRESENTATION_WORKSPACE_REPRESENTATION_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>

namespace workspace_representation
{

class WorkspaceRepresentation
{
private:
  std::string urdf_string_;

public:
  WorkspaceRepresentation(std::string urdf_string);
  virtual ~WorkspaceRepresentation();

  /// Represent workspace by sampling configuration randomly
  void prepareWorkspace();
};

}

#endif /* INCLUDE_WORKSPACE_REPRESENTATION_WORKSPACE_REPRESENTATION_H */
