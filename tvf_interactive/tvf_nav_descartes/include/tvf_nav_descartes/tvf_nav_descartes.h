/*------------------------------------------------------------------------
Author = Andrew Sharp
Copyright = Copyright 2017, The University of Texas at Austin,
Nuclear Robotics Group
Credits = Andrew Sharp
License = BSD
Version = 1.0.2
Maintainer = Andrew Sharp
Email = asharp@utexas.edu
Status = Production
Doc =

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------*/

#ifndef TVF_NAV_DESCARTES_H
#define TVF_NAV_DESCARTES_H

// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tvf_nav_descartes/TVFNavDescartesAction.h>
#include <tvf_nav_descartes/TVFNavDescartesGoal.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
// Includes the utility function for converting to trajectory_msgs::JointTrajectory's
#include <descartes_utilities/ros_conversions.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <move_interface/move_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

namespace tvf_nav_descartes
{
class TVFNavDescartes
{
protected:
  ros::NodeHandle node_handle_;
  // NodeHandle instance must be created the next line or a strange error occurs.
  actionlib::SimpleActionServer< tvf_nav_descartes::TVFNavDescartesAction >
    action_server_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;
  tf::TransformListener listener_;

  // Create messages that are used to published feedback/result
  tvf_nav_descartes::TVFNavDescartesFeedback feedback_;
  tvf_nav_descartes::TVFNavDescartesResult result_;

private:
  typedef std::vector< descartes_core::TrajectoryPtPtr > TrajectoryVec;
  typedef TrajectoryVec::const_iterator TrajectoryIter;

  int InitModel( );
  /*
  // Generates an completely defined (zero-tolerance) cartesian point from a pose
  descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);
  // Generates a cartesian point with free rotation about the Z axis of the EFF frame
  descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);
  */
  // Sends a ROS trajectory to the robot controller
  bool executeTrajectory( const trajectory_msgs::JointTrajectory& trajectory );
  void publishPosesMarkers( const EigenSTL::vector_Affine3d& poses );
  void clearPosesMarkers( const EigenSTL::vector_Affine3d& poses );
  void publishPathArrows( const EigenSTL::vector_Affine3d& poses );
  void clearPathArrows( const EigenSTL::vector_Affine3d& poses );
  std::vector< double > getCurrentJointState( const std::string& topic );

  std::string robot_name_;
  std::string world_frame_;
  std::vector< std::string > joint_names_;

  moveit::planning_interface::MoveGroupInterface group_;
  MoveInterface* mi_;
  descartes_core::RobotModelPtr model_;
  descartes_planner::DensePlanner planner_;

public:
  TVFNavDescartes( std::string name, std::string robot_name );
  int executeCB( const tvf_nav_descartes::TVFNavDescartesGoalConstPtr &goal );
};

}

#endif // TVF_NAV_DESCARTES_H
