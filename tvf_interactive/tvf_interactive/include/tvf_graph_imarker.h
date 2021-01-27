/*------------------------------------------------------------------------
Author = Andrew Sharp
Copyright = Copyright 2017, The University of Texas at Austin,
Nuclear Robotics Group
Credits = Andrew Sharp
License = BSD
Version = 0.0.1
Maintainer = Andrew Sharp
Email = asharp@utexas.edu
Status = Production
Doc = This class operates the TVF graph interactive marker server for IK
testing and part placement experiments.

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

#ifndef TVF_GRAPH_IMARKER_H
#define TVF_GRAPH_IMARKER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlanningScene.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigenvalues>

#include <move_interface/move_interface.h>
#include <tvf_nav_descartes/tvf_nav_descartes.h>
#include <tvf_nav_descartes/TVFNavDescartesGoal.h>
#include <tvf_graph.h>

namespace tvf_graph_imarker
{
class TVFGraphIMarker
{
public:
  TVFGraphIMarker(
    boost::shared_ptr< interactive_markers::InteractiveMarkerServer > tvf_server,
    boost::shared_ptr< interactive_markers::InteractiveMarkerServer > mesh_server,
    std::string robot_name, std::string eef,
    robot_model::RobotModelPtr kinematic_model,
    std::vector < pcl::PointCloud<pcl::PointNormal >::Ptr,
    Eigen::aligned_allocator < pcl::PointCloud< pcl::PointNormal >::Ptr > > pc_vector );
  ~TVFGraphIMarker( );

  // Interactive marker servers
  boost::shared_ptr< interactive_markers::InteractiveMarkerServer >
    tvf_server_, mesh_server_;
  // Action client for executing path motions
  actionlib::SimpleActionClient< tvf_nav_descartes::TVFNavDescartesAction > ac_;

  // Functions
  void LoadParams( std::string id );
  void LoadGraph( );
  //void SetControl( int control_method );
  void RunMove( );
  void MakeMeshControl( );
  void FrameCallback( const ros::TimerEvent& );

private:
  // ROS
  MoveInterface* mi_;
  tvf_graph::TVFGraph tvf_graph_;
  //std::vector< MoveInterface* > mi_vector_;
  ros::NodeHandle node_handle_;
  ros::Publisher marker_pub_;
  tf::TransformListener listener_;
  tf::Transform tvf_transform_;
  geometry_msgs::Pose tvf_pose_;
  geometry_msgs::PoseStamped pose_stamped_, chosen_pose_, mesh_pose_;
  // Create planning scene and publisher for adding the VF collision mesh
  moveit_msgs::PlanningScene moveit_scene_;
  ros::Publisher moveit_scene_pub_;
  visualization_msgs::InteractiveMarker mesh_marker_;
  // Variables for IK check
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  moveit::core::GroupStateValidityCallbackFn constraint_fn_;
  // Create planning scene for collision checking
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ptr_;
  planning_scene::PlanningScene planning_scene_;
  std::vector< planning_scene::PlanningScenePtr > planning_scene_vector_;
  //planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene_;
  //std::vector< planning_scene_monitor::LockedPlanningSceneRW >
  //planning_scene_rw_vec_;
  //TODO: remove frame_translation_ and frame_rotation_ as obsolete parameters
  // Pulled from ROS parameter values
  std::string robot_name_, robot_eef_, tvf_id_, world_frame_, package_name_,
    folder_path_, vf_frame_, file_path_, reachable_string_;
  std::vector< double > xyz_, rpy_, frame_translation_, frame_rotation_,
    mesh_translation_, mesh_rotation_, condition_numbers_;
  double accessible_percentage_ = 0.0, speed_ = 0.0, vf_scale_factor_,
    mesh_marker_scale_factor_, marker_scale_factor_;
  // Class
  unsigned int current_vertex_ = 0;
  int rotational_resolution_ = 0, nearest_neighbors_ = 0;
  std::vector < unsigned int > path_vertices_;
  bool time_to_move_ = false, is_cart_ = false;
  // Colors
  std::vector< double > black_ = { 0.0, 0.0, 0.0 },
                        red_ = { 1.0, 0.0, 0.0 },
                        green_ = { 0.0, 1.0, 0.0 },
                        blue_ = { 0.0, 0.0, 1.0 },
                        white_ = { 1.0, 1.0, 1.0 };
  // Graph iterators for pose stamped graph
  std::pair< tvf_graph::TVFGraph::VertexIterator,
             tvf_graph::TVFGraph::VertexIterator > vs_;
  std::pair< tvf_graph::TVFGraph::AdjacencyIterator,
             tvf_graph::TVFGraph::AdjacencyIterator > neighbors_;
  std::pair< tvf_graph::TVFGraph::EdgeIterator,
             tvf_graph::TVFGraph::EdgeIterator > es_;
  // Virtual fixture graph
  tvf_graph::TVFGraph::UndirectedGraph neighbor_graph_, path_graph_;
  // Interactive marker menu handler for the right click menu
  interactive_markers::MenuHandler menu_handler_;
  // Functions
  void InitMenu( );
  //void FullControl( );
  void NearestNeighborControl( );
  double CheckCollisions( const unsigned int index,
                          tvf_graph::TVFGraph::UndirectedGraph &graph );
  bool IsStateValid( planning_scene::PlanningScene *planning_scene,
                     moveit::core::RobotState *robot_state,
                     const moveit::core::JointModelGroup *group,
                     const double *ik_solution );
  bool IsStateValidMulti( planning_scene::PlanningScenePtr planning_scene,
                          moveit::core::RobotState *robot_state,
                          const moveit::core::JointModelGroup *group,
                          const double *ik_solution );
  void UpdateTVF( );
  void TVFIMarker( const unsigned int index_val, std::vector< double > &color,
                   tvf_graph::TVFGraph::UndirectedGraph &graph );
  void LoadCollisionMeshes( );
  void ProcessFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void ProcessMeshFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void UpdateVertexPose( const unsigned int vertex,
                         const unsigned int index,
                         geometry_msgs::Pose pose );
  void MoveToPose( const unsigned int vertex_index );
  void AddVertexToPath( const unsigned int vertex_index );
  void RemoveVertexFromPath( const unsigned int vertex_index );
  void UpdatePathMarkers( );
  void AddPathArrow( const unsigned int start, const unsigned int end,
                     std::vector< double > &color );
  void RemovePathArrow( const unsigned int start );

  geometry_msgs::PoseArray PathVerticesToPoseArray( );
  void ClearPathVertices( );
  void PlanWithDescartes( );
  void CheckConditionNumber( );
};

}

#endif