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
Doc = This code launches the TVF graph interactive marker server node.

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

#include <chrono>
#include <ros/ros.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <tvf_graph_imarker.h>
#include <tvf_nav_descartes/tvf_nav_descartes.h>

int main ( int argc, char** argv )
{
  std::string node_name = "tvf_graph_imarker_node", mesh, robot_name, eef,
    tvf_id;
  // Create vectors of point clouds for layer storage
  std::vector < pcl::PointCloud< pcl::PointNormal >::Ptr,
                Eigen::aligned_allocator< pcl::PointCloud<
                pcl::PointNormal >::Ptr > > pc_vector;
  // Start the clock
  std::chrono::system_clock::time_point start = std::chrono::system_clock::now( );
  // Start ROS
  ros::init( argc, argv, node_name );
  ros::NodeHandle node_handle;
  ros::Time::init( );
  ros::AsyncSpinner spinner ( 3 );
  spinner.start( );
  // Robot model variables
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel( );
  // Create Interactive Marker server to pass into the imarker class
  boost::shared_ptr< interactive_markers::InteractiveMarkerServer > tvf_server,
    mesh_server;
  // Reset IM servers
  mesh_server.reset(
    new interactive_markers::InteractiveMarkerServer( "mesh_imarkers" , "" ,
                                                      false ) );
  tvf_server.reset(
    new interactive_markers::InteractiveMarkerServer( "tvf_imarkers" , "" ,
                                                       false ) );
  // Pull ROS params
  if ( !ros::param::get( "robot_name", robot_name ) )
  {
    ROS_ERROR_STREAM( "Failed to get param 'robot_name'" );
  }
  if ( !ros::param::get( "robot_eef", eef ) )
  {
    ROS_ERROR_STREAM( "Failed to get param 'robot_name'" );
  }
  if ( !ros::param::get( "tvf_id", tvf_id ) )
  {
    ROS_ERROR_STREAM( "Failed to get param 'tvf_id'" );
  }
  // Create vf graph imarker class and provide with the initialized servers
  tvf_graph_imarker::TVFGraphIMarker tvf_graph_imarker( tvf_server, mesh_server,
                                                        robot_name, eef,
                                                        kinematic_model,
                                                        pc_vector );
  tvf_graph_imarker.LoadParams( tvf_id );
  tvf_graph_imarker.LoadGraph( );
  ROS_INFO_STREAM( "Load graph successful." );
  tvf_graph_imarker.MakeMeshControl( );
  ROS_INFO_STREAM( "Mesh control successful." );
  ros::Timer frame_timer =
    node_handle.createTimer( ros::Duration( 0.01 ),
                             &tvf_graph_imarker::TVFGraphIMarker::FrameCallback,
                             &tvf_graph_imarker );
  ROS_INFO_STREAM( "Frame timer created." );
  tvf_graph_imarker.RunMove( );
  //ROS_INFO_STREAM( "Run move complete." );
  // Apply changes to the server and spin
  mesh_server->applyChanges( );
  tvf_server->applyChanges( );
  //ros::spin( );
  ros::waitForShutdown( );
  mesh_server.reset( );
  tvf_server.reset( );
}