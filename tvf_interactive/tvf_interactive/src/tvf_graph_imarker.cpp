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

#include <tvf_graph_imarker.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <omp.h>

namespace tvf_graph_imarker
{

TVFGraphIMarker::TVFGraphIMarker(
    boost::shared_ptr< interactive_markers::InteractiveMarkerServer > tvf_server,
    boost::shared_ptr< interactive_markers::InteractiveMarkerServer > mesh_server,
    std::string robot_name, std::string eef,
    robot_model::RobotModelPtr kinematic_model,
    std::vector < pcl::PointCloud<pcl::PointNormal >::Ptr,
                  Eigen::aligned_allocator < pcl::PointCloud< pcl::PointNormal >::Ptr > >
    pc_vector ):
  tvf_server_( tvf_server ),
  mesh_server_( mesh_server ),
  ac_( "tvf_nav_descartes_server", true ),
  moveit_scene_( ),
  planning_scene_( kinematic_model ),
  tvf_graph_( pc_vector )
{
  // Check validity of inputs
  robot_name_ = robot_name;
  robot_eef_ = eef;
  kinematic_model_ = kinematic_model;
  // Get kinematic state and fill joint model
  kinematic_state_.reset( new robot_state::RobotState( kinematic_model_ ) );
  joint_model_group_ = kinematic_model_->getJointModelGroup( robot_name_ );
  // Start marker publisher
  marker_pub_ =
    node_handle_.advertise< visualization_msgs::Marker >( "tvf_markers", 1000 );
  // Start MoveIt scene publisher and planning scene monitor
  moveit_scene_pub_ =
    node_handle_.advertise< moveit_msgs::PlanningScene >( "/planning_scene", 1 );
  planning_scene_monitor_ptr_ =
    std::make_shared< planning_scene_monitor::PlanningSceneMonitor >( "robot_description" );
  // Create MoveInterface
  mi_ = new MoveInterface( );
  // Create MoveInterface for each thread
  /*
  #pragma omp parallel
  #pragma omp critical
  {
    if ( omp_get_thread_num( ) == 0 )
    {
      //planning_scene_vector_.resize( omp_get_num_threads( ) );
      for ( unsigned int i = 0; i < omp_get_num_threads( ); ++i )
      {
        mi_vector_.push_back( new MoveInterface( ) );
      }
    }
  }
  */

  // Wait for Descartes action client
  ROS_INFO_STREAM( "Waiting on action server." );
  ac_.waitForServer( );
  // Initiate the interactive marker menu
  InitMenu( );
}

TVFGraphIMarker::~TVFGraphIMarker( )
{
  /*
  for ( std::vector< MoveInterface* >::iterator iter = mi_vector_.begin( );
        iter != mi_vector_.end( ); ++iter )
  {
    delete( *iter );
  }
  mi_vector_.clear( );
  */
}

void TVFGraphIMarker::InitMenu( )
{
  // Add options for right click on interactive marker options
  menu_handler_.insert( "Make current pose.",
    boost::bind( &TVFGraphIMarker::ProcessFeedback, this, _1 ) );
  menu_handler_.insert( "Update pose.",
    boost::bind( &TVFGraphIMarker::ProcessFeedback, this, _1 ) );
  menu_handler_.insert( "Move to pose.",
    boost::bind( &TVFGraphIMarker::ProcessFeedback, this, _1 ) );
  menu_handler_.insert( "Add pose to path.",
    boost::bind( &TVFGraphIMarker::ProcessFeedback, this, _1 ) );
  menu_handler_.insert( "Remove pose from path.",
    boost::bind( &TVFGraphIMarker::ProcessFeedback, this, _1 ) );
  menu_handler_.insert( "Test path.",
    boost::bind( &TVFGraphIMarker::ProcessFeedback, this, _1 ) );
  menu_handler_.insert( "Clear path.",
    boost::bind( &TVFGraphIMarker::ProcessFeedback, this, _1 ) );
}

void TVFGraphIMarker::LoadParams( std::string tvf_id )
{
  tvf_id_ = tvf_id;
  // ROS parameter names
  std::string surface_str = "/surface_" + tvf_id_;
  std::string vf_frame_str = surface_str + "/name";
  std::string package_name_str = surface_str + "/package_name";
  std::string folder_path_str = surface_str + "/folder_path";
  std::string xyz_str = surface_str + "_xyz";
  std::string rpy_str = surface_str + "_rpy";
  std::string frame_translation_str = surface_str + "/frame_translation";
  std::string frame_rotation_str = surface_str + "/frame_rotation";
  std::string mesh_translation_str = surface_str + "/mesh_translation";
  std::string mesh_rotation_str = surface_str + "/mesh_rotation";
  std::string vf_scale_factor_str = surface_str + "/vf_scale_factor";
  std::string mesh_marker_scale_factor_str =
    surface_str + "/mesh_marker_scale_factor";
  std::string marker_scale_factor_str = surface_str + "/marker_scale_factor";
  std::string nearest_neighbors_str = surface_str + "/nearest_neighbors";
  std::string rotational_resolution_str =
    surface_str + "/rotational_resolution";
  // Get ROS parameter values
  if ( !ros::param::get( "world", world_frame_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param 'world'" );
  }
  if ( !ros::param::get( vf_frame_str, vf_frame_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << vf_frame_str );
  }
  if ( !ros::param::get( package_name_str, package_name_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << package_name_str );
  }
  if ( !ros::param::get( folder_path_str, folder_path_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << folder_path_str );
  }
  if ( !ros::param::get( xyz_str, xyz_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << xyz_str );
  }
  if ( !ros::param::get( rpy_str, rpy_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << rpy_str );
  }
  //TODO: remove frame_translation_ and frame_rotation_ as obsolete parameters
  if ( !ros::param::get( frame_translation_str, frame_translation_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << frame_translation_str );
  }
  if ( !ros::param::get( frame_rotation_str, frame_rotation_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << frame_rotation_str);
  }
  if ( !ros::param::get( mesh_translation_str, mesh_translation_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << mesh_translation_str);
  }
  if ( !ros::param::get( mesh_rotation_str, mesh_rotation_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << mesh_rotation_str);
  }
  if ( !ros::param::get( vf_scale_factor_str, vf_scale_factor_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << vf_scale_factor_str );
  }
  if ( !ros::param::get( mesh_marker_scale_factor_str,
       mesh_marker_scale_factor_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << mesh_marker_scale_factor_str );
  }
  if ( !ros::param::get( marker_scale_factor_str, marker_scale_factor_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << marker_scale_factor_str );
  }
  if ( !ros::param::get( nearest_neighbors_str, nearest_neighbors_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << nearest_neighbors_str );
  }
  if ( !ros::param::get( rotational_resolution_str, rotational_resolution_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << rotational_resolution_str <<
                      " setting to 0.");
    rotational_resolution_ = 0;
  }
  // Build pose out of mesh transform data
  mesh_pose_.header.frame_id = "/" + vf_frame_;
  mesh_pose_.pose.position.x = mesh_translation_[ 0 ];
  mesh_pose_.pose.position.y = mesh_translation_[ 1 ];
  mesh_pose_.pose.position.z = mesh_translation_[ 2 ];
  tf::Quaternion quaternion =
      tf::createQuaternionFromRPY( mesh_rotation_[ 0 ],
                                   mesh_rotation_[ 1 ],
                                   mesh_rotation_[ 2 ] );
  // Copy over quaternion data
  mesh_pose_.pose.orientation.x = quaternion.x( );
  mesh_pose_.pose.orientation.y = quaternion.y( );
  mesh_pose_.pose.orientation.z = quaternion.z( );
  mesh_pose_.pose.orientation.w = quaternion.w( );
  // Assign starting TVF world frame pose
  tvf_pose_.position.x = xyz_[ 0 ];
  tvf_pose_.position.y = xyz_[ 1 ];
  tvf_pose_.position.z = xyz_[ 2 ];
  quaternion = tf::createQuaternionFromRPY( rpy_[ 0 ], rpy_[ 1 ], rpy_[ 2 ] );
  // Copy over quaternion data
  tvf_pose_.orientation.x = quaternion.x( );
  tvf_pose_.orientation.y = quaternion.y( );
  tvf_pose_.orientation.z = quaternion.z( );
  tvf_pose_.orientation.w = quaternion.w( );
  // TVF transform from world frame
  tvf_transform_.setOrigin( tf::Vector3( tvf_pose_.position.x,
                                         tvf_pose_.position.y,
                                         tvf_pose_.position.z ) );
  tvf_transform_.setRotation( quaternion );
  // Frame_id is world frame since the tvf frame isn't necessarily published yet
  mesh_marker_.header.frame_id = world_frame_;
  mesh_marker_.scale = mesh_marker_scale_factor_;
  // Set the file path
  file_path_ = ros::package::getPath( package_name_ ) + folder_path_ + vf_frame_;
  // Set Move Interface parameters
  mi_->initialize( robot_name_, world_frame_ );
  mi_->setPlannerId( "RRTConnectkConfigDefault" );
  mi_->setPlanningTime( 1.0 );
  /*
  for ( unsigned int i = 0; i < mi_vector_.size( ); ++i )
  {
    mi_vector_[ i ]->initialize( robot_name_, world_frame_ );
    mi_vector_[ i ]->setPlannerId( "RRTConnectkConfigDefault" );
    mi_vector_[ i ]->setPlanningTime( 1.0 );
  }
  */
}

void TVFGraphIMarker::LoadGraph(  )
{
  // Output mesh path
  ROS_INFO_STREAM( "Input file path: " << file_path_ );
  // Initialize class and load graph structure
  std::vector < pcl::PointCloud< pcl::PointNormal >::Ptr,
                Eigen::aligned_allocator< pcl::PointCloud<
                pcl::PointNormal >::Ptr > > pc_vector;
  tvf_graph_.LoadParams( tvf_id_ );
  tvf_graph_.LoadGraph( );
}

void TVFGraphIMarker::RunMove( )
{
  // Function to read and execute pose moves
  ros::Rate rate( 5 );
  while ( ros::ok( ) )
  {
    if ( time_to_move_ )
    {
      // Move to pose stamped variable constructed in MoveToPose
      mi_->moveArm( pose_stamped_, speed_, is_cart_ );
      if( mi_->waitForStatus( ) == MoveInterface::STATUS_ERROR )
      {
        ROS_ERROR_STREAM( "Move to graph vertex incomplete." );
      }
      else
      {
        ROS_INFO_STREAM( "Move to graph vertex complete." );
      }
      // Reset bool for next move
      time_to_move_ = false;
    }
    rate.sleep( );
  }
}

void TVFGraphIMarker::UpdateTVF( )
{
  // Create interactive marker to show information
  visualization_msgs::InteractiveMarker int_marker;
  visualization_msgs::InteractiveMarkerControl control;
  visualization_msgs::Marker marker;
  // Clear the tvf marker server
  tvf_server_->clear( );
  NearestNeighborControl( );
  // Fill in information
  int_marker.name = "Info";
  int_marker.header.frame_id = vf_frame_;
  int_marker.pose.orientation.w = 1.0;
  int_marker.scale = mesh_marker_.scale;
  int_marker.description =
    "Pose frame_id: " + chosen_pose_.header.frame_id +
    ",\np.x: " + std::to_string( chosen_pose_.pose.position.x ) +
    ", p.y: " + std::to_string( chosen_pose_.pose.position.y ) +
    ", p.z: " + std::to_string( chosen_pose_.pose.position.z ) +
    ",\no.x: " + std::to_string( chosen_pose_.pose.orientation.x ) +
    ", o.y: " + std::to_string( chosen_pose_.pose.orientation.y ) +
    ", o.z: " + std::to_string( chosen_pose_.pose.orientation.z ) +
    ", o.w: " + std::to_string( chosen_pose_.pose.orientation.w ) +
    "\n" + reachable_string_ + ", Percentage: " +
    std::to_string( accessible_percentage_ );
  control.always_visible = true;
  control.markers.push_back( marker );
  int_marker.controls.push_back( control );
  // Update server with marker, the callback function for the marker
  tvf_server_->insert( int_marker );
  tvf_server_->setCallback( int_marker.name,
                            boost::bind( &TVFGraphIMarker::ProcessFeedback,
                                         this, _1 ) );
  ROS_INFO_STREAM( "Done calculating IK." );
  // Load markers for previously constructed path if available
  if ( path_vertices_.size( ) )
  {
    ROS_INFO_STREAM( "Loading previous path." );
    AddVertexToPath( path_vertices_.back( ) );
  }
  UpdatePathMarkers( );
  tvf_server_->applyChanges( );
}

void TVFGraphIMarker::MakeMeshControl( )
{
  // Initialize variable for marker, control
  visualization_msgs::Marker marker;
  visualization_msgs::InteractiveMarkerControl control;
  // Frame_id is world frame since the tvf frame isn't necessarily published yet
  mesh_marker_.header.frame_id = world_frame_;
  mesh_marker_.header.stamp = ros::Time::now( );
  mesh_marker_.name = vf_frame_ + "IMarker";
  mesh_marker_.description = "";
  mesh_marker_.controls.clear( );
  // Add x control
  control.orientation.w = 0.7071;
  control.orientation.x = 0.7071;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "rotate_x";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  mesh_marker_.controls.push_back( control );
  control.name = "move_x";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mesh_marker_.controls.push_back( control );
  // Add z control
  control.orientation.w = 0.7071;
  control.orientation.x = 0.0;
  control.orientation.y = 0.7071;
  control.orientation.z = 0.0;
  control.name = "rotate_z";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  mesh_marker_.controls.push_back( control );
  control.name = "move_z";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mesh_marker_.controls.push_back( control );
  // Add y control
  control.orientation.w = 0.7071;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.7071;
  control.name = "rotate_y";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  mesh_marker_.controls.push_back( control );
  control.name = "move_y";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  mesh_marker_.controls.push_back( control );
  // Add free form control
  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "move_rotate_3d";
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  mesh_marker_.controls.push_back( control );
  // Pull in appropriate mesh
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://" + package_name_ + folder_path_ +
    vf_frame_ + ".stl";
  // Calculate mesh pose
  marker.header.frame_id = vf_frame_;
  marker.pose = mesh_pose_.pose;
  marker.scale.x = vf_scale_factor_;
  marker.scale.y = vf_scale_factor_;
  marker.scale.z = vf_scale_factor_;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.50;
  // Apparently this section needs to be last
  control.always_visible = true;
  control.markers.push_back( marker );
  mesh_marker_.controls.push_back( control );
  mesh_marker_.pose = tvf_pose_;
  // Add to and update server
  mesh_server_->insert( mesh_marker_ );
  mesh_server_->setCallback( mesh_marker_.name,
                             boost::bind( &TVFGraphIMarker::ProcessMeshFeedback,
                                          this, _1 ) );
  mesh_server_->applyChanges( );
}

void TVFGraphIMarker::NearestNeighborControl( )
{
  // Counter to determine the accessible percentage
  unsigned int accessible = 0, counter = 0;
  double reachable;
  std::vector< double > color;
  ROS_INFO_STREAM( "Updating Neighbors..." );
  // Update neighbor graph
  // Get all neighbors for provided vertex index
  //tvf_graph_.GetAllNeighbors( current_vertex_, neighbor_graph_ );
  // Get k nearest neighbors to provided vertex index
  tvf_graph_.GetNearestNeighbors( current_vertex_, nearest_neighbors_,
                                  neighbor_graph_ );
  // Get neighbors with decreasing likelihood beyond the intralayer distance
  //tvf_graph_.GetScaledNeighbors( current_vertex_, neighbor_graph_ );
  ROS_INFO_STREAM( "Neighbors updated. Calculating IK..." );
  // Iterate through the neighbors and add interactive markers to RViz. Marker
  // color by current vertex, same layer vertices, and different layer vertices
  for ( vs_ = vertices( neighbor_graph_ ); vs_.first != vs_.second; ++vs_.first )
  {
    if ( *vs_.first == 0 )
    {
      color = white_;
    }
    else if ( neighbor_graph_[ 0 ].layer ==
              neighbor_graph_[ *vs_.first ].layer )
    {
      reachable = CheckCollisions( *vs_.first, neighbor_graph_ );
      // Calculate color
      color = { 1.0 - reachable, reachable, 0.0 };
      if ( reachable > 0.5 )
      {
        ++accessible;
      }
      counter ++;
    }
    else
    {
      // If the vertex is in a different layer go ahead and add it.
      color = blue_;
    }
    // Add an interactive marker to the visual scene
    TVFIMarker( *vs_.first, color, neighbor_graph_ );
  }
  accessible_percentage_ = 100.0 * ( double )accessible / counter;
  reachable_string_ = "Reachable Poses: " + std::to_string( accessible ) +
    " of " + std::to_string( counter );
}

double TVFGraphIMarker::CheckCollisions( const unsigned int index,
  tvf_graph::TVFGraph::UndirectedGraph &graph )
{
  // Planning parameters
  speed_ = 0.90;
  is_cart_ = false;
  unsigned int counter = 0;
  double z_rotation;
  tf::Quaternion q_orig, q_rot, q_new;
  tf::StampedTransform transform;
  geometry_msgs::PoseStamped pose_tvf, pose_world;
  Eigen::Affine3d int_marker_pose_affine;
  // Fill in pose stamped
  pose_tvf.header.frame_id =
    graph[ index ].pose_stamped.header.frame_id;
  pose_tvf.pose.position.x = graph[ index ].pose_stamped.pose.position.x *
    vf_scale_factor_ + mesh_pose_.pose.position.x;
  pose_tvf.pose.position.y = graph[ index ].pose_stamped.pose.position.y *
    vf_scale_factor_ + mesh_pose_.pose.position.y;
  pose_tvf.pose.position.z = graph[ index ].pose_stamped.pose.position.z *
    vf_scale_factor_ + mesh_pose_.pose.position.z;
  // Test varied orientations around the normal
  for ( unsigned int i = 0; i <= rotational_resolution_; ++i )
  {
    if ( i == 0 )
    {
      pose_tvf.pose.orientation =
        graph[ index ].pose_stamped.pose.orientation;
    }
    else
    {
      z_rotation = 6.28318 * ( i  / ( double )rotational_resolution_ );
      q_rot = tf::createQuaternionFromRPY( 0.0, 0.0, z_rotation );
      quaternionMsgToTF( graph[ index ].pose_stamped.pose.orientation,
                         q_orig );
      q_new = q_rot * q_orig;  // Calculate the new orientation
      q_new.normalize( );
      quaternionTFToMsg( q_new, pose_tvf.pose.orientation );
    }
    // Check for appropriate transform
    try
    {
      listener_.waitForTransform( "/" + vf_frame_, world_frame_, ros::Time( 0 ),
                                  ros::Duration( 3.0 ) );
      listener_.lookupTransform( "/" + vf_frame_, world_frame_, ros::Time( 0 ),
                                 transform );
      listener_.transformPose( world_frame_, pose_tvf, pose_world );
      tf::poseMsgToEigen( pose_world.pose, int_marker_pose_affine );
    }
    catch ( tf::TransformException &ex )
    {
      ROS_ERROR( "%s", ex.what( ) );
    }
    unsigned thread_id = omp_get_thread_num( );
    constraint_fn_ = boost::bind( &TVFGraphIMarker::IsStateValid, this,
                                  &planning_scene_, _1, _2, _3 );
    /*
    constraint_fn_ = boost::bind( &TVFGraphIMarker::IsStateValidMulti, this,
                                  planning_scene_vector_[ thread_id ],
                                  _1, _2, _3 );
    */
    // Check planning scene
    if ( kinematic_state_->setFromIK( joint_model_group_,
                                      int_marker_pose_affine,
                                      robot_eef_, 2, 0.1, constraint_fn_ ) )
    {
      ++counter;
    }
  }
  if ( rotational_resolution_ == 0 )
  {
    return counter;
  }
  else
  {
    return ( ( double )counter / rotational_resolution_ );
  }
}

bool TVFGraphIMarker::IsStateValid(
    planning_scene::PlanningScene *planning_scene,
    moveit::core::RobotState *robot_state,
    const moveit::core::JointModelGroup *group,
    const double *ik_solution )
{
  robot_state->setJointGroupPositions( group, ik_solution );
  robot_state->update( );

  collision_detection::AllowedCollisionMatrix acm =
    planning_scene->getAllowedCollisionMatrix( );
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  req.verbose = false;
  req.group_name = group->getName( );
  res.clear( );

  planning_scene_monitor_ptr_->requestPlanningSceneState( );
  planning_scene_monitor::LockedPlanningSceneRW
    lps( planning_scene_monitor_ptr_ );
  lps->setCurrentState( *robot_state );
  lps->checkCollision( req, res, *robot_state, acm );
  return !res.collision;
}

bool TVFGraphIMarker::IsStateValidMulti(
    planning_scene::PlanningScenePtr planning_scene,
    moveit::core::RobotState *robot_state,
    const moveit::core::JointModelGroup *group,
    const double *ik_solution )
{
  /*
  unsigned thread_id = omp_get_thread_num( );
  std::cout << "Checking valid state on thread: " << thread_id << std::endl;
  */

  robot_state->setJointGroupPositions( group, ik_solution );
  robot_state->update( );

  collision_detection::AllowedCollisionMatrix acm =
    planning_scene->getAllowedCollisionMatrix( );
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  req.verbose = false;
  req.group_name = group->getName( );
  res.clear( );

  /*
  planning_scene_vector_[ thread_id ]->setCurrentState( *robot_state );
  planning_scene_vector_[ thread_id ]->checkCollision( req, res,
                                                       *robot_state, acm );
  */

  planning_scene_monitor_ptr_->requestPlanningSceneState( );
  planning_scene_monitor::LockedPlanningSceneRW lps( planning_scene_monitor_ptr_ );
  lps->setCurrentState( *robot_state );
  lps->checkCollision( req, res, *robot_state, acm );
  /*
  std::cout << "Done checking valid state on thread: " << thread_id
            << std::endl;
  */
  return !res.collision;
}

void TVFGraphIMarker::LoadCollisionMeshes( )
{
  // Temporary pose and transform for mesh adjustment
  geometry_msgs::Pose temp_pose;
  geometry_msgs::PoseStamped mesh_world;
  tf::StampedTransform transform;
  // Create a collision object for a collision mesh
  moveit_msgs::CollisionObject collision_mesh;
  // Frame_id should be mesh name but id could be numeric
  collision_mesh.header.frame_id = world_frame_;
  collision_mesh.header.stamp = ros::Time::now( );
  collision_mesh.id = vf_frame_ + "_mesh";
  // Pull in appropriate mesh
  std::string file_path = "package://" + package_name_ + folder_path_ +
    vf_frame_ + ".stl";
  shapes::Mesh* mesh = shapes::createMeshFromResource( file_path );
  // Convert into proper type for the collision scene
  shape_msgs::Mesh co_mesh;
  shape_msgs::Mesh co_mesh_scaled;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape( mesh, co_mesh_msg );
  co_mesh = boost::get< shape_msgs::Mesh >( co_mesh_msg );
  // Scale the mesh based on ROS params
  for ( unsigned int ii = 0; ii < co_mesh.triangles.size( ) ; ++ii )
  {
    shape_msgs::MeshTriangle triangle;
    triangle.vertex_indices[ 0 ] = co_mesh.triangles[ ii ].vertex_indices[ 0 ];
    triangle.vertex_indices[ 1 ] = co_mesh.triangles[ ii ].vertex_indices[ 1 ];
    triangle.vertex_indices[ 2 ] = co_mesh.triangles[ ii ].vertex_indices[ 2 ];
    co_mesh_scaled.triangles.push_back( triangle );
  }
  for ( unsigned int ii = 0; ii < co_mesh.vertices.size( ) ; ++ii )
  {
    geometry_msgs::Point vertex;
    vertex.x = co_mesh.vertices[ ii ].x * vf_scale_factor_;
    vertex.y = co_mesh.vertices[ ii ].y * vf_scale_factor_;
    vertex.z = co_mesh.vertices[ ii ].z * vf_scale_factor_;
    co_mesh_scaled.vertices.push_back( vertex );
  }
  // Add mesh to collision object meshes
  collision_mesh.meshes.push_back( co_mesh_scaled );
  // Check for appropriate transform
  try
  {
    listener_.waitForTransform( "/" + vf_frame_, world_frame_, ros::Time( 0 ),
                                ros::Duration( 3.0 ) );
    listener_.lookupTransform( "/" + vf_frame_, world_frame_, ros::Time( 0 ),
                               transform );
    listener_.transformPose( world_frame_, mesh_pose_, mesh_world );
  }
  catch ( tf::TransformException &ex )
  {
    ROS_ERROR( "%s", ex.what( ) );
  }
  // Add pose to collision object meshes at TVF frame
  collision_mesh.mesh_poses.push_back( mesh_world.pose );
  // Specify as a mesh to be added
  collision_mesh.operation = moveit_msgs::CollisionObject::ADD;
  // Add to the world collision scene
  moveit_scene_.world.collision_objects.push_back( collision_mesh );
  moveit_scene_.is_diff = true;
  // Publish to collision scene
  moveit_scene_pub_.publish( moveit_scene_ );
}

void TVFGraphIMarker::TVFIMarker( const unsigned int index,
                                  std::vector< double > &color,
                                  tvf_graph::TVFGraph::UndirectedGraph &graph )
{
  // Pull pose stamped from the graph structure
  geometry_msgs::PoseStamped pose_stamped = graph[ index ].pose_stamped;
  // Create interactive marker
  visualization_msgs::InteractiveMarker int_marker;
  // Name the marker using it index to ensure proper linking to its neighbors
  int_marker.name = std::to_string( graph[ index ].id );
  tvf_server_->erase( int_marker.name );
  // Fill in pose stamped information with a scaling factor the scale up or
  // down the entire virtual fixture
  int_marker.header.frame_id = pose_stamped.header.frame_id;
  int_marker.pose.position.x = pose_stamped.pose.position.x *
    vf_scale_factor_ + mesh_pose_.pose.position.x;
  int_marker.pose.position.y = pose_stamped.pose.position.y *
    vf_scale_factor_ + mesh_pose_.pose.position.y;
  int_marker.pose.position.z = pose_stamped.pose.position.z *
    vf_scale_factor_ + mesh_pose_.pose.position.z;
  int_marker.pose.orientation = pose_stamped.pose.orientation;
  //int_marker.description = int_marker.name;
  int_marker.scale = 0.05;
  // Create rotational controller for interactive marker
  visualization_msgs::InteractiveMarkerControl control;
  control.name = "rotational control";
  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.always_visible = true;
  // Create visual marker for interactive marker
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  // Scale the marker dimensions based on a class scaling factor
  marker.scale.x = 0.20 * marker_scale_factor_;
  marker.scale.y = 0.10 * marker_scale_factor_;
  marker.scale.z = 0.05 * marker_scale_factor_;
  marker.color.a = 0.50;
  // Color the marker based on layer differences with the current pose
  marker.color.r = color[ 0 ];
  marker.color.g = color[ 1 ];
  marker.color.b = color[ 2 ];
  // Add visual marker to control and the control to interactive marker
  control.markers.push_back( marker );
  int_marker.controls.push_back( control );
  // Add additional menu controls to interactive marker
  control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::BUTTON;
  int_marker.controls.push_back( control );
  // Update server with marker, the callback function for the marker
  tvf_server_->insert ( int_marker );
  tvf_server_->setCallback( int_marker.name,
                            boost::bind( &TVFGraphIMarker::ProcessFeedback,
                                         this, _1 ) );
  // Update menu handler and apply changes to the server
  menu_handler_.apply( *tvf_server_, int_marker.name );
}

void TVFGraphIMarker::ProcessFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  unsigned int vertex, index;
  // Use this menu to move to pose or add pose to path plan
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      // Get index of vertex of manipulated marker
      vertex = std::stoi( feedback->marker_name );
      if ( feedback->menu_entry_id == 1 )
      {
        ROS_INFO_STREAM( "Updating current pose to graph vertex: " <<
                         std::to_string( vertex ) );
        current_vertex_ = vertex;
        UpdateTVF( );
      }
      else if ( feedback->menu_entry_id == 2 )
      {
        // Loop through neighbor graph to get neighbor index for TVF index
        for ( vs_ = vertices( neighbor_graph_ ); vs_.first != vs_.second;
              ++vs_.first )
        {
          if ( neighbor_graph_[ *vs_.first ].id == vertex )
          {
            index = *vs_.first;
          }
        }
        // Update the orientation of the manipulated graph marker,
        // feedback pose is in TVF frame
        UpdateVertexPose( vertex, index, feedback->pose );
      }
      else if ( feedback->menu_entry_id == 3 )
      {
        // Loop through neighbor graph to get neighbor index for TVF index
        for ( vs_ = vertices( neighbor_graph_ ); vs_.first != vs_.second;
              ++vs_.first )
        {
          if ( neighbor_graph_[ *vs_.first ].id == vertex )
          {
            index = *vs_.first;
          }
        }
        std:cout << "Vertex: " << vertex << " Index: " << index << std::endl;
        // Move EEF to pose of the manipulated graph marker
        MoveToPose( index );
      }
      else if ( feedback->menu_entry_id == 4 )
      {
        // Add manipulated graph marker to path vector
        AddVertexToPath( vertex );
      }
      else if ( feedback->menu_entry_id == 5 )
      {
        // Remove manipulated graph marker from path vector
        RemoveVertexFromPath( vertex );
      }
      else if ( feedback->menu_entry_id == 6 )
      {
        // Check path condition numbers
        CheckConditionNumber( );
        // Send path poses to Descartes server
        //PlanWithDescartes( );
      }
      else if ( feedback->menu_entry_id == 7 )
      {
        // Empty the path vector
        ClearPathVertices( );
      }
      else
      {
        ROS_ERROR_STREAM( "Menu item not found: " << feedback->menu_entry_id );
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      break;
  }
  // Apply changes to the interactive marker server
  tvf_server_->applyChanges( );
}

void TVFGraphIMarker::UpdateVertexPose( const unsigned int vertex,
                                        const unsigned int index,
                                        geometry_msgs::Pose pose )
{
  ROS_INFO_STREAM( "Updating pose for TVF graph vertex: " <<
                   std::to_string( vertex ) << ", neighbor graph vertex: " <<
                   std::to_string( index ) );
  // Overwrite the graph pose stamped entry with the operator's updated one
  tvf_graph_.UpdateVertex( vertex, index, pose, neighbor_graph_ );
}

void TVFGraphIMarker::MoveToPose( const unsigned int index )
{
  geometry_msgs::PoseStamped pose_stamped =
    neighbor_graph_[ index ].pose_stamped;
  pose_stamped.header.seq = 1;
  pose_stamped.header.stamp = ros::Time::now( );
  // Pull the pose information from the vf graph and scale appropriately
  pose_stamped.pose.position.x =
    neighbor_graph_[ index ].pose_stamped.pose.position.x * vf_scale_factor_ +
    mesh_pose_.pose.position.x;
  pose_stamped.pose.position.y =
    neighbor_graph_[ index ].pose_stamped.pose.position.y * vf_scale_factor_ +
    mesh_pose_.pose.position.y;
  pose_stamped.pose.position.z =
    neighbor_graph_[ index ].pose_stamped.pose.position.z * vf_scale_factor_ +
    mesh_pose_.pose.position.z;
  // Set class variables
  pose_stamped_ = pose_stamped;
  speed_ = 0.90;
  is_cart_ = false;
  time_to_move_ = true;
}

void TVFGraphIMarker::AddVertexToPath( const unsigned int index )
{
  // Add vertex to graph, pull edges from TVF, and update path markers
  tvf_graph_.AddVertex( index, path_graph_ );
  tvf_graph_.AddPathEdges( index, path_graph_ );
  UpdatePathMarkers( );
}

void TVFGraphIMarker::RemoveVertexFromPath( const unsigned int index )
{
  double reachable = 0.0;
  bool success = false, found = false;
  std::vector< double > color;
  // Loop through neighbor graph to get neighbor index for TVF index
  for ( vs_ = vertices( path_graph_ ); vs_.first != vs_.second; ++vs_.first )
  {
    if ( path_graph_[ *vs_.first ].id == index )
    {
      found = true;
      if ( path_graph_[ *vs_.first ].layer == neighbor_graph_[ 0 ].layer )
      {
        reachable = CheckCollisions( *vs_.first, path_graph_ );
        // Calculate color
        color = { 1.0 - reachable, reachable, 0.0 };
      }
      else
      {
        color = blue_;
      }
      // Update visual scene interactive marker
      TVFIMarker( *vs_.first, color, path_graph_ );
      success = tvf_graph_.RemoveVertex( index, path_graph_ );
      UpdatePathMarkers( );
      ROS_INFO_STREAM( "Removing graph vertex: " << std::to_string( index ) <<
                         " from path vertices. Path vertices: " <<
                         boost::num_vertices( path_graph_ ) );
      break;
    }
  }
  if ( !found )
  {
    ROS_ERROR_STREAM( "Selected graph vertex " << std::to_string( index ) <<
                      " is not present in the path graph." );
  }
}

geometry_msgs::PoseArray TVFGraphIMarker::PathVerticesToPoseArray( )
{
  ROS_INFO_STREAM( "Path vertices: " << boost::num_vertices( path_graph_ ) );
  geometry_msgs::PoseArray pose_array;
  // Fill in the pose stamped header
  pose_array.header.stamp = ros::Time::now( );
  pose_array.header.frame_id = vf_frame_;
  // Iterate through the list of indexes
  for ( vs_ = vertices( path_graph_ ); vs_.first != vs_.second; ++vs_.first )
  {
    geometry_msgs::Pose pose;
    // Pull the pose information from the vf graph and scale appropriately
    pose.position.x = path_graph_[ *vs_.first ].pose_stamped.pose.position.x *
      vf_scale_factor_ + mesh_pose_.pose.position.x;
    pose.position.y = path_graph_[ *vs_.first ].pose_stamped.pose.position.y *
      vf_scale_factor_ + mesh_pose_.pose.position.y;
    pose.position.z = path_graph_[ *vs_.first ].pose_stamped.pose.position.z *
      vf_scale_factor_ + mesh_pose_.pose.position.z;
    pose.orientation = path_graph_[ *vs_.first ].pose_stamped.pose.orientation;
    // Add it to the pose array
    pose_array.poses.push_back( pose );
  }
  return pose_array;
}

void TVFGraphIMarker::ClearPathVertices( )
{
  double reachable = 0.0;
  std::vector< double > color;
  // Iterate through all vertices and delete them
  for ( int i = boost::num_vertices( path_graph_ ) - 1; i >= 0; --i )
  {
    if ( neighbor_graph_[ 0 ].id == path_graph_[ i ].id )
    {
      color = white_;
    }
    else if ( neighbor_graph_[ 0 ].layer == path_graph_[ i ].layer )
    {
      // Check collisions and calculate color
      reachable = CheckCollisions( i, path_graph_ );
      color = { 1.0 - reachable, reachable, 0.0 };
    }
    else
    {
      color = blue_;
    }
    // Update visual scene interactive marker
    TVFIMarker( i, color, path_graph_ );
    // Remove vertex from graph
    tvf_graph_.RemoveVertex( path_graph_[ i ].id, path_graph_ );
  }
  // Once all vertices have been removed update path markers
  UpdatePathMarkers( );
  ROS_INFO_STREAM( "Cleared path vertices. Path vertices: " <<
                   boost::num_vertices( path_graph_ ) );
}

void TVFGraphIMarker::PlanWithDescartes( )
{
  // Send path to Descartes sever for execution
  ac_.waitForServer( );
  tvf_nav_descartes::TVFNavDescartesGoal goal;
  // Build pose array from vector of graph vertex ids
  goal.pose_array = PathVerticesToPoseArray( );
  ac_.sendGoal( goal );
  ac_.waitForResult( );
  // Get failed points from planning
  tvf_nav_descartes::TVFNavDescartesResult failed_points = *ac_.getResult( );
  // Sort the path to determine the number of segments
  std::sort( failed_points.result.begin( ), failed_points.result.end( ) );
  // Adjust path markers
  if( std::find( failed_points.result.begin( ),
                 failed_points.result.end( ), -1 ) !=
      failed_points.result.end( ) )
  {
    ROS_INFO_STREAM( "Path test completed successfully." );
  }
  else
  {
    ROS_INFO_STREAM( "Path test complete, " << failed_points.result.size( ) <<
                     " points failed." );
    for ( unsigned int i = 0; i < failed_points.result.size( ); i++ )
    {
      TVFIMarker( failed_points.result[ i ], black_, path_graph_ );
      ROS_INFO_STREAM( "Path test point: " << failed_points.result[ i ] <<
                       " failed." );

    }
  }
 }

void TVFGraphIMarker::CheckConditionNumber( )
{
  double roll_1, pitch_1, yaw_1, roll_2, pitch_2, yaw_2, condition_number;
  std::vector< double > joint_positions, color;
  std::vector< std::string > joint_names;
  geometry_msgs::PoseArray pose_array = PathVerticesToPoseArray( );
  sensor_msgs::JointStateConstPtr joint_state;
  tf::Quaternion quat_1, quat_2;
  Eigen::VectorXd cart_delta( 6 );
  Eigen::MatrixXd J_1, J_pseudo, J_2, joint_delta;
  // Check condition number for all points in path
  for ( unsigned int i = 1; i < pose_array.poses.size( ); i++ )
  {
    // Calculate the Cartesian change
    cart_delta( 0 ) = pose_array.poses[ i ].position.x -
                      pose_array.poses[ i - 1 ].position.x;
    cart_delta( 1 ) = pose_array.poses[ i ].position.y -
                      pose_array.poses[ i - 1 ].position.y;
    cart_delta( 2 ) = pose_array.poses[ i ].position.z -
                      pose_array.poses[ i - 1 ].position.z;
    // Convert to RPY
    tf::quaternionMsgToTF( pose_array.poses[ i ].orientation, quat_1 );
    tf::quaternionMsgToTF( pose_array.poses[ i ].orientation, quat_2 );
    tf::Matrix3x3( quat_1 ).getRPY( roll_1, pitch_1, yaw_1 );
    tf::Matrix3x3( quat_2 ).getRPY( roll_2, pitch_2, yaw_2 );
    // Calculate the RPY change
    cart_delta( 3 ) = roll_1 - roll_2;
    cart_delta( 4 ) = pitch_1 - pitch_2;
    cart_delta( 5 ) = yaw_1 - yaw_2;
    // Update Jacobian
    J_1 = kinematic_state_->getJacobian( joint_model_group_ );
    joint_delta =
      J_1.transpose( ) * ( J_1 * J_1.transpose( ) ).inverse( ) * cart_delta;
    // Update joint state
    joint_state = ros::topic::waitForMessage< sensor_msgs::JointState >
      ( "/joint_states", ros::Duration( 0.0 ) );
    if ( !joint_state ) throw
      std::runtime_error( "Joint state message capture failed." );
    joint_positions = joint_state->position;
    // Combine previous joints and joint delta
    for ( unsigned int i = 0; i < joint_positions.size( ); ++i )
    {
      joint_positions[ i ] = joint_positions[ i ] + joint_delta( i );
    }
    // Update kinematic state and
    kinematic_state_->setVariablePositions( joint_positions );
    kinematic_state_->update( );
    // Update Jacobian
    J_2 = kinematic_state_->getJacobian( joint_model_group_ );
    // Calculate the condition number
    Eigen::JacobiSVD< Eigen::MatrixXd > svd( J_2, Eigen::ComputeThinU );
    svd = Eigen::JacobiSVD< Eigen::MatrixXd > ( J_2 );
    condition_number = svd.singularValues( )( 0 ) /
      svd.singularValues( )( svd.singularValues( ).size( ) - 1 );
    // Add condition number to list
    condition_numbers_.push_back( condition_number );
    // Update color based on condition number
    if ( condition_numbers_[ i ] < 10 )
    {
      color = black_;
    }
    else if ( 10 <= condition_numbers_[ i ] < 100 )
    {
      color = { 0.5, 0.5, 0.5 };
    }
    else if ( 100 <= condition_numbers_[ i ] )
    {
      color = white_;
    }
    // Output condition number and update marker
    ROS_INFO_STREAM( condition_number );
    //TODO: make sure the color corresponds to the right node
    TVFIMarker( i, color, path_graph_ );
  }
}

void TVFGraphIMarker::UpdatePathMarkers( )
{
  std::vector< double > color;
  visualization_msgs::Marker marker;
  // Clear markers
  marker.ns = "arrow";
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish( marker );
  // Add interactive markers with updated colors
  for ( unsigned int i = 0; i < boost::num_vertices( path_graph_ ); i++ )
  {
    color = { 0.0,
              0.3 + 0.7 * i / boost::num_vertices( path_graph_ ),
              0.3 + 0.7 * i / boost::num_vertices( path_graph_ ) };
    TVFIMarker( i, color, path_graph_ );
    if ( i > 0 )
    {
      // Add arrows between path vertices
      AddPathArrow(  i - 1 , i, color );
    }
  }
}

void TVFGraphIMarker::AddPathArrow( const unsigned int start,
                                    const unsigned int end,
                                    std::vector< double > &color )
{
  // Size arrow locations to marker
  geometry_msgs::Point pt_start, pt_end;
  pt_start.x = path_graph_[ start ].pose_stamped.pose.position.x *
    vf_scale_factor_ + mesh_pose_.pose.position.x;
  pt_start.y = path_graph_[ start ].pose_stamped.pose.position.y *
    vf_scale_factor_ + mesh_pose_.pose.position.y;
  pt_start.z = path_graph_[ start ].pose_stamped.pose.position.z *
    vf_scale_factor_ + mesh_pose_.pose.position.z;
  pt_end.x = path_graph_[ end ].pose_stamped.pose.position.x * vf_scale_factor_
    + mesh_pose_.pose.position.x;
  pt_end.y = path_graph_[ end ].pose_stamped.pose.position.y * vf_scale_factor_
    + mesh_pose_.pose.position.y;
  pt_end.z = path_graph_[ end ].pose_stamped.pose.position.z * vf_scale_factor_
    + mesh_pose_.pose.position.z;
  // Create an arrow marker in the vf frame
  visualization_msgs::Marker marker;
  marker.header.frame_id = vf_frame_;
  marker.header.stamp = ros::Time( );
  marker.ns = "arrow";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration( 0 );
  // Assign unique id
  marker.id = start;
  // Clear points and fill start and end of arrow
  marker.points.clear( );
  marker.points.push_back( pt_start );
  marker.points.push_back( pt_end );
  // Scale the arrow
  marker.scale.x = 0.025 * marker_scale_factor_;
  marker.scale.y = 0.05 * marker_scale_factor_;
  marker.scale.z = 0.05 * marker_scale_factor_;
  marker.color.r = color[ 0 ];
  marker.color.g = color[ 1 ];
  marker.color.b = color[ 2 ];
  marker.color.a = 0.50;
  // Publish the markers
  marker_pub_.publish( marker );
}

void TVFGraphIMarker::ProcessMeshFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      // Update the chosen pose
      chosen_pose_.header.frame_id = world_frame_;
      chosen_pose_.pose = feedback->pose;
      // Update the mesh marker
      MakeMeshControl( );
      // Reload the collision mesh in the proper location
      LoadCollisionMeshes( );
      // Update with the current vertex in the graph
      UpdateTVF( );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      // Update TVF world frame pose
      tvf_pose_ = feedback->pose;
      // Update TVF world frame transform
      tvf_transform_.setOrigin( tf::Vector3( feedback->pose.position.x,
                                             feedback->pose.position.y,
                                             feedback->pose.position.z ) );
      tvf_transform_.setRotation( tf::Quaternion( feedback->pose.orientation.x,
                                                  feedback->pose.orientation.y,
                                                  feedback->pose.orientation.z,
                                                  feedback->pose.orientation.w )
                                );
      break;
  }
  // Apply changes to the interactive marker server
  mesh_server_->applyChanges( );
}

void TVFGraphIMarker::FrameCallback( const ros::TimerEvent& )
{
  // Broadcast transform
  static tf::TransformBroadcaster br;
  br.sendTransform( tf::StampedTransform( tvf_transform_, ros::Time::now( ),
                                          world_frame_, "/" + vf_frame_ ) );
}

}