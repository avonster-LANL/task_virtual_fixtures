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

#include <tvf_nav_descartes/tvf_nav_descartes.h>

namespace tvf_nav_descartes
{

TVFNavDescartes::TVFNavDescartes( std::string name, std::string robot_name ):
 action_server_( node_handle_, name,
                 boost::bind( &TVFNavDescartes::executeCB, this, _1 ), false ),
 group_ ( moveit::planning_interface::MoveGroupInterface( robot_name ) ),
 model_ ( new descartes_moveit::MoveitStateAdapter )
{
  action_server_.start();
  marker_pub_ =
    node_handle_.advertise<visualization_msgs::Marker>( "descartes_markers", 1000 );
  marker_array_pub_ =
    node_handle_.advertise< visualization_msgs::MarkerArray >( "descartes_array_markers", 1000 );
  // Fill in input class variables
  robot_name_ = robot_name;
  if ( !ros::param::get( "world", world_frame_) )
  {
    ROS_ERROR_STREAM( "Failed to get param 'world'" );
  }
  node_handle_.getParam( "/controller_joint_names", joint_names_ );
  InitModel( );
  ROS_INFO_STREAM( "TVF_nav_descartes Initialized." );
}

int TVFNavDescartes::InitModel( )
{
  // Descartes robot model variables
  const std::string robot_description = "robot_description";
  const std::string tcp_frame = "eef";
  // Fill and initialize Descartes robot model
  if ( !model_->initialize( robot_description, robot_name_,
                            world_frame_, tcp_frame ) )
  {
    ROS_INFO( "Could not initialize robot model" );
    return -1;
  }
  // Let's turn on collision checking.
  model_->setCheckCollisions( true );
  // Initialize Descartes planner
  if ( !planner_.initialize( model_ ) )
  {
    ROS_ERROR( "Failed to initialize planner" );
    return -2;
  }
  // Create MoveInterface
  mi_ = new MoveInterface( );
  mi_->initialize( robot_name_ );
  mi_->setPlannerId( "RRTConnectkConfigDefault" );
  return 0;
}

void TVFNavDescartes::publishPosesMarkers(
  const EigenSTL::vector_Affine3d& poses )
{
  float axis_line_width = 0.02, axis_line_length = 0.02;
  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;
  z_axes.type = y_axes.type = x_axes.type =
    visualization_msgs::Marker::LINE_LIST;
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";
  z_axes.action = y_axes.action = x_axes.action =
    visualization_msgs::Marker::ADD;
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
  z_axes.header.frame_id = y_axes.header.frame_id =
    x_axes.header.frame_id = world_frame_;
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = axis_line_width;

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";
  line.action = visualization_msgs::Marker::ADD;
  line.lifetime = ros::Duration( 0 );
  line.header.frame_id = world_frame_;
  line.scale.x = axis_line_width;
  line.id = 0;
  line.color.r = 1;
  line.color.g = 1;
  line.color.b = 0;
  line.color.a = 1;

  // creating axes markers
  z_axes.points.reserve( 2 * poses.size( ) );
  y_axes.points.reserve( 2 * poses.size( ) );
  x_axes.points.reserve( 2 * poses.size( ) );
  line.points.reserve( poses.size( ) );
  x_axes.pose.orientation.w = 1;
  y_axes.pose.orientation.w = 1;
  z_axes.pose.orientation.w = 1;
  line.pose.orientation.w = 1;
  geometry_msgs::Point p_start,p_end;
  double distance = 0;
  Eigen::Affine3d prev = poses[ 0 ];
  for( unsigned int i = 0; i < poses.size( ); i++ )
  {
    const Eigen::Affine3d& pose = poses[i];
    distance = (pose.translation() - prev.translation()).norm();

    tf::pointEigenToMsg(pose.translation(),p_start);

    if(distance > 0.01)
    {
      Eigen::Affine3d moved_along_x =
        pose * Eigen::Translation3d( axis_line_length, 0, 0 );
      tf::pointEigenToMsg( moved_along_x.translation( ), p_end );
      x_axes.points.push_back( p_start );
      x_axes.points.push_back( p_end );

      Eigen::Affine3d moved_along_y =
        pose * Eigen::Translation3d( 0, axis_line_length, 0 );
      tf::pointEigenToMsg( moved_along_y.translation( ), p_end );
      y_axes.points.push_back( p_start );
      y_axes.points.push_back( p_end );

      Eigen::Affine3d moved_along_z =
        pose * Eigen::Translation3d( 0, 0, axis_line_length );
      tf::pointEigenToMsg( moved_along_z.translation( ),p_end );
      z_axes.points.push_back( p_start );
      z_axes.points.push_back( p_end );

      // saving previous
      prev = pose;
    }
    line.points.push_back( p_start );
  }

  markers_msg.markers.push_back( x_axes );
  markers_msg.markers.push_back( y_axes );
  markers_msg.markers.push_back( z_axes );
  markers_msg.markers.push_back( line );

  while( marker_array_pub_.getNumSubscribers( ) < 1 )
  {
    ros::Duration( 1.0 ).sleep( );
    ROS_INFO_THROTTLE( 5, "Displaying Descartes path markers." );
  }
  marker_array_pub_.publish( markers_msg );
  ros::spinOnce( );
  ros::Duration( 1.0 ).sleep( );
}

void TVFNavDescartes::clearPosesMarkers(
  const EigenSTL::vector_Affine3d& poses )
{
  float axis_line_width = 0.02, axis_line_length = 0.02;
  // creating rviz markers
  visualization_msgs::Marker z_axes, y_axes, x_axes, line;
  visualization_msgs::MarkerArray markers_msg;
  z_axes.type = y_axes.type = x_axes.type =
    visualization_msgs::Marker::LINE_LIST;
  z_axes.ns = y_axes.ns = x_axes.ns = "axes";
  z_axes.action = y_axes.action = x_axes.action =
    visualization_msgs::Marker::ADD;
  z_axes.lifetime = y_axes.lifetime = x_axes.lifetime = ros::Duration(0);
  z_axes.header.frame_id = y_axes.header.frame_id =
    x_axes.header.frame_id = world_frame_;
  z_axes.scale.x = y_axes.scale.x = x_axes.scale.x = axis_line_width;

  // z properties
  z_axes.id = 0;
  z_axes.color.r = 0;
  z_axes.color.g = 0;
  z_axes.color.b = 1;
  z_axes.color.a = 1;

  // y properties
  y_axes.id = 1;
  y_axes.color.r = 0;
  y_axes.color.g = 1;
  y_axes.color.b = 0;
  y_axes.color.a = 1;

  // x properties
  x_axes.id = 2;
  x_axes.color.r = 1;
  x_axes.color.g = 0;
  x_axes.color.b = 0;
  x_axes.color.a = 1;

  // line properties
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.ns = "line";
  line.action = visualization_msgs::Marker::DELETE;
  line.lifetime = ros::Duration( 0 );
  line.header.frame_id = world_frame_;
  line.scale.x = axis_line_width;
  line.id = 0;

  markers_msg.markers.push_back( x_axes );
  markers_msg.markers.push_back( y_axes );
  markers_msg.markers.push_back( z_axes );
  markers_msg.markers.push_back( line );
  marker_array_pub_.publish( markers_msg );
  ros::spinOnce( );
  ros::Duration( 1.0 ).sleep( );
}

void TVFNavDescartes::publishPathArrows(
  const EigenSTL::vector_Affine3d& poses )
{
  std::vector< double > color;
  geometry_msgs::Point p_start, p_end;
  // Create an arrow marker in the world frame
  visualization_msgs::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = ros::Time();
  marker.ns = "arrow";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration( 0 );
  for( unsigned int i = 0; i < poses.size( ) - 1; i++ )
  {
    // Assign unique id
    marker.id = i;
    // Convert Affine to point
    tf::pointEigenToMsg( poses[ i ].translation( ), p_start );
    tf::pointEigenToMsg( poses[ i + 1 ].translation( ), p_end );
    // Clear points and fill start and end of arrow
    marker.points.clear( );
    marker.points.push_back( p_start );
    marker.points.push_back( p_end );
    // Scale the arrow
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    // Color the arrow just like the interactive markers
    color = { 0.0,
              0.3 + 0.7 * i / poses.size( ),
              0.3 + 0.7 * i / poses.size( ) };
    marker.color.r = color[ 0 ];
    marker.color.g = color[ 1 ];
    marker.color.b = color[ 2 ];
    marker.color.a = 1.0;
    // Publish the markers
    marker_pub_.publish( marker );
    ros::spinOnce( );
  }
}

void TVFNavDescartes::clearPathArrows(
  const EigenSTL::vector_Affine3d& poses )
{
  std::vector< double > color;
  geometry_msgs::Point p_start, p_end;
  // Create an arrow marker in the world frame
  visualization_msgs::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = ros::Time();
  marker.ns = "arrow";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::DELETE;
  marker.lifetime = ros::Duration( 0 );
  for( unsigned int i = 0; i < poses.size( ) - 1; i++ )
  {
    // Assign unique id
    marker.id = i;
    // Publish the markers
    marker_pub_.publish( marker );
    ros::spinOnce( );
  }
}

std::vector< double > TVFNavDescartes::getCurrentJointState( const std::string& topic )
{
  sensor_msgs::JointStateConstPtr state = ros::topic::waitForMessage<sensor_msgs::JointState>( topic, ros::Duration(0.0) );
  if (!state) throw std::runtime_error("Joint state message capture failed");
  return state->position;
}

/*descartes_core::TrajectoryPtPtr TVFNavDescartes::makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame( pose ) ) );
}

descartes_core::TrajectoryPtPtr TVFNavDescartes::makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt( pose, M_PI / 2.0 - 0.0001, AxialSymmetricPt::X_AXIS ) );
}*/

bool TVFNavDescartes::executeTrajectory(
    const trajectory_msgs::JointTrajectory& trajectory )
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction >
    action_client( "joint_trajectory_action", true );

  if ( !action_client.waitForServer( ros::Duration( 5.0 ) ) )
  {
    ROS_ERROR( "Could not connect to action server" );
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration( 5.0 );

  return action_client.sendGoalAndWait( goal ) ==
    actionlib::SimpleClientGoalState::SUCCEEDED;
}

int TVFNavDescartes::executeCB(
    const tvf_nav_descartes::TVFNavDescartesGoalConstPtr &goal )
{
  // TODO: get axially symmetric points working
  std::vector< int > failed_points = { -1 };
  //ROS_INFO_STREAM( "Pose array: " << goal->pose_array );
  TrajectoryVec descartes_path;
  TrajectoryVec descartes_result;

  descartes_core::Frame tool_base = descartes_core::Frame::Identity( );
  descartes_core::Frame tool_pt = descartes_core::Frame::Identity( );

  // Poses are in the VF frame so convert them to the world frame
  tf::StampedTransform vf_tf;
  if ( listener_.waitForTransform( world_frame_,
                                   goal->pose_array.header.frame_id,
                                   ros::Time( 0 ), ros::Duration( 5.0 ) ))
  {
    listener_.lookupTransform( world_frame_, goal->pose_array.header.frame_id,
                               ros::Time( 0 ), vf_tf );
  }
  else
  {
    ROS_ERROR( "Transform error." );
    action_server_.setSucceeded( result_ );
    return -2;
  }

  // Descartes uses eigen, so let's convert the data type
  Eigen::Affine3d vf;
  tf::transformTFToEigen( vf_tf, vf );
  descartes_core::Frame wobj_base( vf );
  EigenSTL::vector_Affine3d poses;

  for ( unsigned int i = 0; i < goal->pose_array.poses.size( ); i++ )
  {
    Eigen::Affine3d pose;
    tf::poseMsgToEigen( goal->pose_array.poses[ i ], pose );
    poses.push_back( vf * pose );

    // Each 'point' represents a point in the VF where the point's X axis is
    // pointed toward the surface, so X rotation is okay.
    descartes_core::Frame wobj_pt( pose );
    /*
    descartes_trajectory::TolerancedFrame wobj_pt( pose );
    wobj_pt.orientation_tolerance.x_lower = -M_PI;
    wobj_pt.orientation_tolerance.x_upper = M_PI;
    wobj_pt.orientation_tolerance.y_lower = 0.02;
    wobj_pt.orientation_tolerance.y_upper = 0.02;
    wobj_pt.orientation_tolerance.z_lower = 0.02;
    wobj_pt.orientation_tolerance.z_upper = 0.02;
    */

    boost::shared_ptr<descartes_trajectory::CartTrajectoryPt> pt(
      new descartes_trajectory::CartTrajectoryPt(
        wobj_base, wobj_pt, tool_base, tool_pt, // Here we specify our frames
        0, // Don't search in cartesian space. We want the points to be exact.
        5.0 * ( M_PI / 90.0 ), // Do search our rotation in 10 degree increments.
        descartes_core::TimingConstraint( 2.0 ) ) ); // Make every point 0.25 seconds apart

    /*descartes_core::TrajectoryPtPtr pt(
      new descartes_trajectory::AxialSymmetricPt( vf * pose, M_PI / 2.0 - 0.0001,
        descartes_trajectory::AxialSymmetricPt::X_AXIS,
         descartes_core::TimingConstraint( 0.50 )) );*/

    //descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint( pose );

    descartes_path.push_back( pt );
  }
  ROS_INFO_STREAM( "Point array size: " << descartes_path.size( ) );
  //publishPosesMarkers( poses );
  //publishPathArrows( poses );
  ros::Duration( 1.0 ).sleep( );

  result_.result = failed_points;
  model_->setCheckCollisions( true );

  if ( !planner_.planPath( descartes_path ) )
  {
    ROS_ERROR( "Could not solve for a valid path. Getting failed points:" );
    failed_points = planner_.getFailedPoints( );
    result_.result = failed_points;
    action_server_.setSucceeded( result_ );
    return -3;
  }
  else if ( !planner_.getPath( descartes_result ) )
  {
    ROS_ERROR( "Could not retrieve path." );
    action_server_.setSucceeded( result_ );
    return -4;
  }
  else
  {
    ROS_INFO( "Converting to ROSJointTrajectory." );
  }

  // 5. Translate the result into a type that ROS understands
  // Generate a ROS joint trajectory with the result path, robot model, given
  // joint names, a certain time delta between each trajectory point
  trajectory_msgs::JointTrajectory joint_solution;
  joint_solution.joint_names = joint_names_;
  // Define a default velocity. Descartes points without specified timing will
  // use this value to limit the fastest moving joint. This usually effects
  // the first point in your path the most.
  const static double default_joint_vel = 0.5; // rad/s
  if ( !descartes_utilities::toRosJointPoints( *model_, descartes_result,
                                               default_joint_vel,
                                               joint_solution.points))
  {
    ROS_ERROR( "Unable to convert Descartes trajectory to joint points" );
    return -5;
  }

  //TODO:Removed so the system can run in simulation with robot industrial sim
  //TODO:industrial sim and the fake controller publish to the same joint state
  //TODO:topic causing simulation problems.
  ROS_INFO( "Moving to Descartes path starting position." );
  std::vector< double > start_pose = joint_solution.points[ 0 ].positions;
  mi_->moveJoints( start_pose, 1.0 );
  if( mi_->waitForStatus( ) == MoveInterface::STATUS_ERROR )
  {
    ROS_INFO_STREAM("Trajectory execution failed. MoveInterface Error AGAIN Alex");
    action_server_.setSucceeded( result_ );
    return -4;
  }
  else
  {
    ROS_INFO( "Move to Descartes path starting position complete." );
  }

  joint_solution.points[ 0 ].positions = getCurrentJointState( "/joint_states" );

  // 6. Send the ROS trajectory to the robot for execution
  if ( !executeTrajectory( joint_solution ) )
  {
    ROS_ERROR( "Could not execute trajectory!" );
    action_server_.setSucceeded( result_ );
    return -5;
  }
  else
  {
    result_.result = failed_points;
    action_server_.setSucceeded( result_ );
    return 0;
  }
  action_server_.setSucceeded( result_ );
}

} // end namespace tvf_nav_descartes


int main( int argc, char** argv )
{
  // Initialize ROS
  ros::init( argc, argv, "tvf_nav_descartes_server" );
  // Required for communication with moveit components
  ros::AsyncSpinner spinner( 1 );
  spinner.start( );

  std::string robot_name;
  if ( !ros::param::get( "robot_name", robot_name ) )
  {
    ROS_ERROR_STREAM( "Failed to get param 'robot_name'" );
  }
  tvf_nav_descartes::TVFNavDescartes
    tvf_nav_descartes( "tvf_nav_descartes_server", robot_name );
  ros::waitForShutdown( );
  return 0;
}