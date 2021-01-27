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

#include <ar_scene_loader.h>

namespace ar_scene_loader
{

ARSceneLoader::ARSceneLoader( ros::NodeHandle node_handle,
                              boost::shared_ptr< interactive_markers::InteractiveMarkerServer > im_server ):
  server_ ( im_server )
{
  // Check inputs
  node_handle_ = node_handle;
}

void ARSceneLoader::LoadROSParams( )
{
  if ( !ros::param::get( "world", world_frame_) )
  {
    ROS_ERROR_STREAM( "Failed to get param 'world'" );
  }
  // Get AR id parameters
  if ( !ros::param::get( "ids", ids_) )
  {
    ROS_ERROR_STREAM( "Failed to get param 'ids'" );
  }
  for ( unsigned int i = 0; i < ids_.size( ); ++i )
  {
    std::string xyz_string = "/ar_" + std::to_string( ids_[ i ] ) + "_xyzs";
    std::string rpy_string = "/ar_" + std::to_string( ids_[ i ] ) + "_rpys";
    std::vector< double > xyz;
    std::vector< double > rpy;
    if ( !ros::param::get( xyz_string, xyz ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << xyz_string );
    }
    else
    {
      xyzs_.push_back( xyz );
    }
    if ( !ros::param::get( rpy_string, rpy ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << rpy_string );
    }
    else
    {
      rpys_.push_back( rpy );
    }
    // ROS Parameter variables

    std::string mesh;
    std::string type;
    double vf_scale_factor;
    double marker_scale_factor;
    std::vector< double > frame_translation;
    std::vector< double > frame_rotation;
    std::vector< double > mesh_translation;
    std::vector< double > mesh_rotation;
    // AR tag string
    std::string ar_str = "/ar_" + std::to_string( ids_[ i ] ) + "/";
    // ROS Parameter names
    std::string package_name_str = "/ar_" + std::to_string( ids_[ i ] ) + "/package_name";
    std::string folder_path_str = "/ar_" + std::to_string( ids_[ i ] ) + "/folder_path";
    std::string mesh_str = "name";
    std::string type_str = "type";
    std::string vf_scale_factor_str = "vf_scale_factor";
    std::string marker_scale_factor_str = "marker_scale_factor";
    std::string frame_translation_str = "frame_translation";
    std::string frame_rotation_str = "frame_rotation";
    std::string mesh_translation_str = "mesh_translation";
    std::string mesh_rotation_str = "mesh_rotation";
    // ROS Parameter strings
    std::string mesh_param_str = ar_str + mesh_str;
    std::string type_param_str = ar_str + type_str;
    std::string vf_scale_factor_param_str = ar_str + vf_scale_factor_str;
    std::string marker_scale_factor_param_str = ar_str + marker_scale_factor_str;
    std::string frame_translation_param_str = ar_str + frame_translation_str;
    std::string frame_rotation_param_str = ar_str + frame_rotation_str;
    std::string mesh_translation_param_str = ar_str + mesh_translation_str;
    std::string mesh_rotation_param_str = ar_str + mesh_rotation_str;
    // Add ROS parameter values to vectors
    if ( !ros::param::get( package_name_str, package_name_ ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << package_name_str );
    }
    if ( !ros::param::get( folder_path_str, folder_path_ ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << folder_path_str );
    }
    if ( !ros::param::get( mesh_param_str, mesh ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << mesh_param_str );
    }
    else
    {
      mesh_names_.push_back( mesh );
    }
    if ( !ros::param::get( type_param_str, type ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << type_param_str );
    }
    else
    {
      types_.push_back( type );
    }
    if ( !ros::param::get( vf_scale_factor_param_str, vf_scale_factor ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << vf_scale_factor_param_str );
    }
    else
    {
      vf_scale_factors_.push_back( vf_scale_factor );
    }
    if ( !ros::param::get( marker_scale_factor_param_str, marker_scale_factor ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << marker_scale_factor_param_str );
    }
    else
    {
      marker_scale_factors_.push_back( marker_scale_factor );
    }
    if ( !ros::param::get( frame_translation_param_str, frame_translation ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << frame_translation_param_str );
    }
    else
    {
      frame_translations_.push_back( frame_translation );
    }
    if ( !ros::param::get( frame_rotation_param_str, frame_rotation ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << frame_rotation_param_str);
    }
    else
    {
      frame_rotations_.push_back( frame_rotation );
    }
    if ( !ros::param::get( mesh_translation_param_str, mesh_translation ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << mesh_translation_param_str);
    }
    else
    {
      mesh_translations_.push_back( mesh_translation );
    }
    if ( !ros::param::get( mesh_rotation_param_str, mesh_rotation ) )
    {
      ROS_ERROR_STREAM( "Failed to get param " << mesh_rotation_param_str);
    }
    else
    {
      mesh_rotations_.push_back( mesh_rotation );
    }
  }
}

void ARSceneLoader::FrameCallback( const ros::TimerEvent& )
{
  // Broadcast all frames
  for ( unsigned int i = 0; i < ids_.size( ); ++i )
  {
    tf::Quaternion quaternion =
      tf::createQuaternionFromRPY( rpys_[ i ][ 0 ],
                                   rpys_[ i ][ 1 ],
                                   rpys_[ i ][ 2 ] );
    tf::Transform t;
    t.setOrigin( tf::Vector3( xyzs_[ i ][ 0 ],
                              xyzs_[ i ][ 1 ],
                              xyzs_[ i ][ 2 ] ) );
    t.setRotation( quaternion );
    // Broadcast transform
    static tf::TransformBroadcaster br;
    br.sendTransform( tf::StampedTransform( t, ros::Time::now( ),
                      world_frame_, "/" + mesh_names_[ i ] ) );
  }
}

void ARSceneLoader::Make6DofMarker( bool fixed, bool show_6dof,
                                    unsigned int interaction_mode,
                                    unsigned int i )
{
  visualization_msgs::InteractiveMarker int_marker;
  // Frame_id should be mesh name but id could be numeric
  int_marker.header.frame_id = "/" + mesh_names_[ i ];
  int_marker.header.stamp = ros::Time::now( );
  int_marker.name = mesh_names_[ i ];
  int_marker.scale = 1;
  int_marker.description = "VF 6-DOF Control";
  // Add pose to collision object meshes
  geometry_msgs::Pose pose;
  pose.position.x = xyzs_[ i ][ 0 ] + mesh_translations_[ i ][ 0 ];
  pose.position.y = xyzs_[ i ][ 1 ] + mesh_translations_[ i ][ 1 ];
  pose.position.z = xyzs_[ i ][ 2 ] + mesh_translations_[ i ][ 2 ];
  tf::Quaternion quaternion =
      tf::createQuaternionFromRPY( mesh_rotations_[ i ][ 0 ] + rpys_[ i ][ 0 ],
                                   mesh_rotations_[ i ][ 1 ] + rpys_[ i ][ 1 ],
                                   mesh_rotations_[ i ][ 2 ] + rpys_[ i ][ 2 ] );
  // Copy over quaternion data
  pose.orientation.x = quaternion.x( );
  pose.orientation.y = quaternion.y( );
  pose.orientation.z = quaternion.z( );
  pose.orientation.w = quaternion.w( );

  // Build mesh control
  MakeMeshControl( int_marker, i );
  int_marker.controls[ 0 ].interaction_mode = interaction_mode;

  visualization_msgs::InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }

  if ( interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE )
  {
      std::string mode_text;
      if ( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if ( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if ( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if( show_6dof )
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server_->insert( int_marker );
  server_->setCallback( int_marker.name,
                        boost::bind( &ARSceneLoader::ProcessFeedback,
                                     this, _1 ));
}

visualization_msgs::Marker
  ARSceneLoader::MakeMeshMarker( visualization_msgs::InteractiveMarker &msg,
                                 unsigned int i )
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  // Pull in appropriate mesh
  marker.mesh_resource = "package://" + package_name_ + folder_path_ +
    mesh_names_[ i ] + ".stl";;
  // Scale mesh
  marker.scale.x = vf_scale_factors_[ i ];
  marker.scale.y = vf_scale_factors_[ i ];
  marker.scale.z = vf_scale_factors_[ i ];
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  return marker;
}

visualization_msgs::InteractiveMarkerControl&
  ARSceneLoader::MakeMeshControl( visualization_msgs::InteractiveMarker &msg,
                                  unsigned int i )
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( MakeMeshMarker( msg, i ) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void ARSceneLoader::LoadControlMeshes( )
{
  for ( unsigned int i = 0; i < ids_.size( ); ++i )
  {
    Make6DofMarker( false, true,
                    visualization_msgs::InteractiveMarkerControl::NONE, i );
    server_->applyChanges( );
  }
}

void ARSceneLoader::LoadCollisionMeshes( ros::Publisher planning_scene_pub,
                                         moveit_msgs::PlanningScene planning_scene )
{
  for ( unsigned int i = 0; i < ids_.size( ); ++i )
  {
    // Create a collision object for a collision mesh
    moveit_msgs::CollisionObject collision_mesh;
    // Frame_id should be mesh name but id could be numeric
    collision_mesh.header.frame_id = mesh_names_[ i ];
    collision_mesh.header.stamp = ros::Time::now( );
    collision_mesh.id = mesh_names_[ i ];
    // Pull in appropriate mesh
    std::string file_path = "package://" + package_name_ + folder_path_ + mesh_names_[ i ] + ".stl";
    shapes::Mesh* mesh = shapes::createMeshFromResource( file_path );
    ROS_INFO_STREAM( file_path );
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
      vertex.x = co_mesh.vertices[ ii ].x * vf_scale_factors_[ i ];
      vertex.y = co_mesh.vertices[ ii ].y * vf_scale_factors_[ i ];
      vertex.z = co_mesh.vertices[ ii ].z * vf_scale_factors_[ i ];
      co_mesh_scaled.vertices.push_back( vertex );
    }

    // Add mesh to collision object meshes
    collision_mesh.meshes.push_back( co_mesh_scaled );
    // Add pose to collision object meshes
    geometry_msgs::Pose pose;
    pose.position.x = xyzs_[ i ][ 0 ] + mesh_translations_[ i ][ 0 ];
    pose.position.y = xyzs_[ i ][ 1 ] + mesh_translations_[ i ][ 1 ];
    pose.position.z = xyzs_[ i ][ 2 ] + mesh_translations_[ i ][ 2 ];
    tf::Quaternion quaternion =
        tf::createQuaternionFromRPY( mesh_rotations_[ i ][ 0 ] + rpys_[ i ][ 0 ],
                                     mesh_rotations_[ i ][ 1 ] + rpys_[ i ][ 1 ],
                                     mesh_rotations_[ i ][ 2 ] + rpys_[ i ][ 2 ] );
    // Copy over quaternion data
    pose.orientation.x = quaternion.x( );
    pose.orientation.y = quaternion.y( );
    pose.orientation.z = quaternion.z( );
    pose.orientation.w = quaternion.w( );
    // Assign pose to the collision mesh
    collision_mesh.mesh_poses.push_back( pose );
    // Specify as a mesh to be added
    collision_mesh.operation = moveit_msgs::CollisionObject::ADD;
    // Add to the world collision scene
    planning_scene.world.collision_objects.push_back( collision_mesh );
    planning_scene.is_diff = true;
    // Publish to collision scene
    planning_scene_pub.publish( planning_scene );
  }
  // Inform user
  ROS_INFO( "All FRVF collision meshes should now be loaded." );
}

void ARSceneLoader::ProcessFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      // Build transform
      static tf::TransformBroadcaster br;
      tf::Transform t;
      tf::Quaternion quaternion( feedback->pose.orientation.x,
                                  feedback->pose.orientation.y,
                                  feedback->pose.orientation.z,
                                  feedback->pose.orientation.w );

      t.setOrigin( tf::Vector3( feedback->pose.position.x,
                                feedback->pose.position.y,
                                feedback->pose.position.z ) );
      t.setRotation( quaternion );
      // Broadcast transform
      br.sendTransform( tf::StampedTransform( t, ros::Time::now( ),
                        world_frame_, "/" + feedback->control_name ) );

      // Create planning scene and publisher for adding the VF collision mesh
      moveit_msgs::PlanningScene planning_scene = moveit_msgs::PlanningScene( );
      ros::Publisher planning_scene_pub =
        node_handle_.advertise< moveit_msgs::PlanningScene >( "/planning_scene",
                                                             1 );
      LoadCollisionMeshes( planning_scene_pub, planning_scene );
      break;
  }

  server_->applyChanges();
}

} // End ar_scene_loader
