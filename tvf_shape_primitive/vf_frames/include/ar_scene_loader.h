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

#ifndef AR_SCENE_LOADER_H
#define AR_SCENE_LOADER_H

#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

namespace ar_scene_loader
{
class ARSceneLoader
{
public:
ARSceneLoader( ros::NodeHandle node_handle,
               boost::shared_ptr< interactive_markers::InteractiveMarkerServer > im_server );
// Interactive marker server
boost::shared_ptr< interactive_markers::InteractiveMarkerServer > server_;
// Functions
void LoadROSParams( );
void LoadControlMeshes( );
void FrameCallback( const ros::TimerEvent& );

private:
// Parameter variables
ros::NodeHandle node_handle_;
std::string world_frame_;
// AR tag information
std::vector< int > ids_;
std::vector< std::vector< double > > xyzs_;
std::vector< std::vector< double > > rpys_;
// Vectors of surface information
std::string package_name_;
std::string folder_path_;
std::vector< std::string > mesh_names_;
std::vector< std::string > types_;
std::vector< double > vf_scale_factors_;
std::vector< double > marker_scale_factors_;
std::vector< std::vector< double > > frame_translations_;
std::vector< std::vector< double > > frame_rotations_;
std::vector< std::vector< double > > mesh_translations_;
std::vector< std::vector< double > > mesh_rotations_;

// Functions
void LoadCollisionMeshes( ros::Publisher pub, moveit_msgs::PlanningScene planning_scene );
void Make6DofMarker( bool fixed, bool show_6dof, unsigned int interaction_mode,
                     unsigned int i );
visualization_msgs::InteractiveMarkerControl&
  MakeMeshControl( visualization_msgs::InteractiveMarker &msg, unsigned int i );
visualization_msgs::Marker
  MakeMeshMarker( visualization_msgs::InteractiveMarker &msg, unsigned int i );
void ProcessFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

};

}

#endif