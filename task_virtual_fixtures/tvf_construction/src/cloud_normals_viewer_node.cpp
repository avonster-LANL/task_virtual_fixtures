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
Doc = This code is used to load and view a point cloud with normals.

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

#include <ros/ros.h>
#include <ros/package.h>

#include <stl_pcn.h>
#include <pcn_layers.h>

int main ( int argc, char** argv )
{
  // Start ROS
  ros::init( argc, argv, "cloud_normal_viewer" );
  // Create cloud with normals to be passed into classes
  pcl::PointCloud< pcl::PointNormal >::Ptr
    input_cloud ( new pcl::PointCloud< pcl::PointNormal > );
  // Get file path
  std::string mesh_name = argv[ 1 ];
  std::string input_path = ros::package::getPath( "tvf_data" ) +
                           "/meshes/" + mesh_name;

  // Instantiate classes
  pcn_layers::PCNLayers pcn_layers( input_cloud );
  // Set input path, load cloud with normals, and view
  pcn_layers.SetFilePaths( input_path, "_10" );
  pcn_layers.LoadNormalPCD( );
  pcn_layers.LoadVFPCD( );
  pcn_layers.ViewCloudNormals( 50.0 );
  return 0;
}