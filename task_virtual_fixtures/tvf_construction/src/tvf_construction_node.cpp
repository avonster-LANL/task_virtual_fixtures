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
Doc = This code is part of the TVF generation pipeline. It combines classes to
calculate surface normals, build the point cloud normal GVF layers, and build
a TVF graph.

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
#include <ros/package.h>

#include <stl_pcn.h>
#include <pcn_layers.h>
#include <tvf_graph.h>

int main ( int argc, char** argv )
{
  // Bools for pipeline generation sections
  bool stl_calculations = false, layer_generation = false, all_graphs = false,
    graph = false;
  // Layer record keeper for file names
  unsigned int layer = 1;
  // Layer calculation variables
  double min_offset, max_offset, intralayer_dist, interlayer_dist,
    average_resolution;
  // STL location variables
  std::string package_name, folder_path, mesh_name, input_path, suffix;
  // ROS parameter variables
  std::string id = argv[ 1 ];
  std::string surface_str = "/surface_" + id + "/";
  std::string mesh_str = surface_str + "name";
  std::string min_offset_str = surface_str + "min_offset";
  std::string max_offset_str = surface_str + "max_offset";
  std::string intralayer_dist_str = surface_str + "intralayer_dist";
  std::string interlayer_dist_str = surface_str + "interlayer_dist";
  std::string package_name_str = surface_str + "package_name";
  std::string folder_path_str = surface_str + "folder_path";
  // Run time variables
  std::chrono::system_clock::time_point start, initialized, end;
  // Create cloud with normals to be passed into classes
  pcl::PointCloud< pcl::PointNormal >::Ptr
    input_cloud ( new pcl::PointCloud<pcl::PointNormal> ),
    vf_layer( new pcl::PointCloud<pcl::PointNormal> );
  // Create vectors of point clouds for layer storage
  std::vector < pcl::PointCloud< pcl::PointNormal >::Ptr,
                Eigen::aligned_allocator< pcl::PointCloud<
                pcl::PointNormal >::Ptr > > pc_vector;
  // Output data file
  ofstream data_file;
  // Start ROS
  ros::init( argc, argv, "tvf_construction_node" );
  ros::Time::init( );
  // Start the clock
  start = std::chrono::system_clock::now( );
  // Get ROS parameter values
  if ( !ros::param::get( mesh_str, mesh_name ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << mesh_str );
  }
  if ( !ros::param::get( min_offset_str, min_offset ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << min_offset );
  }
  if ( !ros::param::get( max_offset_str, max_offset ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << max_offset_str );
  }
  if ( !ros::param::get( intralayer_dist_str, intralayer_dist ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << intralayer_dist_str );
  }
  if ( !ros::param::get( interlayer_dist_str, interlayer_dist ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << interlayer_dist_str );
  }
  if ( !ros::param::get( package_name_str, package_name ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << package_name_str );
  }
  if ( !ros::param::get( folder_path_str, folder_path ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << folder_path_str );
  }
  if ( !ros::param::get( "/stl_calculations", stl_calculations ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << "/stl_calculations" );
  }
  if ( !ros::param::get( "/layer_generation", layer_generation ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << "/layer_generation" );
  }
  if ( !ros::param::get( "/all_graphs", all_graphs ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << "/all_graphs" );
  }
  if ( !ros::param::get( "/graph", graph ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << "/graph" );
  }
  // Get file path
  input_path = ros::package::getPath( package_name ) + folder_path + mesh_name;
  // Open the data file
  data_file.open( input_path + "/data/data_" + std::to_string( intralayer_dist ) +
                  ".csv" );
  data_file << mesh_name << ",Distance,Size,Average Resolution,\n";
  // Instantiate normal vector corrector class
  stl_pcn::STLPCN stl_pcn( input_cloud );
  // Instantiate VF construction class with corrected normal point cloud
  pcn_layers::PCNLayers pcn_layers( input_cloud );
  pcn_layers.SetFilePaths( input_path, "" );
  // Initialize class and the virtual fixture point cloud
  tvf_graph::TVFGraph tvf_graph( pc_vector );
  tvf_graph.LoadParams( id );
  // Output initialization time to terminal
  initialized = std::chrono::system_clock::now( );
  ROS_INFO_STREAM( "Initialization time: " <<
                   std::chrono::duration_cast<std::chrono::microseconds>(
                   initialized - start ).count( ) );
  // Interpolate the STL file if in launch file
  if ( stl_calculations )
  {
    // Load the STL, interpolate points, pull normals, and save
    stl_pcn.Run( input_path, "", intralayer_dist, 1e-5, 5.0 );
  }
  if ( layer_generation )
  {
    // Load surface point cloud with normals
    pcn_layers.LoadNormalPCD( );
  }
  // Start moving out specified distance step
  for ( double dist = min_offset; dist <= max_offset; dist += interlayer_dist )
  {
    if ( layer_generation )
    {
      // Extend normals, interpolate and voxelize, save GVF lay and mod file
      pcn_layers.Run( intralayer_dist, dist, layer );
      // Check resolution and output data
      average_resolution = pcn_layers.CheckResolution( intralayer_dist );
      // Get the VF layer
      vf_layer = pcn_layers.GetVFLayer( );
      data_file << "Layer: " << layer << "," << dist << ","
                << vf_layer->points.size( ) << ","
                << average_resolution << std::endl;
      ROS_INFO_STREAM( "Layer: " << layer << " GVF distance: " << dist <<
                       " Size: " << vf_layer->points.size( ) <<
                       " Average Resolution: " << average_resolution );
    }
    if ( all_graphs )
    {
      // Construct suffix for intermediate graphs
      suffix = "_" + std::to_string( layer );
      // Build graph from all layers, subtract last increment
      tvf_graph.Run( layer, suffix );
    }
    // Increment layer
    ++layer;
  }
  if ( graph )
  {
    // Build graph from all layers, subtract last increment
    tvf_graph.Run( layer - 1, "" );
    tvf_graph.SaveLayerData( );
  }
  // Close the data file
  data_file.close( );
  // Output STL calculation time to terminal
  end = std::chrono::system_clock::now( );
  ROS_INFO_STREAM( "Run time: " <<
                   std::chrono::duration_cast<std::chrono::microseconds>(
                   end - start ).count( ) );
  ros::shutdown( );
  return 0;
}