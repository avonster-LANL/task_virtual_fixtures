/*------------------------------------------------------------------------
Author = Andrew Sharp
Copyright = Copyright 2017, The University of Texas at Austin,
Nuclear Robotics Group
Credits = Andrew Sharp
License = BSD
Version = 1.0.1
Maintainer = Andrew Sharp
Email = asharp@utexas.edu
Status = Production
Doc = This code is part of the TVF generation pipeline. It combines classes to
calculate surface normals, build the point cloud normal GVF layers, and build
a TVF graph. This node reads a batch of parametric STL files and outputs their
graph structures.

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
#include <thread>
#include <ros/ros.h>
#include <ros/package.h>

#include <stl_pcn.h>
#include <pcn_layers.h>
#include <tvf_graph.h>

struct path_leaf_string
{
  std::string operator( )
    ( const boost::filesystem::directory_entry& entry ) const
  {
      return entry.path( ).leaf( ).string( );
  }
};

int main ( int argc, char** argv )
{
  // Bools for pipeline generation sections
  bool stl_calculations = false, layer_generation = false, all_graphs = false,
    graph = false;
  // Layer record keeper for file names
  unsigned int layers = 0, total_counter = 0, warning_counter = 0,
    num_triangles = 0, num_points = 0;
  // Layer calculation variables
  double min_offset, max_offset, intralayer_dist, interlayer_dist,
    problem_percentage = 0, normal_tolerance, normal_magnitude,
    average_resolution, stl_multiplier, const_resolution;
  // Normals vector for testing if normal average is withing tolerance
  std::vector< double > stl_normals = { 0.0, 0.0, 0.0, 0.0 },
    stl_normals_2 = { 0.0, 0.0, 0.0, 0.0 }, pcn_normals = { 0.0, 0.0, 0.0 };
  // STL location variables
  std::string id = argv[ 1 ], package_name, folder_path, mesh_name, input_path,
    file_name, file_path, suffix;
  // ROS parameter variables
  std::string surface_str = "/surface_" + id + "/";
  std::string mesh_str = surface_str + "name";;
  std::string min_offset_str = surface_str + "min_offset";
  std::string max_offset_str = surface_str + "max_offset";
  std::string intralayer_dist_str = surface_str + "intralayer_dist";
  std::string interlayer_dist_str = surface_str + "interlayer_dist";
  std::string normal_tolerance_str = surface_str + "normal_tolerance";
  std::string package_name_str = surface_str + "package_name";
  std::string folder_path_str = surface_str + "folder_path";
  // String vector for file iteration
  std::vector< std::string > files;
  std::size_t found;
  // Output data file
  ofstream data_file;
  // Create cloud with normals to be passed into classes
  pcl::PointCloud< pcl::PointNormal >::Ptr
    input_cloud ( new pcl::PointCloud< pcl::PointNormal > ),
    vf_layer( new pcl::PointCloud<pcl::PointNormal> );
  // Create vectors of point clouds for layer storage
  std::vector < pcl::PointCloud< pcl::PointNormal >::Ptr,
                Eigen::aligned_allocator< pcl::PointCloud<
                pcl::PointNormal >::Ptr > > pc_vector;
  // Start ROS
  ros::init( argc, argv, "parametric_tvf_construction_node" );
  ros::NodeHandle node_handle;
  ros::Time::init( );
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
  if ( !ros::param::get( normal_tolerance_str, normal_tolerance ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << normal_tolerance_str );
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
  if ( !ros::param::get( "/stl_multiplier", stl_multiplier ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << "/stl_multiplier" );
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
  input_path = ros::package::getPath( package_name ) + folder_path;
  // Instantiate normal vector corrector class
  stl_pcn::STLPCN stl_pcn( input_cloud );
  // Instantiate VF construction class with corrected normal point cloud
  pcn_layers::PCNLayers pcn_layers( input_cloud );
  // Initialize class and the virtual fixture point cloud
  tvf_graph::TVFGraph tvf_graph( pc_vector );
  // Boost variables for iteration through the folder
  boost::filesystem::path folder( input_path );
  boost::filesystem::directory_iterator start( folder ), end;
  // Iterate through the file
  std::transform( start, end, std::back_inserter( files ),
                  path_leaf_string( ) );
  // Open the data file
  data_file.open( input_path + "/data/data_" + std::to_string( stl_multiplier ) +
                  ".csv" );
  for ( unsigned int i = 0; i < files.size( ); ++i )
  {
    found = files[ i ].find( "0.stl" );
    if ( found != std::string::npos )
    {
      // Get file path
      file_path = input_path + files[ i ].substr( 0, files[ i ].size( ) - 4 );
      // Set the file paths
      stl_pcn.SetFilePaths( file_path, "" );
      pcn_layers.SetFilePaths( file_path, "" );
      // Load the STL
      num_triangles = stl_pcn.LoadSTL( );
      // Set the file paths and load the STL
      stl_normals = stl_pcn.AverageSTLNormals( );
      // Calculate the normal magnitude, intralayer distance, and
      // constant resolution radius
      normal_magnitude = sqrtf( pow( stl_normals[ 0 ], 2 ) +
                                pow( stl_normals[ 1 ], 2 ) +
                                pow( stl_normals[ 2 ], 2 ) );
      intralayer_dist = stl_normals[ 3 ] * stl_multiplier;
      const_resolution = stl_normals[ 3 ] * 5.0;
      ROS_INFO_STREAM( "Intralayer distance: " << intralayer_dist );
      // Interpolate the STL file if in launch file
      if ( stl_calculations )
      {
        // Load the STL, interpolate points, pull normals, and save
        warning_counter +=
          stl_pcn.Run( file_path, "", intralayer_dist, 1e-5, 5.0 );
      }
      if ( layer_generation )
      {
        // Load surface point cloud with normals
        num_points = pcn_layers.LoadNormalPCD( );
        data_file << files[ i ].substr( 0, files[ i ].size( ) - 4 )
                  << ",Distance,Size,Average Resolution,\n";
      }
      // Clear variables
      layers = 1;
      // Start moving out specified distance step
      //for ( double dist = min_offset; dist <= max_offset; dist += interlayer_dist )
      for ( double dist = stl_normals[ 3 ] * 10.0;
            dist <= stl_normals[ 3 ] * 50.0 + 0.1;
            dist += stl_normals[ 3 ] * 10.0 )
      {
        if ( layer_generation )
        {
          // Extend normals, interpolate and voxelize, save GVF lay and mod file
          pcn_layers.Run( intralayer_dist, dist, layers );
          // Check resolution and output data
          average_resolution = pcn_layers.CheckResolution( const_resolution );
          // Get the VF layer
          vf_layer = pcn_layers.GetVFLayer( );
          data_file << "Layer: " << layers << "," << dist << ","
                    << vf_layer->points.size( ) << ","
                    << average_resolution << std::endl;
          ROS_INFO_STREAM( "Layer: " << layers << " GVF distance: " << dist <<
                           " Size: " << vf_layer->points.size( ) <<
                           " Average Resolution: " << average_resolution );
        }
        // Increment layer
        ++layers;
      }
      total_counter++;
    }
    else
    {
      // Do nothing
    }
  }
  if ( layer_generation )
  {
    // Close the data file
    data_file.close( );
  }
  ROS_INFO_STREAM( "Mesh surface normals warnings " << warning_counter <<
                   " out of " << total_counter << " total." );
  if ( graph )
  {
    for ( unsigned int z_squareness = 0; z_squareness <= 4; z_squareness += 2 )
    {
      for ( unsigned int xy_squareness = 0; xy_squareness <= 4;
            xy_squareness += 2 )
      {
        // Build file names
        file_name = "z_" + std::to_string( z_squareness * 10 ) +
                    "_xy_" + std::to_string( xy_squareness * 10 );
        node_handle.setParam( "surface_" + id + "/name", file_name );
        tvf_graph.LoadParams( id );
        // Build graph from all layers, subtract last increment
        tvf_graph.Run( layers - 1, "" );
        tvf_graph.SaveLayerData( );
      }
    }
  }
  ros::shutdown( );
  return 0;
}