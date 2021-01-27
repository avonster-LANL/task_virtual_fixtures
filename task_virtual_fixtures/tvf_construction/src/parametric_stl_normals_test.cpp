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
  // Layer record keeper for file names
  unsigned int layer = 0, total_counter = 0, warning_counter = 0,
    num_triangles = 0, original_num_triangles = 0, num_points = 0, iterations;
  int k_nearest_neighbor;
  // Bool for choosing STL or estimated surface normals
  bool use_stl_normals = true;
  // Layer calculation variables
  double min_offset, max_offset, intralayer_dist, interlayer_dist,
    problem_percentage = 0, normal_tolerance;
  // Normals vectors for testing if normal average is withing tolerance
  std::vector< double > stl_normals = { 0.0, 0.0, 0.0, 0.0 },
    pcn_normals = { 0.0, 0.0, 0.0 };
  // STL location variables
  std::string id = argv[ 1 ], package_name, folder_path, mesh_name, input_path,
    file_name, file_path;
  // ROS parameter variables
  std::string surface_str = "/surface_" + id + "/";
  std::string mesh_str = surface_str + "name";
  std::string k_nearest_neighbor_str = surface_str + "k_nearest_neighbor";
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
    vf_layer ( new pcl::PointCloud< pcl::PointNormal > );
  // Create vectors of point clouds for layer storage
  std::vector < pcl::PointCloud< pcl::PointNormal >::Ptr,
                Eigen::aligned_allocator< pcl::PointCloud<
                pcl::PointNormal >::Ptr > > vf_layers, pc_vector;
  // Start ROS
  ros::init( argc, argv, "parametric_stl_testing_node" );
  ros::NodeHandle node_handle;
  ros::Time::init( );
  // Get ROS parameter values
  if ( !ros::param::get( mesh_str, mesh_name ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << mesh_str );
  }
  if ( !ros::param::get( k_nearest_neighbor_str, k_nearest_neighbor ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << k_nearest_neighbor_str );
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
  for ( unsigned int i = 0; i < files.size( ); ++i )
  {
    found = files[ i ].find( "0.stl" );
    if ( found != std::string::npos )
    {
      // Get file path
      file_path = input_path + files[ i ].substr( 0, files[ i ].size( ) - 4 );
      ROS_INFO_STREAM( "ID: " << id << " File path: " << file_path );
      // Set the file paths and load the STL
      stl_pcn.SetFilePaths( file_path, "" );
      original_num_triangles = stl_pcn.LoadSTL( );
      // Check STL normals
      stl_normals = stl_pcn.AverageSTLNormals( );
      ROS_INFO_STREAM( "Average triangle side distance: " << stl_normals[ 3 ] <<
                       " Normal x: " << stl_normals[ 0 ] <<
                       " Normal y: " << stl_normals[ 1 ] <<
                       " Normal z: " << stl_normals[ 2 ] );
      iterations = 1;
      // Interpolate STL based on task intralayer distance
      intralayer_dist = ( stl_normals[ 3 ] * 16.0 ) / pow( 2.0, iterations );
      original_num_triangles = stl_pcn.InterpolateSTL( intralayer_dist );
      problem_percentage = stl_pcn.STLNormals( );
      // Check problems with normal percentage
      if ( problem_percentage > 2.0 )
      {
        ROS_WARN_STREAM( "Problem percentage: " << problem_percentage <<
                         " recommend visual mesh inspection." );
      }
      else
      {
        ROS_INFO_STREAM( "Problem percentage: " << problem_percentage );
      }
      /*
      // Voxel filter the PC with normals
      stl_pcn.FilterCloudNormals( );
      // Check to see if averaged normals are outside of provided tolerance
      if ( std::abs( stl_normals[ 0 ] ) > normal_tolerance ||
           std::abs( stl_normals[ 1 ] ) > normal_tolerance ||
           std::abs( stl_normals[ 2 ] ) > normal_tolerance )
      {
        pcn_normals = stl_pcn.AveragePCNNormals( );
        // Check to see if averaged normals are outside of provided tolerance
        while ( iterations <= 4 )
        {
          // Interpolate STL based on task intralayer distance
          intralayer_dist = ( stl_normals[ 3 ] * 16.0 ) / pow( 2.0, iterations );
          ROS_INFO_STREAM( "Intralayer distance: " << intralayer_dist );
          num_triangles = stl_pcn.InterpolateSTL( intralayer_dist );
          stl_pcn.STLNormals( );
          // Voxel filter the PC with normals
          stl_pcn.FilterCloudNormals( );
          pcn_normals = stl_pcn.AveragePCNNormals( );
          if ( std::abs( pcn_normals[ 0 ] ) > normal_tolerance ||
               std::abs( pcn_normals[ 1 ] ) > normal_tolerance ||
               std::abs( pcn_normals[ 2 ] ) > normal_tolerance )
          {
            ROS_WARN_STREAM( "Iterations: " << iterations <<
                             " Intralayer distance: " << intralayer_dist <<
                             " Normal x: " << pcn_normals[ 0 ] <<
                             " Normal y: " << pcn_normals[ 1 ] <<
                             " Normal z: " << pcn_normals[ 2 ] <<
                             ". PCN failed average normals test. " <<
                             "Interpolating further to check mesh." );
            ++iterations;
          }
          else
          {
            break;
          }
        }
        if ( iterations <= 4 )
        {
          ROS_INFO_STREAM( "Interpolated PCN passed average normals test." <<
                           " Normal x: " << pcn_normals[ 0 ] <<
                           " Normal y: " << pcn_normals[ 1 ] <<
                           " Normal z: " << pcn_normals[ 2 ] );
        }
        else
        {
          ROS_ERROR_STREAM( "Further interpolation failed. " <<
                            "Verify surface normals." <<
                            " Normal x: " << pcn_normals[ 0 ] <<
                            " Normal y: " << pcn_normals[ 1 ] <<
                            " Normal z: " << pcn_normals[ 2 ] <<
                            " Original triangles: " << original_num_triangles <<
                            " Current triangles: " << num_triangles );
          ++warning_counter;
        }
      }
      else
      {
        ROS_INFO_STREAM( "STL passed average normals test. Normal x: " <<
                         stl_normals[ 0 ] << " Normal y: " << stl_normals[ 1 ] <<
                         " Normal z: " << stl_normals[ 2 ] );
      }
      */
      ++total_counter;
    }
  }
  ROS_INFO_STREAM( "Mesh surface normals warnings " << warning_counter <<
                   " out of " << total_counter << " total." );
  ros::shutdown( );
  return 0;
}