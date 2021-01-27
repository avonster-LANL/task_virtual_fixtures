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
calculate surface normals and builds the point cloud normal GVF layers.

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
  // Layer record keeper for file names
  unsigned int layers = 0, total_counter = 0, warning_counter = 0,
    original_num_triangles = 0, num_triangles = 0, num_points = 0, iterations, i;
  std::vector < unsigned int >
    models = { 38, 50, 51, 52, 54, 55,
               56, 57, 58, 59, 62,
               63, 64, 65, 66, 67,
               68, 69, 70, 71, 72,
               73, 74, 76, 77, 78 };

  int k_nearest_neighbor;
  // Bool for choosing STL or estimated surface normals
  bool use_stl_normals = true;
  // Layer calculation variables
  double min_offset, max_offset, intralayer_dist, interlayer_dist,
    problem_percentage = 0, normal_tolerance;
  // Normals vectors for testing if normal average is withing tolerance
  std::vector< double > stl_normals = { 0.0, 0.0, 0.0, 0.0 },
    pcn_normals = { 0.0, 0.0, 0.0 };
  // STL file location variables
  std::string package_name, folder_path, mesh_name, input_path, suffix;
  // ROS parameter variables
  std::string id, surface_str, mesh_str, k_nearest_neighbor_str, min_offset_str,
    max_offset_str, intralayer_dist_str, interlayer_dist_str,
    normal_tolerance_str, package_name_str, folder_path_str;
  // Create cloud with normals to be passed into classes
  pcl::PointCloud< pcl::PointNormal >::Ptr
    input_cloud ( new pcl::PointCloud< pcl::PointNormal > );
  // Instantiate normal vector corrector class
  stl_pcn::STLPCN stl_pcn( input_cloud );
  // Start ROS
  ros::init( argc, argv, "long_stl_pcn_construction_node" );
  ros::Time::init( );
  // Loop through surface models to check for incorrect surface normals
  for ( unsigned int ii = 0; ii < models.size( ); ++ii )
  {
    i = models[ ii ];
    // ROS parameter variables
    id = std::to_string( i );
    surface_str = "/surface_" + id + "/";
    mesh_str = surface_str + "name";
    k_nearest_neighbor_str = surface_str + "k_nearest_neighbor";
    min_offset_str = surface_str + "min_offset";
    max_offset_str = surface_str + "max_offset";
    intralayer_dist_str = surface_str + "intralayer_dist";
    interlayer_dist_str = surface_str + "interlayer_dist";
    normal_tolerance_str = surface_str + "normal_tolerance";
    package_name_str = surface_str + "package_name";
    folder_path_str = surface_str + "folder_path";
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
    input_path = ros::package::getPath( package_name ) + folder_path +
      mesh_name;
    ROS_INFO_STREAM( "ID: " << id << " File path: " << input_path );
    // Set the file paths
    stl_pcn.SetFilePaths( input_path, "" );
    // Load the STL
    original_num_triangles = stl_pcn.LoadSTL( );
    // Set the file paths and load the STL
    stl_normals = stl_pcn.AverageSTLNormals( );
    ROS_INFO_STREAM( "Average triangle side distance: " << stl_normals[ 3 ] <<
                     " Normal x: " << stl_normals[ 0 ] <<
                     " Normal y: " << stl_normals[ 1 ] <<
                     " Normal z: " << stl_normals[ 2 ] );
    iterations = 1;
    // Interpolate STL based on task intralayer distance
    intralayer_dist = ( stl_normals[ 3 ] * 16.0 ) / pow( 2.0, iterations );
    ROS_INFO_STREAM( "Intralayer distance: " << intralayer_dist );
    original_num_triangles = stl_pcn.InterpolateSTL( intralayer_dist );
    // Check problems with normal percentage
    problem_percentage = stl_pcn.STLNormals( );
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
    // Check to see if averaged normals are outside of provided tolerance
    if ( std::abs( stl_normals[ 0 ] ) > normal_tolerance ||
         std::abs( stl_normals[ 1 ] ) > normal_tolerance ||
         std::abs( stl_normals[ 2 ] ) > normal_tolerance )
    {
      pcn_normals = stl_pcn.AveragePCNNormals( );
      // Check to see if averaged normals are outside of provided tolerance
      while ( iterations <= 5 )
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
      if ( iterations <= 5 )
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
  ROS_INFO_STREAM( "Mesh surface normals warnings " << warning_counter <<
                   " out of " << total_counter << " total." );
  ros::shutdown( );
  return 0;
}