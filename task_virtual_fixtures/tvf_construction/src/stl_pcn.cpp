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
Doc = This code is part of the TVF generation pipeline. This class is for
loading a STL file, interpolating based on task parameters, and converting to
a point cloud with normals.

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

#include <stl_pcn.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>

#include <omp.h>

namespace stl_pcn
{
STLPCN::STLPCN( pcl::PointCloud< pcl::PointNormal >::Ptr input_cloud ) :
  original_cloud_ ( new pcl::PointCloud< pcl::PointXYZ > ),
  working_cloud_ ( new pcl::PointCloud< pcl::PointXYZ > ),
  cloud_normals_ ( new pcl::PointCloud< pcl::Normal > ),
  filtered_with_normals_ ( new pcl::PointCloud< pcl::PointNormal > ),
  tree_ ( new pcl::search::KdTree< pcl::PointXYZ > ),
  cloud_with_normals_ ( input_cloud ),
  stl_info_ ( "stl_info" )
{
  // Set omp thread limit
  omp_set_dynamic( 0 );
  // Get current number of threads
  unsigned int thread_num = omp_get_max_threads( );
  // Use one less than maximum
  omp_set_num_threads( thread_num - 1 );
}

STLPCN::~STLPCN( )
{

}

int STLPCN::Run( std::string input_path, std::string suffix,
  double intralayer_dist, double epsilon_1, double epsilon_2 )
{
  /*
  This function runs the entire class pipeline.
  */
  // Warning for return
  int warning = 0;
  // Layer calculation variables
  double normal_magnitude = 0, problem_percentage = 0;
  // Normals vectors for testing if normal average is withing tolerance
  std::vector< double > stl_normals = { 0.0, 0.0, 0.0, 0.0 };
  // Set input and save file paths
  SetFilePaths( input_path, suffix );
  // Load the STL
  LoadSTL( );
  // Get average mesh unit normal vector
  stl_normals = AverageSTLNormals( );
  // Calculate the normal magnitude
  normal_magnitude = sqrtf( pow( stl_normals[ 0 ], 2 ) +
                            pow( stl_normals[ 1 ], 2 ) +
                            pow( stl_normals[ 2 ], 2 ) );
  std::cout << "Interpolate next" << std::endl;
  InterpolateSTL( intralayer_dist );
  // Check problems with normal percentage
  problem_percentage = STLNormals( );
  // Output information to operator
  if ( normal_magnitude >= epsilon_1 || problem_percentage >= epsilon_2 )
  {
    std::cout << "Average normal magnitude: " << normal_magnitude <<
                 " or high problem percentage: " << problem_percentage <<
                 ". Recommend visual mesh inspection." << std::endl;
    warning = 1;
  }
  else
  {
    std::cout << "Average normal magnitude: " << normal_magnitude <<
                 ", problem percentage: " << problem_percentage << std::endl;
  }
  // Filter and save the PCN
  FilterCloudNormals( );
  SaveCloudNormals( );
  return warning;
}

void STLPCN::SetFilePaths( std::string input_path, std::string suffix )
{
  /*
  This function sets the class variables for input, .pcd, .stl, .obj, and
  cloud normal files paths.
  */
  // Set file path
  file_path_ = input_path;
  // Set file names
  mesh_path_ = file_path_ + ".obj";
  stl_path_ = file_path_ + ".stl";
  pcd_path_ = file_path_ + ".pcd";
  cloud_path_ = file_path_ + suffix + "_cloud_normals.pcd";
}

void STLPCN::PrintFilePaths( )
{
  /*
  This function outputs the class variables for input, .pcd, .stl, .obj, and
  cloud normal files paths to the terminal.
  */
  std::cout << "Mesh file: " << mesh_path_  << "\n"
            << "STL file: " << stl_path_ << "\n"
            << "PCD file: " << pcd_path_ << "\n"
            << "Cloud and normals file: " << cloud_path_ << std::endl;
}

unsigned int STLPCN::LoadPCD( )
{
  /*
  Load the PCD file at the class pcd path location and return the size of the
  PC. Output an error if the file isn't found.
  */
  // Make sure cloud is empty
  original_cloud_->points.clear( );
  // Loading PCD file
  if ( pcl::io::loadPCDFile<pcl::PointXYZ> ( pcd_path_,
                                             *original_cloud_ ) == -1 )
  {
    PCL_ERROR ( "Couldn't read file" );
    return 1;
  }
  return original_cloud_->points.size( );
}

unsigned int STLPCN::LoadPCDNormals( )
{
  /*
  Load the PCD with normals file at the class pcd path location and return the
  size of the PC. Output an error if the file isn't found.
  */
  // Make sure cloud is empty
  filtered_with_normals_->points.clear( );
  // Loading PCD with normals file
  if ( pcl::io::loadPCDFile<pcl::PointNormal> ( cloud_path_,
                                               *filtered_with_normals_ ) == -1 )
  {
    PCL_ERROR ( "Couldn't read file" );
    return 1;
  }
  return filtered_with_normals_->points.size( );
}

unsigned int STLPCN::LoadSTL( )
{
  /*
  Load the STL file at the class stl path location and return the number of
  triangles.
  */
  // Load STL file
  stl_info_ = stl_parser::parse_stl( stl_path_ );
  return stl_info_.triangles.size( );
}

void STLPCN::SetWorkingCloud( )
{
  /*
  This function copies the class original cloud variable into the class working
  cloud variable for later calculations.
  */
  // Make sure cloud is empty
  working_cloud_->points.clear( );
  pcl::copyPointCloud( *original_cloud_, *working_cloud_ );
}

void STLPCN::SetFilterCloud( )
{
  /*
  This function copies the class cloud with normals variable into the class
  filtered with normals variable. To be used when filtering is not desired.
  */
  // Make sure cloud is empty
  filtered_with_normals_->points.clear( );
  pcl::copyPointCloud( *cloud_with_normals_, *filtered_with_normals_ );
}

unsigned int STLPCN::FilterWorkingCloud( )
{
  /*
  This function voxelizes the class original cloud variable and copies it into
  the class working cloud variable. The voxelization leaf size is one fifth of
  the intralayer distance.
  */
  // Set leaf size for filtering
  double leaf_size = intralayer_dist_ * 0.01;
  // Create voxel filter
  pcl::VoxelGrid< pcl::PointXYZ > voxel_filter;
  // Set voxel filter leaf size
  voxel_filter.setLeafSize ( leaf_size, leaf_size, leaf_size );
  // Set voxel filter input cloud
  voxel_filter.setInputCloud ( original_cloud_ );
  // Perform voxel filtering and copy into working cloud
  voxel_filter.filter ( *working_cloud_ );
  return working_cloud_->points.size( );
}

unsigned int STLPCN::FilterCloudNormals( )
{
  /*
  This function voxelizes the class cloud with normals variable and copies it
  into the class filtered with normals variable. The voxelization leaf size is
  one fifth of the intralayer distance.
  */
  // Set leaf size for filtering
  double leaf_size = intralayer_dist_ * 0.01;
  // Create voxel filter
  pcl::VoxelGrid< pcl::PointNormal > voxel_filter;
  // Set voxel filter leaf size
  voxel_filter.setLeafSize ( leaf_size, leaf_size, leaf_size );
  voxel_filter.setInputCloud ( cloud_with_normals_ );
  // Perform voxel filtering and copy into working cloud
  voxel_filter.filter ( *filtered_with_normals_ );
  return filtered_with_normals_->points.size( );
}

std::vector< double > STLPCN::AverageSTLNormals( )
{
  /*
  This function averages the x, y, and z normals and average side length in the
  class STL information and compares it to a provided tolerance value. If the
  normals are greater than the tolerance value it is considered a failure and
  the function returns false. Otherwise the function returns true.
  */
  // Double for summing the total triangle area
  double area_sum;
  // Vector for averaging triangle normals and side lengths
  std::vector< double > normals = { 0.0, 0.0, 0.0, 0.0 };
  // Loop through all triangles
  #pragma omp parallel for schedule( guided )
  for ( unsigned int i = 0; i < stl_info_.triangles.size( ); ++i )
  {
    double triangle_avg = 0.0, area = 0.0;
    std::vector< double > a( 3 ), b( 3 );
    // Calculate triangle area
    a = { stl_info_.triangles[ i ].v2.x - stl_info_.triangles[ i ].v1.x,
          stl_info_.triangles[ i ].v2.y - stl_info_.triangles[ i ].v1.y,
          stl_info_.triangles[ i ].v2.z - stl_info_.triangles[ i ].v1.z };
    b = { stl_info_.triangles[ i ].v3.x - stl_info_.triangles[ i ].v1.x,
          stl_info_.triangles[ i ].v3.y - stl_info_.triangles[ i ].v1.y,
          stl_info_.triangles[ i ].v3.z - stl_info_.triangles[ i ].v1.z };
    area = 0.5 * sqrt( pow( ( ( a[ 1 ] * b[ 2 ] ) - ( a[ 2 ] * b[ 1 ] ) ), 2 ) +
                       pow( ( ( a[ 2 ] * b[ 0 ] ) - ( a[ 0 ] * b[ 2 ] ) ), 2 ) +
                       pow( ( ( a[ 0 ] * b[ 1 ] ) - ( a[ 1 ] * b[ 0 ] ) ), 2 ) );
    // Average the triangle side lengths
    triangle_avg = ( sqrtf( pow( stl_info_.triangles[ i ].v1.x -
                                 stl_info_.triangles[ i ].v2.x, 2 ) +
                            pow( stl_info_.triangles[ i ].v1.y -
                                 stl_info_.triangles[ i ].v2.y, 2 ) +
                            pow( stl_info_.triangles[ i ].v1.z -
                                 stl_info_.triangles[ i ].v2.z, 2 ) ) +
                     sqrtf( pow( stl_info_.triangles[ i ].v2.x -
                                 stl_info_.triangles[ i ].v3.x, 2 ) +
                            pow( stl_info_.triangles[ i ].v2.y -
                                 stl_info_.triangles[ i ].v3.y, 2 ) +
                            pow( stl_info_.triangles[ i ].v2.z -
                                 stl_info_.triangles[ i ].v3.z, 2 ) ) +
                     sqrtf( pow( stl_info_.triangles[ i ].v3.x -
                                 stl_info_.triangles[ i ].v1.x, 2 ) +
                            pow( stl_info_.triangles[ i ].v3.y -
                                 stl_info_.triangles[ i ].v1.y, 2 ) +
                            pow( stl_info_.triangles[ i ].v3.z -
                                 stl_info_.triangles[ i ].v1.z, 2 ) ) ) / 3.0;
    #pragma omp critical ( sum_normals )
    {
      // Keep sum of triangle areas
      area_sum += area;
      // Sum surface normals
      normals[ 0 ] += stl_info_.triangles[ i ].normal.x * area;
      normals[ 1 ] += stl_info_.triangles[ i ].normal.y * area;
      normals[ 2 ] += stl_info_.triangles[ i ].normal.z * area;
      normals[ 3 ] += triangle_avg;
    }
  }
  // Average the surface normals and distance
  normals[ 0 ] /= stl_info_.triangles.size( ) * area_sum;
  normals[ 1 ] /= stl_info_.triangles.size( ) * area_sum;
  normals[ 2 ] /= stl_info_.triangles.size( ) * area_sum;
  normals[ 3 ] /= stl_info_.triangles.size( );
  return normals;
}

unsigned int STLPCN::InterpolateSTL( const double intralayer_dist )
{
  /*
  Function sets the intralayer distance class variable then loops through the
  STL information triangles and passes them to another function to test side
  length and interpolate if necessary. While used instead for loop since the
  number of triangles is increasing.
  */
  // Set intralayer distance class variable
  intralayer_dist_ = intralayer_dist;
  // Vector to keep track of the number of STL information triangles during the
  // interpolation process
  std::vector < unsigned int > num_triangles = { 0, 0 };
  // Assigning here instead of above to avoid compilation warning
  num_triangles[ 1 ] = stl_info_.triangles.size( );
  while ( num_triangles[ 0 ] < stl_info_.triangles.size( ) )
  {
    #pragma omp parallel for schedule( guided )
    for ( unsigned int i = num_triangles[ 0 ]; i <= stl_info_.triangles.size( );
          ++i )
    {
      bool divided = true;
      while ( divided )
      {
        divided = InterpolateTriangle( i );
      }
    }
    // Output growth in the number of triangles to terminal
    std::cout << "Number of triangles expanded from " << num_triangles[ 1 ]
              << " to " << stl_info_.triangles.size( ) << std::endl;
    // Update number of STL information triangles
    num_triangles[ 0 ] = num_triangles[ 1 ];
    num_triangles[ 1 ] = stl_info_.triangles.size( );
  }
  // Convert the STL info to a PC
  STLToPC( );
  return stl_info_.triangles.size( );
}

bool STLPCN::InterpolateTriangle( const unsigned int i )
{
  /*
  Function checks side lengths for the triangle at index i. If a side is too
  long the triangle is divided into smaller triangles. Triangles can be divided
  into either two or six smaller triangles depending on the value of the method
  variable. Dividing into six triangles more uniformly adds points than the
  bisection method.
  */
  // Bool to see if the triangle was divided
  bool divided = false;
  // 0 for bisection division method, 1 for six triangle division method
  unsigned int method = 1;
  // Variables for the triangle points added during triangle division
  stl_parser::point mid_pt, mid_12, mid_23, mid_31;
  // Calculate the lengths of each side
  std::vector< double > lengths =
    { sqrtf( pow( stl_info_.triangles[ i ].v1.x -
                  stl_info_.triangles[ i ].v2.x, 2 ) +
             pow( stl_info_.triangles[ i ].v1.y -
                  stl_info_.triangles[ i ].v2.y, 2 ) +
             pow( stl_info_.triangles[ i ].v1.z -
                  stl_info_.triangles[ i ].v2.z, 2 ) ),
      sqrtf( pow( stl_info_.triangles[ i ].v1.x -
                  stl_info_.triangles[ i ].v3.x, 2 ) +
             pow( stl_info_.triangles[ i ].v1.y -
                  stl_info_.triangles[ i ].v3.y, 2 ) +
             pow( stl_info_.triangles[ i ].v1.z -
                  stl_info_.triangles[ i ].v3.z, 2 ) ),
      sqrtf( pow( stl_info_.triangles[ i ].v2.x -
                  stl_info_.triangles[ i ].v3.x, 2 ) +
             pow( stl_info_.triangles[ i ].v2.y -
                  stl_info_.triangles[ i ].v3.y, 2 ) +
             pow( stl_info_.triangles[ i ].v2.z -
                  stl_info_.triangles[ i ].v3.z, 2 ) ) };
  // Six triangle division method
  if ( method == 1 )
  {
    // If one of the lengths is too long introduce a mid point and cut into
    // six separate triangles
    if ( ( lengths[ 0 ] > intralayer_dist_ ) ||
         ( lengths[ 1 ] > intralayer_dist_ ) ||
         ( lengths[ 2 ] > intralayer_dist_ ) )
    {
      divided = true;
      // Calculate the middle of the triangle
      mid_pt.x = ( stl_info_.triangles[ i ].v1.x +
                   stl_info_.triangles[ i ].v2.x +
                   stl_info_.triangles[ i ].v3.x ) / 3.0;
      mid_pt.y = ( stl_info_.triangles[ i ].v1.y +
                   stl_info_.triangles[ i ].v2.y +
                   stl_info_.triangles[ i ].v3.y ) / 3.0;
      mid_pt.z = ( stl_info_.triangles[ i ].v1.z +
                   stl_info_.triangles[ i ].v2.z +
                   stl_info_.triangles[ i ].v3.z ) / 3.0;
      // Calculate middle points on each side
      mid_12.x = ( stl_info_.triangles[ i ].v1.x +
                   stl_info_.triangles[ i ].v2.x ) * 0.5;
      mid_12.y = ( stl_info_.triangles[ i ].v1.y +
                   stl_info_.triangles[ i ].v2.y ) * 0.5;
      mid_12.z = ( stl_info_.triangles[ i ].v1.z +
                   stl_info_.triangles[ i ].v2.z ) * 0.5;
      mid_23.x = ( stl_info_.triangles[ i ].v2.x +
                   stl_info_.triangles[ i ].v3.x ) * 0.5;
      mid_23.y = ( stl_info_.triangles[ i ].v2.y +
                   stl_info_.triangles[ i ].v3.y ) * 0.5;
      mid_23.z = ( stl_info_.triangles[ i ].v2.z +
                   stl_info_.triangles[ i ].v3.z ) * 0.5;
      mid_31.x = ( stl_info_.triangles[ i ].v3.x +
                   stl_info_.triangles[ i ].v1.x ) * 0.5;
      mid_31.y = ( stl_info_.triangles[ i ].v3.y +
                   stl_info_.triangles[ i ].v1.y ) * 0.5;
      mid_31.z = ( stl_info_.triangles[ i ].v3.z +
                   stl_info_.triangles[ i ].v1.z ) * 0.5;
      // Create new triangle_1
      stl_parser::triangle triangle_1 ( stl_info_.triangles[ i ].normal,
                                        mid_12,
                                        stl_info_.triangles[ i ].v2,
                                        mid_23 );
      // Create new triangle_2
      stl_parser::triangle triangle_2 ( stl_info_.triangles[ i ].normal,
                                        mid_23,
                                        stl_info_.triangles[ i ].v3,
                                        mid_31 );
      // Create new triangle_3
      stl_parser::triangle triangle_3 ( stl_info_.triangles[ i ].normal,
                                        mid_12,
                                        mid_23,
                                        mid_31 );
      /*
      // Create new triangle_4
      stl_parser::triangle triangle_4 ( stl_info_.triangles[ i ].normal,
                                        mid_12,
                                        mid_23,
                                        mid_pt );
      // Create new triangle_5
      stl_parser::triangle triangle_5 ( stl_info_.triangles[ i ].normal,
                                        mid_23,
                                        mid_31,
                                        mid_pt );
      */
      #pragma omp critical ( push_triangle )
      {
        //std::cout << "Interpolating Triangle." << std::endl;
        // Change vertex of current triangle triangle_0
        stl_info_.triangles[ i ].v2 = mid_12;
        stl_info_.triangles[ i ].v3 = mid_31;
        // Add new triangles at end of array to also be checked
        stl_info_.triangles.push_back( triangle_1 );
        stl_info_.triangles.push_back( triangle_2 );
        stl_info_.triangles.push_back( triangle_3 );
        //stl_info_.triangles.push_back( triangle_4 );
        //stl_info_.triangles.push_back( triangle_5 );
      }
    }
    else
    {
      // All lengths are within limits, do nothing
      divided = false;
    }
  }
  // Bisection triangle division method
  else if ( method == 0 )
  {
    // Single side midpoint interpolation method
    if ( lengths[ 0 ] > intralayer_dist_ )
    {
      divided = true;
      mid_pt.x = ( stl_info_.triangles[ i ].v1.x +
                   stl_info_.triangles[ i ].v2.x ) * 0.5;
      mid_pt.y = ( stl_info_.triangles[ i ].v1.y +
                   stl_info_.triangles[ i ].v2.y ) * 0.5;
      mid_pt.z = ( stl_info_.triangles[ i ].v1.z +
                   stl_info_.triangles[ i ].v2.z ) * 0.5;
      // Create new triangle
      stl_parser::triangle new_triangle ( stl_info_.triangles[ i ].normal,
                                          mid_pt,
                                          stl_info_.triangles[ i ].v2,
                                          stl_info_.triangles[ i ].v3 );
      #pragma omp critical ( push_triangle )
      {
        // Change vertex of current triangle
        stl_info_.triangles[ i ].v2 = mid_pt;
        // Add triangle to array so it can also be checked
        stl_info_.triangles.push_back( new_triangle );
      }
    }
    else if ( lengths[ 1 ] > intralayer_dist_ )
    {
      divided = true;
      mid_pt.x = ( stl_info_.triangles[ i ].v2.x +
                   stl_info_.triangles[ i ].v3.x ) * 0.5;
      mid_pt.y = ( stl_info_.triangles[ i ].v2.y +
                   stl_info_.triangles[ i ].v3.y ) * 0.5;
      mid_pt.z = ( stl_info_.triangles[ i ].v2.z +
                   stl_info_.triangles[ i ].v3.z ) * 0.5;
      // Create new triangle
      stl_parser::triangle new_triangle ( stl_info_.triangles[ i ].normal,
                                          stl_info_.triangles[ i ].v1,
                                          mid_pt,
                                          stl_info_.triangles[ i ].v3 );
      #pragma omp critical ( push_triangle )
      {
        // Change vertex of current triangle
        stl_info_.triangles[ i ].v3 = mid_pt;
        // Add triangle to array so it can also be checked
        stl_info_.triangles.push_back( new_triangle );
      }
    }
    else if ( lengths[ 2 ] > intralayer_dist_ )
    {
      divided = true;
      mid_pt.x = ( stl_info_.triangles[ i ].v3.x +
                   stl_info_.triangles[ i ].v1.x ) * 0.5;
      mid_pt.y = ( stl_info_.triangles[ i ].v3.y +
                   stl_info_.triangles[ i ].v1.y ) * 0.5;
      mid_pt.z = ( stl_info_.triangles[ i ].v3.z +
                   stl_info_.triangles[ i ].v1.z ) * 0.5;
      // Create new triangle
      stl_parser::triangle new_triangle ( stl_info_.triangles[ i ].normal,
                                          stl_info_.triangles[ i ].v2,
                                          stl_info_.triangles[ i ].v3,
                                          mid_pt );
      #pragma omp critical ( push_triangle )
      {
        // Change vertex of current triangle
        stl_info_.triangles[ i ].v3 = mid_pt;
        // Add triangle to array so it can also be checked
        stl_info_.triangles.push_back( new_triangle );
      }
    }
    else
    {
      // All lengths are within limits, do nothing
      divided = false;
    }
  }
  return divided;
}

void STLPCN::STLToPC( )
{
  /*
  This function converts the class STL information into the class original cloud
  variable. The conversion method does add duplicates of points which are in
  multiple triangles. Therefore, the cloud is filtered before continuing
  calculations.
  */
  // Clear any existing points
  original_cloud_->points.clear( );
  // Add all triangle vertices to the point cloud
  #pragma omp parallel for schedule( guided )
  for ( unsigned int i = 0; i < stl_info_.triangles.size( ); ++i )
  {
    pcl::PointXYZ new_point_1, new_point_2, new_point_3;
    // Get V1
    new_point_1.x = stl_info_.triangles[ i ].v1.x;
    new_point_1.y = stl_info_.triangles[ i ].v1.y;
    new_point_1.z = stl_info_.triangles[ i ].v1.z;
    // Get V2
    new_point_2.x = stl_info_.triangles[ i ].v2.x;
    new_point_2.y = stl_info_.triangles[ i ].v2.y;
    new_point_2.z = stl_info_.triangles[ i ].v2.z;
    // Get V3
    new_point_3.x = stl_info_.triangles[ i ].v3.x;
    new_point_3.y = stl_info_.triangles[ i ].v3.y;
    new_point_3.z = stl_info_.triangles[ i ].v3.z;
    // Add V1, V2, V3 to point cloud
    #pragma omp critical ( push_point )
    {
      original_cloud_->points.push_back( new_point_1 );
      original_cloud_->points.push_back( new_point_2 );
      original_cloud_->points.push_back( new_point_3 );
    }
  }
  // Resize cloud
  original_cloud_->width = 1;
  original_cloud_->height = original_cloud_->points.size( );
  // Filter cloud to remove duplicate points
  FilterWorkingCloud( );
}

double STLPCN::STLNormals( )
{
  /*
  This function copies the class working cloud variable's point information into
  the class cloud with normals variable. Each point is then assigned the average
  normals of the triangles in which it is present.
  */
  // Variables for the detection of STL surface normal problems
  unsigned int num_unlisted = 0, problem_counter = 0;
  double problem_fraction = 0.0;
  // Make sure cloud is empty
  cloud_with_normals_->points.clear( );
  // Combine filtered cloud and fill in cloud normals from STL data
  pcl::copyPointCloud( *working_cloud_, *cloud_with_normals_ );
  // Iterate through the entire cloud
  #pragma omp parallel for schedule( guided )
  for ( unsigned int i = 0; i < cloud_with_normals_->points.size( ); ++i )
  {
    // Counter for averaging the normals of triangles a point is in and for
    // keeping track of the number of points in the cloud not listed in
    // triangles.
    unsigned int num_triangles = 0;
    // Variable for normalizing the point x, y, z normals
    double normal_sum = 0.0, magnitude = 0.0, area = 0.0, area_sum = 0.0;
    // Vectors to hold point to vertex distances and normal summations
    std::vector< double > dist( 3 ), normals_sum( 3 ), a( 3 ), b( 3 );
    // Iterate through all STL information triangles
    for ( unsigned int j = 0; j < stl_info_.triangles.size( ); ++j )
    {
      // Calculate the distance from the PC point to each of the triangle's
      // vertices
      TriangleVertexesDistances( i, j, dist );
      // If the point in the point cloud is one of the vertices
      if ( ( dist[ 0 ] < intralayer_dist_ * 0.1 ) ||
           ( dist[ 1 ] < intralayer_dist_ * 0.1 ) ||
           ( dist[ 2 ] < intralayer_dist_ * 0.1 ) )
      {
        // Calculate triangle area
        a = { stl_info_.triangles[ j ].v2.x - stl_info_.triangles[ j ].v1.x,
              stl_info_.triangles[ j ].v2.y - stl_info_.triangles[ j ].v1.y,
              stl_info_.triangles[ j ].v2.z - stl_info_.triangles[ j ].v1.z };
        b = { stl_info_.triangles[ j ].v3.x - stl_info_.triangles[ j ].v1.x,
              stl_info_.triangles[ j ].v3.y - stl_info_.triangles[ j ].v1.y,
              stl_info_.triangles[ j ].v3.z - stl_info_.triangles[ j ].v1.z };
        area = 0.5 * sqrt( pow( ( ( a[ 1 ] * b[ 2 ] ) - ( a[ 2 ] * b[ 1 ] ) ), 2 ) +
                           pow( ( ( a[ 2 ] * b[ 0 ] ) - ( a[ 0 ] * b[ 2 ] ) ), 2 ) +
                           pow( ( ( a[ 0 ] * b[ 1 ] ) - ( a[ 1 ] * b[ 0 ] ) ), 2 ) );
        // Keep sum of triangle areas
        area_sum += area;
        // Keep running sum of point x, y, z normals multiplied by triangle area
        normals_sum[ 0 ] += stl_info_.triangles[ j ].normal.x * area;
        normals_sum[ 1 ] += stl_info_.triangles[ j ].normal.y * area;
        normals_sum[ 2 ] += stl_info_.triangles[ j ].normal.z * area;
        // Increment the number of triangles the point is present in
        ++num_triangles;
      }
    }
    // If the point is found in at least one triangle's vertices
    if ( num_triangles > 0 )
    {
      // Average collected normals
      normals_sum[ 0 ] /= num_triangles * area_sum;
      normals_sum[ 1 ] /= num_triangles * area_sum;
      normals_sum[ 2 ] /= num_triangles * area_sum;
      // Calculate vector magnitude
      magnitude = sqrt( pow( normals_sum[ 0 ], 2 ) +
                        pow( normals_sum[ 1 ], 2 ) +
                        pow( normals_sum[ 2 ], 2 ) );
      // Test for problems with normals
      if ( magnitude <= 1e-8 )
      {
        #pragma omp critical ( problem )
        {
          ++problem_counter;
        }
      }
      // Convert to unit vector
      normals_sum[ 0 ] /= magnitude;
      normals_sum[ 1 ] /= magnitude;
      normals_sum[ 2 ] /= magnitude;
      // Put normals in point cloud
      #pragma omp critical ( invert_normals )
      {
        cloud_with_normals_->points[ i ].normal_x = normals_sum[ 0 ];
        cloud_with_normals_->points[ i ].normal_y = normals_sum[ 1 ];
        cloud_with_normals_->points[ i ].normal_z = normals_sum[ 2 ];
      }

    }
    else
    {
      // Increment unlisted counter if the point isn't present in at least one
      // triangle
      #pragma omp critical ( unlisted )
      {
        ++num_unlisted;
      }
      //std::cout << "Point: " << i << " Unlisted points: "
      //          << num_unlisted << std::endl;
    }
  }
  if ( num_unlisted > 0 )
  {
    std::cout << "Unlisted points: " << num_unlisted << std::endl;
  }
  std::cout << "Cloud points: " << cloud_with_normals_->points.size( ) <<
               ", Problem points: " << problem_counter << std::endl;
  problem_fraction = ( double )problem_counter /
    cloud_with_normals_->points.size( );
  return  100.0 * problem_fraction;
}

void STLPCN::EstimateNormals( const unsigned int knn )
{
  /*
  This function estimates point normals based on a k nearest neighbors or
  r radius neighbors plane fitting calculation.
  */
  // Create the normal estimation class with the number of threads
  pcl::NormalEstimationOMP< pcl::PointXYZ, pcl::Normal >
    normal_estimation( 10 );
  // Pass the input point cloud
  normal_estimation.setInputCloud( working_cloud_ );
  // Set the view point to zero since there isn't one for STL conversion
  normal_estimation.setViewPoint( 0.0, 0.0, 0.0 );
  // Create an empty kdtree representation, and pass it to the normal
  // estimation object. Its content will be filled inside the object, based
  // on the given input dataset (as no other search surface is given).
  normal_estimation.setSearchMethod( tree_ );
  // Use k nearest or all neighbors in a sphere of radius x cm
  //normal_estimation.setKSearch( knn );
  normal_estimation.setRadiusSearch( intralayer_dist_ * 3.0 );
  // Compute the point normals
  normal_estimation.compute( *cloud_normals_ );
  // Make sure cloud is empty
  cloud_with_normals_->points.clear( );
  // Combine filtered cloud and cloud normals
  pcl::copyPointCloud( *working_cloud_, *cloud_with_normals_ );
  pcl::copyPointCloud( *cloud_normals_, *cloud_with_normals_ );
  // Output information to terminal
  std::cout << "Estimated cloud normal points: "
            << cloud_with_normals_->points.size( ) << std::endl;
}

void STLPCN::CorrectNormals( )
{
  /*
  This function iterates through the class PC with normals and the STL
  triangles. It finds all of the triangles a point is present in and averages
  the triangle normals. It then compares the angle between the averaged triangle
  normal and the point normal. If the angle is larger than 90 degrees the point
  normal is inverted.
  */
  #pragma omp parallel for schedule( guided )
  for ( unsigned int i = 0; i < cloud_with_normals_->points.size( ); ++i )
  {
    // Counter for averaging the normals of triangles a point is in and for
    // keeping track of the number of points in the cloud not listed in
    // triangles
    unsigned int counter = 0, unlisted_counter = 0;
    double v_dist, magnitude;
    std::vector< double > dist( 3 ), normals( 3 );
    for ( unsigned int j = 0; j < stl_info_.triangles.size( ); ++j )
    {
      TriangleVertexesDistances( i, j, dist );
      // If the point in the point cloud is one of the vertices
      if ( ( dist[ 0 ] < intralayer_dist_ * 0.1 ) ||
           ( dist[ 1 ] < intralayer_dist_ * 0.1 ) ||
           ( dist[ 2 ] < intralayer_dist_ * 0.1 ) )
      {
        normals[ 0 ] += stl_info_.triangles[ j ].normal.x;
        normals[ 1 ] += stl_info_.triangles[ j ].normal.y;
        normals[ 2 ] += stl_info_.triangles[ j ].normal.z;
        ++counter;
      }
    }

    if ( counter > 0 )
    {
      // Average collected normals
      normals[ 0 ] /= counter;
      normals[ 1 ] /= counter;
      normals[ 2 ] /= counter;
      // Change to unit vector
      magnitude =  sqrt( pow( normals[ 0 ], 2 ) +
                         pow( normals[ 1 ], 2 ) +
                         pow( normals[ 2 ], 2 ) );
      normals[ 0 ] /= magnitude;
      normals[ 1 ] /= magnitude;
      normals[ 2 ] /= magnitude;
      // Calculate the distance between the normal unit vectors in case the
      // normal needs inverted
      v_dist = sqrtf ( pow ( ( cloud_with_normals_->points[ i ].normal_x -
                               normals[ 0 ] ), 2 ) +
                       pow ( ( cloud_with_normals_->points[ i ].normal_y -
                               normals[ 1 ] ), 2 ) +
                       pow ( ( cloud_with_normals_->points[ i ].normal_z -
                               normals[ 2 ] ), 2 ) );
      // If the distance is greater than sqrt( 2 ) than the angle is greater
      // than 90 degrees and the normal vector should be inverted
      if ( v_dist > 1.4142 )
      {
        InvertCloudNormal( i );
      }
      else
      {
        // The angle is acceptable, do nothing
      }
    }
    else
    {
      // If the point cloud point isn't present in 1 or more triangles then
      // add it to the unlisted counter
      ++unlisted_counter;
      std::cout << "Point: " << i << " Unlisted points: "
                << unlisted_counter << std::endl;
    }
  }
}

std::vector< double > STLPCN::AveragePCNNormals( )
{
  /*
  This function averages and returns the x, y, and z normals in the class
  variable cloud with normals.
  */
  // Variable for averaging normals
  unsigned int counter = 0;
  std::vector< double > normals = { 0.0, 0.0, 0.0 };
  // Loop through entire point cloud
  for ( unsigned int i = 0; i < cloud_with_normals_->points.size( ); ++i )
  {
    // Check to make sure there aren't any NaNs in the point
    if ( !std::isnan( cloud_with_normals_->points[ i ].x ) &&
         !std::isnan( cloud_with_normals_->points[ i ].y ) &&
         !std::isnan( cloud_with_normals_->points[ i ].z ) &&
         !std::isnan( cloud_with_normals_->points[ i ].normal_x ) &&
         !std::isnan( cloud_with_normals_->points[ i ].normal_y ) &&
         !std::isnan( cloud_with_normals_->points[ i ].normal_z ) &&
         !std::isnan( cloud_with_normals_->points[ i ].normal_y ) &&
         !std::isnan( cloud_with_normals_->points[ i ].curvature ) )
    {
      // Increment counter to avoid nans
      ++counter;
      // Sum surface normals
      normals[ 0 ] += cloud_with_normals_->points[ i ].normal_x;
      normals[ 1 ] += cloud_with_normals_->points[ i ].normal_y;
      normals[ 2 ] += cloud_with_normals_->points[ i ].normal_z;
    }
  }
  // Average the surface normals
  normals[ 0 ] = normals[ 0 ] / counter;
  normals[ 1 ] = normals[ 1 ] / counter;
  normals[ 2 ] = normals[ 2 ] / counter;
  return normals;
}

void STLPCN::MovingLeastSquaresEstimation( )
{
  /*
  This function applies a moving least squares PC smoothing algorithm.
  */
  // Create new PC
  pcl::PointCloud< pcl::PointNormal > mls_points;
  // Init object (second point type is for the normals, even if unused )
  pcl::MovingLeastSquares< pcl::PointXYZ, pcl::PointNormal > mls;
  // Set input PC
  mls.setInputCloud ( working_cloud_ );
  // Set parameters
  mls.setComputeNormals ( true );
  mls.setPolynomialFit ( true );
  mls.setSearchMethod ( tree_ );
  mls.setSearchRadius ( 0.05 );
  // Reconstruct
  mls.process( mls_points );
  // Make sure cloud is empty
  working_cloud_->points.clear( );
  // Copy information back into class working cloud variable
  pcl::copyPointCloud( mls_points, *working_cloud_ );
  // Output size to terminal
  std::cout << "MLS normal points: " << working_cloud_->points.size( )
            << std::endl;
}

void STLPCN::TriangleVertexesDistances( const unsigned int i,
                                        const unsigned int j,
                                        std::vector< double > &dist )
{
  /*
  Function calculates the distance between a the point at the specified index in
  the class cloud with normals variable and the vertices of the triangle at the
  specified class STL information index.
  */
  dist = { sqrtf( pow( cloud_with_normals_->points[ i ].x -
                       stl_info_.triangles[ j ].v1.x, 2 ) +
                  pow( cloud_with_normals_->points[ i ].y -
                       stl_info_.triangles[ j ].v1.y, 2 ) +
                  pow( cloud_with_normals_->points[ i ].z -
                       stl_info_.triangles[ j ].v1.z, 2 ) ),
           sqrtf( pow( cloud_with_normals_->points[ i ].x -
                       stl_info_.triangles[ j ].v2.x, 2 ) +
                  pow( cloud_with_normals_->points[ i ].y -
                       stl_info_.triangles[ j ].v2.y, 2 ) +
                  pow( cloud_with_normals_->points[ i ].z -
                       stl_info_.triangles[ j ].v2.z, 2 ) ),
           sqrtf( pow( cloud_with_normals_->points[ i ].x -
                       stl_info_.triangles[ j ].v3.x, 2 ) +
                  pow( cloud_with_normals_->points[ i ].y -
                       stl_info_.triangles[ j ].v3.y, 2 ) +
                  pow( cloud_with_normals_->points[ i ].z -
                       stl_info_.triangles[ j ].v3.z, 2 ) ) };
}

void STLPCN::InvertCloudNormal( const unsigned int i )
{
  /*
  Function inverts the normal vector for the point at the specified index in
  the class cloud with normals variable
  */
  #pragma omp critical ( invert_normals )
  {
    cloud_with_normals_->points[ i ].normal_x *= -1.0;
    cloud_with_normals_->points[ i ].normal_y *= -1.0;
    cloud_with_normals_->points[ i ].normal_z *= -1.0;
  }
}

void STLPCN::ViewCloudNormals( double size )
{
  /*
  This function displays the task mesh and class variable filtered with normals
  in PCLVisualizer.
  */
  // Create mesh variable and load task mesh
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileOBJ( mesh_path_, mesh );
  // Make class variable filtered with normals' points red
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointNormal > red
    ( filtered_with_normals_, 255, 0, 0 );
  // Create PCLVisualizer viewer
  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer
    ( new pcl::visualization::PCLVisualizer( "3D Viewer" ) );
  // Set viewer parameters
  viewer->setBackgroundColor ( 0, 0, 0 );
  viewer->addCoordinateSystem ( 1.0 * size );
  // Add class variable filtered with normals' points to viewer as red
  viewer->addPointCloud< pcl::PointNormal >
    ( filtered_with_normals_, red, "cloud with normals" );
  // Add class variable filtered with normals' normals to viewer
  viewer->addPointCloudNormals< pcl::PointNormal, pcl::PointNormal >
    ( filtered_with_normals_, filtered_with_normals_, 1, 0.5 * size, "normals" );
  // Render class variable filtered with normals
  viewer->setPointCloudRenderingProperties
    ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud with normals" );
  // Add task mesh to viewer
  viewer->addPolygonMesh( mesh, "meshes", 0 );
  // Spin viewer until it is closed
  while ( !viewer->wasStopped ( ) )
  {
    viewer->spinOnce ( 100 );
  }
}

unsigned int STLPCN::SaveCloudNormals( )
{
  /*
  Function saves class variable filtered with normals to class variable cloud
  path's location as ASCII file.
  */
  pcl::io::savePCDFileASCII( cloud_path_, *filtered_with_normals_ );
  return filtered_with_normals_->points.size( );
}

pcl::PointCloud< pcl::PointNormal >::Ptr STLPCN::GetPointCloud( )
{
  /*
  Function returns class variable filtered with normals
  */
  return filtered_with_normals_;
}

}