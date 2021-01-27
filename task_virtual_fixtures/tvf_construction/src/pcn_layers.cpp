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
constructing point cloud normal TVF layers from an input point cloud with
normals and task information.

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

#include <pcn_layers.h>

#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <omp.h>

namespace pcn_layers
{

PCNLayers::PCNLayers( pcl::PointCloud< pcl::PointNormal >::Ptr input_cloud ) :
  cloud_with_normals_ ( input_cloud ),
  vf_layer_ ( new pcl::PointCloud< pcl::PointNormal > ),
  layer_mesh_ ( new pcl::PointCloud< pcl::PointNormal > ),
  kdtree_ ( new pcl::KdTreeFLANN< pcl::PointNormal > )
{
  // Set omp thread limit
  omp_set_dynamic( 0 );
  // Get current number of threads
  unsigned int thread_num = omp_get_max_threads( );
  // Use one less than maximum
  omp_set_num_threads( thread_num - 1 );
}

PCNLayers::~PCNLayers( )
{

}

void PCNLayers::SetFilePaths( std::string input_path, std::string suffix )
{
  file_path_ = input_path;
  // Set file names
  cloud_path_ = file_path_ + suffix + "_cloud_normals.pcd";
  mesh_path_ = file_path_ + ".obj";
  vf_path_ = file_path_ + suffix + "_vf_layer";

  std::cout << "Cloud and normals file: " << cloud_path_ << "\n"
            << "Mesh file: " << mesh_path_ << "\n" << "VF file: "
            << vf_path_ << std::endl;
}

unsigned int PCNLayers::LoadNormalPCD( )
{
  std::cout << cloud_path_ << std::endl;
  cloud_with_normals_->points.clear( );
  // Loading PCD file
  if ( pcl::io::loadPCDFile< pcl::PointNormal > ( cloud_path_,
                                                  *cloud_with_normals_ ) == -1 )
  {
    PCL_ERROR ( "Couldn't read file" );
    return 1;
  }
  else
  {
    return cloud_with_normals_->points.size( );
  }
}

unsigned int PCNLayers::LoadVFPCD( )
{
  // Loading PCD file
  std::string filename = vf_path_ + "_2.pcd";
  vf_layer_->points.clear( );
  if ( pcl::io::loadPCDFile< pcl::PointNormal > ( filename,
                                                  *vf_layer_ ) == -1 )
  {
    PCL_ERROR ( "Couldn't read file" );
    return 1;
  }
  else
  {
    return vf_layer_->points.size( );
  }
}

void PCNLayers::Run( double intralayer_dist, double dist, unsigned int layer )
{
  /*
  This function run the entire PC portion of the TVF generation pipeline.
  ConvertToMesh, ViewVFLayer, and SaveVFMOD are optional functions.
  */
  // Extend surface normals
  ExtendNormals( dist );
  // Convert Layer to mesh in case the FRVF is needed
  //ConvertToMesh( layer );
  // Interpolate and voxelize the GVF layer
  InterpolateVoxelize( intralayer_dist, dist );
  // View VF layer before saving if necessary
  //ViewVFLayer( );
  // Save the point cloud layer
  SaveVFLayer( layer );
  // Save the point cloud layer for use with ABB software
  SaveVFMod( layer );
}

void PCNLayers::ExtendNormals( double dist )
{
  /*
  This function extends the input cloud normals, inverts the point normal, and
  adds the point to a new point cloud to create a new VF layer.
  */
  // Clear any existing points
  vf_layer_->points.clear( );
  // Loop through point cloud
  #pragma omp parallel for schedule( guided )
  for ( unsigned int i = 0; i < cloud_with_normals_->points.size( ); ++i )
  {
    pcl::PointNormal new_point;
    new_point.x = cloud_with_normals_->points[ i ].x +
                  ( cloud_with_normals_->points[ i ].normal_x * dist );
    new_point.y = cloud_with_normals_->points[ i ].y +
                  ( cloud_with_normals_->points[ i ].normal_y * dist );
    new_point.z = cloud_with_normals_->points[ i ].z +
                  ( cloud_with_normals_->points[ i ].normal_z * dist );
    // Invert normal
    new_point.normal_x = -cloud_with_normals_->points[ i ].normal_x;
    new_point.normal_y = -cloud_with_normals_->points[ i ].normal_y;
    new_point.normal_z = -cloud_with_normals_->points[ i ].normal_z;
    new_point.curvature = cloud_with_normals_->points[ i ].curvature;
    #pragma omp critical ( push_point )
    {
      vf_layer_->points.push_back( new_point );
    }
  }
  // Resize cloud to save
  vf_layer_->width = 1;
  vf_layer_->height = vf_layer_->points.size( );
}

unsigned int PCNLayers::InterpolateVoxelize( double &intralayer_dist,
                                             double &dist )
{
  unsigned int original_size = vf_layer_->points.size( ),
    interpolated_size;
  // Set leaf size for filtering
  float r_size = 0.5 * dist + 2.0 * intralayer_dist,
    leaf_size = intralayer_dist;
  // Voxel filter for after interpolation
  pcl::VoxelGrid< pcl::PointNormal > voxel_filter;
  pcl::PointCloud< pcl::PointNormal >::Ptr
    additions ( new pcl::PointCloud< pcl::PointNormal > ),
    filtered ( new pcl::PointCloud< pcl::PointNormal > );
  additions->points.clear( );
  filtered->points.clear( );
  // Interpolate points in the layer based on nearest neighbor distances
  // Loop through the entire cloud
  #pragma omp parallel for schedule( guided )
  for ( unsigned int i = 0; i < vf_layer_->points.size( ); ++i )
  {
    // Check to make sure there aren't any NaNs in the point
    if ( !std::isnan( vf_layer_->points[ i ].x ) &&
         !std::isnan( vf_layer_->points[ i ].y ) &&
         !std::isnan( vf_layer_->points[ i ].z ) &&
         !std::isnan( vf_layer_->points[ i ].normal_x ) &&
         !std::isnan( vf_layer_->points[ i ].normal_y ) &&
         !std::isnan( vf_layer_->points[ i ].normal_z ) &&
         !std::isnan( vf_layer_->points[ i ].normal_y ) &&
         !std::isnan( vf_layer_->points[ i ].curvature ) )
    {
      // Multithreaded radius search variables
      std::vector< int > pt_id_neighbor_search;
      std::vector< float > pt_neighbor_squared_distance;
      pcl::KdTreeFLANN< pcl::PointNormal > kdtree;
      // Set cloud input
      kdtree.setInputCloud( vf_layer_ );
      // Get r radius neighbors
      kdtree.radiusSearch( vf_layer_->points[ i ], r_size,
                           pt_id_neighbor_search,
                           pt_neighbor_squared_distance );
      // Get k nearest neighbors
      //kdtree.nearestKSearch( vf_layer_->points[ i ],
      //                     vf_layer_->points.size( ) * 0.10,
      //                     pt_id_neighbor_search,
      //                     pt_neighbor_squared_distance );
      // Check neighbors exist and point is in low density section of cloud
      if ( pt_id_neighbor_search.size( ) > 0 )// &&
           //pt_id_neighbor_search.size( ) < vf_layer_->points.size( ) * 0.10 )
      {
        // Loop through all neighbors
        for ( unsigned int j = 0; j < pt_id_neighbor_search.size( ); ++j )
        {
          // If the neighbor is too far away add a new point in between
          if ( sqrt( pt_neighbor_squared_distance[ j ] ) > intralayer_dist )
          {
            // Variable for adding new points
            pcl::PointNormal new_point;
            float v_dist =
              sqrtf ( pow ( ( vf_layer_->
                                points[ pt_id_neighbor_search[ j ] ].normal_x -
                              vf_layer_->points[ i ].normal_x  ), 2 ) +
                      pow ( ( vf_layer_->
                                points[ pt_id_neighbor_search[ j ] ].normal_y -
                              vf_layer_->points[ i ].normal_y  ), 2 ) +
                      pow ( ( vf_layer_->
                                points[ pt_id_neighbor_search[ j ] ].normal_z -
                              vf_layer_->points[ i ].normal_z  ), 2 ) );
            // Check distance between normals
            if ( v_dist < sqrt( 2.0 ) )
            {
              // Create new point
              new_point.x = ( vf_layer_->points[ pt_id_neighbor_search[ j ] ].x +
                              vf_layer_->points[ i ].x ) * 0.5;
              new_point.y = ( vf_layer_->points[ pt_id_neighbor_search[ j ] ].y +
                              vf_layer_->points[ i ].y ) * 0.5;
              new_point.z = ( vf_layer_->points[ pt_id_neighbor_search[ j ] ].z +
                              vf_layer_->points[ i ].z ) * 0.5;
              // Invert normal
              new_point.normal_x =
                ( vf_layer_->points[ pt_id_neighbor_search[ j ] ].normal_x +
                  vf_layer_->points[ i ].normal_x ) * 0.5;
              new_point.normal_y =
                ( vf_layer_->points[ pt_id_neighbor_search[ j ] ].normal_y +
                  vf_layer_->points[ i ].normal_y ) * 0.5;
              new_point.normal_z =
                ( vf_layer_->points[ pt_id_neighbor_search[ j ] ].normal_z +
                  vf_layer_->points[ i ].normal_z ) * 0.5;
              new_point.curvature =
                ( vf_layer_->points[ pt_id_neighbor_search[ j ] ].curvature +
                  vf_layer_->points[ i ].curvature ) * 0.5;
              if ( !std::isnan( new_point.x ) &&
                   !std::isnan( new_point.y ) &&
                   !std::isnan( new_point.z ) &&
                   !std::isnan( new_point.normal_x ) &&
                   !std::isnan( new_point.normal_y ) &&
                   !std::isnan( new_point.normal_z ) &&
                   !std::isnan( new_point.normal_y ) &&
                   !std::isnan( new_point.curvature ) )
              {
                #pragma omp critical ( push_addition )
                {
                  additions->points.push_back( new_point );
                }
              }
            }
            else
            {
              // Angle between normals is too high, ignore
            }
          }
        }
      }
    }
  }
  *vf_layer_ += *additions;
  interpolated_size = vf_layer_->points.size( );
  // Resize cloud to save
  vf_layer_->width = 1;
  vf_layer_->height = vf_layer_->points.size( );
  // Voxel filter point cloud
  voxel_filter.setInputCloud( vf_layer_ );
  voxel_filter.setLeafSize( leaf_size, leaf_size, leaf_size );
  voxel_filter.filter( *filtered );
  // Clear any existing points
  vf_layer_->points.clear( );
  pcl::copyPointCloud( *filtered, *vf_layer_ );
  // Output information
  //std::cout << "Original cloud: " << original_size <<
  //             " Interpolated size: " << interpolated_size <<
  //             " Filtered size: " << vf_layer_->points.size( ) << std::endl;
  return vf_layer_->points.size( );
}

double PCNLayers::CheckResolution( double &intralayer_dist )
{
  // Resolution calculation variables
  double layer_total, layer_mean;
  // K nearest or R radius search variables
  std::vector< int > pt_id_neighbor_search, num_neighbors;
  std::vector< float > pt_neighbor_squared_distance;
  // Set the search tree input cloud
  kdtree_.setInputCloud( vf_layer_ );
  // Loop through the entire cloud
  for ( unsigned int i = 0; i < vf_layer_->points.size( ); ++i )
  {
    // Check to make sure there aren't any NaNs in the point
    if ( !std::isnan( vf_layer_->points[ i ].x ) &&
         !std::isnan( vf_layer_->points[ i ].y ) &&
         !std::isnan( vf_layer_->points[ i ].z ) &&
         !std::isnan( vf_layer_->points[ i ].normal_x ) &&
         !std::isnan( vf_layer_->points[ i ].normal_y ) &&
         !std::isnan( vf_layer_->points[ i ].normal_z ) &&
         !std::isnan( vf_layer_->points[ i ].normal_y ) &&
         !std::isnan( vf_layer_->points[ i ].curvature ) )
    {
      kdtree_.radiusSearch( vf_layer_->points[ i ], 3.0 * intralayer_dist,
                            pt_id_neighbor_search,
                            pt_neighbor_squared_distance );
      num_neighbors.push_back( pt_id_neighbor_search.size( ) );
    }
  }
  // Calculate the total number of neighbors
  layer_total = std::accumulate( num_neighbors.begin( ),
                                 num_neighbors.end( ), 0.0 );
  // Calculate the average number of neighbors
  layer_mean = layer_total / vf_layer_->points.size( );
  //std::cout << "Cloud size: " << vf_layer_->points.size( ) <<
  //             " Total neighbors: " << layer_total <<
  //             " Average: " << layer_mean << std::endl;
  return layer_mean;
}

void PCNLayers::ConvertToMesh( const unsigned int i )
{
  /*
  This function creates a mesh of the VF layer by using Poisson surface
  reconstruction after copying the VF layer and inverting the surface normals.
  */
  // Construct filename
  //std::string iterator = std::to_string( i );
  std::string filename = vf_path_ + "_" + std::to_string( i );
  // Initialize objects
  pcl::Poisson< pcl::PointNormal > poisson;
  pcl::PolygonMesh mesh;
  // Clear any existing points
  layer_mesh_->points.clear( );
  // Copy point cloud
  pcl::copyPointCloud( *vf_layer_, *layer_mesh_ );
  // Loop through point cloud
  for ( unsigned int i = 0; i < layer_mesh_->points.size( ); ++i )
  {
    // Invert surface normals
    layer_mesh_->points[ i ].normal_x = -layer_mesh_->points[ i ].normal_x;
    layer_mesh_->points[ i ].normal_y = -layer_mesh_->points[ i ].normal_y;
    layer_mesh_->points[ i ].normal_z = -layer_mesh_->points[ i ].normal_z;
    layer_mesh_->points[ i ].curvature = -layer_mesh_->points[ i ].curvature;
  }
  // Resize cloud to save
  layer_mesh_->width = 1;
  layer_mesh_->height = layer_mesh_->points.size( );
  // Perform Poisson surface reconstruction
  poisson.setDepth ( 9 );
  poisson.setInputCloud( layer_mesh_ );
  poisson.reconstruct( mesh );
  // Save mesh
  pcl::io::saveOBJFile( filename + "_mesh.obj", mesh );
  return;
}

void PCNLayers::ViewVFLayer( )
{
  // visualize the mesh and surface normals
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileOBJ( mesh_path_, mesh );
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointNormal >
    red( cloud_with_normals_, 255, 0, 0 );
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointNormal >
    blue( vf_layer_, 0, 0, 255 );
  boost::shared_ptr< pcl::visualization::PCLVisualizer >
    viewer( new pcl::visualization::PCLVisualizer( "3D Viewer" ) );

  viewer->setBackgroundColor( 0, 0, 0 );
  viewer->addCoordinateSystem( 0.5 );
  // viewer->initCameraParameters( );
  viewer->addPointCloud< pcl::PointNormal > ( cloud_with_normals_, red,
                                              "part cloud" );
  viewer->setPointCloudRenderingProperties
    ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "part cloud" );
  viewer->addPointCloudNormals< pcl::PointNormal, pcl::PointNormal >
    ( cloud_with_normals_, cloud_with_normals_, 1, 0.25, "normals" );

  viewer->addPointCloud< pcl::PointNormal > ( vf_layer_, blue, "TVF layer" );
  viewer->setPointCloudRenderingProperties
    ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "TVF layer" );
  viewer->addPointCloudNormals< pcl::PointNormal, pcl::PointNormal >
    ( vf_layer_, vf_layer_, 1, 0.25, "TVF normals" );

  viewer->addPolygonMesh( mesh, "Meshes", 0 );
  while ( !viewer->wasStopped( ) )
  {
    viewer->spinOnce( 100 );
  }
}

void PCNLayers::ViewCloudNormals( double normal_length )
{
  pcl::PointCloud< pcl::PointNormal >::Ptr
    layer_1 ( new pcl::PointCloud< pcl::PointNormal > ),
    layer_2 ( new pcl::PointCloud< pcl::PointNormal > ),
    layer_3 ( new pcl::PointCloud< pcl::PointNormal > );

  std::string filename_1 = vf_path_ + "_1.pcd",
    filename_2 = vf_path_ + "_2.pcd",
    filename_3 = vf_path_ + "_3.pcd";
  std::cout << "Loading point clouds: " <<
    filename_1 << filename_2 << filename_3 << std::endl;
  pcl::io::loadPCDFile< pcl::PointNormal > ( filename_1, *layer_1 );
  pcl::io::loadPCDFile< pcl::PointNormal > ( filename_2, *layer_2 );
  //pcl::io::loadPCDFile< pcl::PointNormal > ( filename_3, *layer_3 );
  std::cout << "Point clouds loaded" << std::endl;
  // visualize the mesh and surface normals
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileOBJ( mesh_path_, mesh );
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointNormal >
    red( cloud_with_normals_, 255, 0, 0 );
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointNormal >
    green( layer_1, 0, 255, 0 );
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointNormal >
    blue( layer_2, 0, 0, 255 );
  //pcl::visualization::PointCloudColorHandlerCustom< pcl::PointNormal >
  //  blue_3( layer_3, 0, 255, 0 );
  boost::shared_ptr< pcl::visualization::PCLVisualizer >
    viewer( new pcl::visualization::PCLVisualizer( "3D Viewer" ) );

  viewer->setBackgroundColor( 0, 0, 0 );
  viewer->addCoordinateSystem( 0.5 );
  // viewer->initCameraParameters( );
  viewer->addPointCloud< pcl::PointNormal > ( cloud_with_normals_, red,
                                              "part cloud" );
  viewer->setPointCloudRenderingProperties
    ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "part cloud" );
  viewer->addPointCloudNormals< pcl::PointNormal, pcl::PointNormal >
    ( cloud_with_normals_, cloud_with_normals_, 2, normal_length, "normals" );

  viewer->addPointCloud< pcl::PointNormal > ( layer_1, green, "TVF layer 1" );
  viewer->setPointCloudRenderingProperties
    ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "TVF layer 1" );
  viewer->addPointCloudNormals< pcl::PointNormal, pcl::PointNormal >
    ( layer_1, layer_1, 1, normal_length, "TVF normals 1" );

  viewer->addPointCloud< pcl::PointNormal > ( layer_2, blue, "TVF layer 2" );
  viewer->setPointCloudRenderingProperties
    ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "TVF layer 2" );
  viewer->addPointCloudNormals< pcl::PointNormal, pcl::PointNormal >
    ( layer_2, layer_2, 1, normal_length, "TVF normals 2" );

  //viewer->addPointCloud< pcl::PointNormal > ( layer_3, blue_3, "TVF layer 3" );
  //viewer->setPointCloudRenderingProperties
  //  ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "TVF layer 3" );
  //viewer->addPointCloudNormals< pcl::PointNormal, pcl::PointNormal >
  //  ( layer_3, layer_3, 1, normal_length, "TVF normals 3" );

  viewer->addPolygonMesh( mesh, "Meshes", 0 );
  while ( !viewer->wasStopped( ) )
  {
    viewer->spinOnce( 100 );
  }
}

void PCNLayers::SaveVFLayer( const unsigned int i )
{
  std::string iterator = std::to_string( i );
  std::string filename = vf_path_ + "_" + iterator + ".pcd";
  pcl::io::savePCDFileASCII( filename, *vf_layer_ );
}

void PCNLayers::SaveVFMod( const unsigned int layer )
{
  // Function to save layer data to a .mod file for ABB robot use
  double scale_units = 1; // meters to millimeters
  unsigned int number_nans = 0;
  std::string rob_str,
    filename = vf_path_ + "_" + std::to_string( layer ) + ".mod";
  Eigen::Quaterniond quaternion, q1, q2;
  Eigen::Vector3d up_vector( 1.0, 0.0, 0.0 ), normal_vector;
  // Filename parameters
  ofstream mod_file;
  mod_file.open( filename );
  mod_file << "MODULE mTVF(SYSMODULE)\n";
  // Write constant robtargets
  for ( unsigned int i = 0; i < vf_layer_->points.size( ); ++i )
  {
    // Get rotation between two vectors
    normal_vector << ( Eigen::Vector3d( ) <<
                       vf_layer_->points[ i ].normal_x,
                       vf_layer_->points[ i ].normal_y,
                       vf_layer_->points[ i ].normal_z ).finished( );
    quaternion = Eigen::Quaterniond( ).setFromTwoVectors( up_vector,
                                                          normal_vector );
    // -90 degree rotation around Y
    q1 = Eigen::Quaterniond( sqrtf( 0.5 ), 0.0, sqrtf( 0.5 ), 0.0 );
    // 180 degree rotation around Z
    q2 = Eigen::Quaterniond( 0.0, 0.0, 0.0, 1.0 );
    // Convert from x toward surface, z up to z toward surface, x up
    quaternion = quaternion * q1 * q2;
    if ( !std::isnan( quaternion.x( ) ) && !std::isnan( quaternion.y( ) ) &&
         !std::isnan( quaternion.x( ) ) && !std::isnan( quaternion.y( ) ) )
    {
      // Build string
      rob_str = "    CONST robtarget pTVF" + std::to_string( i - number_nans ) + ":=[[" +
        std::to_string( scale_units * vf_layer_->points[ i ].x ) + "," +
        std::to_string( scale_units * vf_layer_->points[ i ].y ) + "," +
        std::to_string( scale_units * vf_layer_->points[ i ].z ) +
        "],[" + std::to_string( quaternion.w( ) ) + "," +
        std::to_string( quaternion.x( ) ) + "," +
        std::to_string( quaternion.y( ) ) + "," +
        std::to_string( quaternion.z( ) ) + "]," +
        "[0,0,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\n";
      // Write string
      mod_file << rob_str;
    }
    else
    {
      // Skip this point
      ++number_nans;
    }
  }
  // Write routine with joint moves for each point
  mod_file << "    PROC rTVF()\n";
  for ( unsigned int i = 0;
        i < vf_layer_->points.size( ) - number_nans; ++i )
  {

    mod_file << "        MoveJ pTVF" + std::to_string( i ) +
                ",v1000,z0,PlasmaPen\\WObj:=wobjTable;\n";
  }
  mod_file << "    ENDPROC\n";
  // Finish writing file and close
  mod_file << "ENDMODULE\n";
  mod_file.close( );
}

pcl::PointCloud< pcl::PointNormal >::Ptr PCNLayers::GetVFLayer( )
{
  return vf_layer_;
}

}