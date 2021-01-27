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
Doc = This code is part of the TVF generation pipeline. This class converts a
vector of PCL point clouds in a graph structure storing layer, curvature,
label, and pose stamped information. It also contains functions for saving and
load properly defined graph structures.

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


#include <tvf_graph.h>
// Include STL class
#include <stl_pcn.h>
// Eigen includes
#include <Eigen/Geometry>
// ROS and ROS message includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
// OMP for multithreading
#include <omp.h>

namespace tvf_graph
{

TVFGraph::TVFGraph( std::vector < pcl::PointCloud<pcl::PointNormal >::Ptr,
  Eigen::aligned_allocator < pcl::PointCloud< pcl::PointNormal >::Ptr > >
  pc_vector ) : vf_pcs_ ( pc_vector )
{
  // Set omp thread limit
  omp_set_dynamic( 0 );
  // Get current number of threads
  unsigned int thread_num = omp_get_max_threads( );
  // Use one less than maximum
  omp_set_num_threads( thread_num - 1 );
}

TVFGraph::~TVFGraph( )
{

}

void TVFGraph::LoadParams( std::string id )
{
  // Layer calculation variables
  double stl_multiplier;
  // Normals vectors for testing if normal average is withing tolerance
  std::vector< double > stl_normals = { 0.0, 0.0, 0.0, 0.0 };
  std::string package_name, folder_path;
  // ROS parameter names
  std::string surface_str = "/surface_" + id + "/";
  std::string package_name_str = surface_str + "package_name";
  std::string folder_path_str = surface_str + "folder_path";
  std::string vf_frame_str = surface_str + "name";
  std::string min_offset_str = surface_str + "min_offset";
  std::string max_offset_str = surface_str + "max_offset";
  std::string interlayer_dist_str = surface_str + "interlayer_dist";
  std::string stl_multiplier_str = surface_str + "stl_multiplier";
  // Get ROS parameter values
  if ( !ros::param::get( package_name_str, package_name ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << package_name_str );
  }
  if ( !ros::param::get( folder_path_str, folder_path ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << folder_path_str );
  }
  if ( !ros::param::get( vf_frame_str, vf_frame_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << vf_frame_str );
  }
  if ( !ros::param::get( min_offset_str, min_offset_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << min_offset_ );
  }
  if ( !ros::param::get( max_offset_str, max_offset_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << max_offset_str );
  }
  if ( !ros::param::get( interlayer_dist_str, interlayer_dist_ ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << interlayer_dist_str );
  }
  if ( !ros::param::get( stl_multiplier_str, stl_multiplier ) )
  {
    ROS_ERROR_STREAM( "Failed to get param " << stl_multiplier_str );
  }
  // Set the file path
  file_path_ = ros::package::getPath( package_name ) + folder_path +
    vf_frame_;
  // Create cloud with normals to be passed into classes
  pcl::PointCloud< pcl::PointNormal >::Ptr
    input_cloud( new pcl::PointCloud< pcl::PointNormal > );
  // Instantiate normal vector corrector class
  stl_pcn::STLPCN stl_pcn( input_cloud );
  // Set the file paths
  stl_pcn.SetFilePaths( file_path_, suffix_ );
  // Load the STL
  stl_pcn.LoadSTL( );
  // Set the file paths and load the STL
  stl_normals = stl_pcn.AverageSTLNormals( );
  // Interpolate STL based on task intralayer distance
  intralayer_dist_ = stl_normals[ 3 ] * stl_multiplier;
  // Fill frame id
  boost::get_property( tvf_graph_, boost::graph_name ) = vf_frame_;
  // Clear vectors
  intralayer_mins_.clear( );
  intralayer_weights_.clear( );
  interlayer_weights_.clear( );
  layer_sizes_.clear( );
}

void TVFGraph::Run( unsigned int layers, std::string suffix )
{
  suffix_ = suffix;
  // Clear vectors
  intralayer_mins_.clear( );
  intralayer_weights_.clear( );
  interlayer_weights_.clear( );
  layer_sizes_.clear( );
  // Load the point clouds
  LoadPC( layers );
  // Convert to graph structure
  CloudToGraph( );
  // Save graph
  std::string layer = "_" + std::to_string( layers );
  SaveGraph( layer );
}

void TVFGraph::LoadPC( unsigned int layers )
{
  // Set file names
  std::string vf_pcs_path = file_path_ + suffix_ + "_vf_layer";
  vf_pcs_.clear( );
  // Iterate through the provided number of layers
  for ( unsigned int layer = 0; layer < layers; ++layer )
  {
    // Loading PCD file
    std::string filename = vf_pcs_path + "_" + std::to_string( layer + 1 ) +
      ".pcd";
    ROS_INFO_STREAM( "VF file: " << filename );
    pcl::PointCloud< pcl::PointNormal >::Ptr
      vf_pc ( new pcl::PointCloud< pcl::PointNormal > );
    if ( pcl::io::loadPCDFile< pcl::PointNormal > ( filename, *vf_pc ) == -1 )
    {
      PCL_ERROR ( "Couldn't read file" );
      return;
    }
    ROS_INFO_STREAM( "Pushing back pcs vector: " << vf_pc->points.size( ) <<
                     " points." );
    // Add file PC to the vector of PCs
    vf_pcs_.push_back( vf_pc );
    ROS_INFO_STREAM( "Point Cloud " << layer + 1 << " has "
                    << vf_pcs_[ layer ]->points.size( ) << " points." );
  }
}

void TVFGraph::LoadGraph( )
{
  // Empty the graph before making a new one
  tvf_graph_.clear( );
  // Fill property maps defined in the header
  // Use ref_property_map to turn a graph property into a property map
  GraphNameMapString graph_name_map_string =
    boost::get_property( tvf_graph_string_, boost::graph_name );
  IndexMapString index_map_string =
    boost::get( boost::vertex_index, tvf_graph_string_ );
  IdMapString id_map_string =
    boost::get( &VertexPropertiesString::id, tvf_graph_string_ );
  LayerMapString layer_map_string =
    boost::get( &VertexPropertiesString::layer, tvf_graph_string_ );
  CurvatureMapString curvature_map_string =
    boost::get( &VertexPropertiesString::curvature, tvf_graph_string_ );
  LabelMapString label_map_string =
    boost::get( &VertexPropertiesString::label, tvf_graph_string_ );
  PoseMapString pose_map_string =
    boost::get( &VertexPropertiesString::pose_stamped, tvf_graph_string_ );
  EdgeWeightMapString edge_weight_map_string =
    boost::get( boost::edge_weight, tvf_graph_string_ );

  // Set up dynamic properties
  boost::dynamic_properties dyprops(boost::ignore_other_properties);
  dyprops.property( "name", graph_name_map_string );
  dyprops.property( "node_id", index_map_string );
  dyprops.property( "id", id_map_string );
  dyprops.property( "layer", layer_map_string );
  dyprops.property( "curvature", curvature_map_string );
  dyprops.property( "label", label_map_string );
  dyprops.property( "pose_stamped", pose_map_string );
  dyprops.property( "edge_weight", edge_weight_map_string );

  std::ifstream load_file( file_path_ + graph_file_ + ".dot" );
  ROS_INFO_STREAM( "Loading file: " + file_path_ + graph_file_ + ".dot" );

  // If load functions correctly
  if ( boost::read_graphviz( load_file, tvf_graph_string_, dyprops, "id" ) )
  {
    // Create index map for pose stamped graph
    IndexMap index_map = boost::get( boost::vertex_index, tvf_graph_ );
    // Create edge variables for the string pose stamped graph
    std::pair< UndirectedGraphString::edge_descriptor, bool > edge_pair_string;
    UndirectedGraphString::edge_descriptor edge_string;

    // Output number of vertices and edges
    ROS_INFO_STREAM( "Pose stamped string graph loaded, name: "
                    << boost::get_property( tvf_graph_string_,
                                            boost::graph_name )
                    << " Verticies: " << boost::num_vertices( tvf_graph_string_ )
                    << " Edges: " << boost::num_edges( tvf_graph_string_ ) );
    ROS_INFO_STREAM( "Copying graph data." );
    // Copy over graph name
    boost::get_property( tvf_graph_, boost::graph_name ) =
      boost::get_property( tvf_graph_string_, boost::graph_name );
    ROS_INFO_STREAM( "Copying graph vertices." );
    // Copy over vertices
    for ( unsigned int index = 0;
          index < boost::num_vertices( tvf_graph_string_ ); ++index )
    {
      boost::add_vertex( tvf_graph_ );
      // Set id to index instead of previous vertex id
      tvf_graph_[ index ].id = index;
      tvf_graph_[ index ].layer = tvf_graph_string_[ index ].layer;
      tvf_graph_[ index ].curvature = tvf_graph_string_[ index ].curvature;
      tvf_graph_[ index ].label = tvf_graph_string_[ index ].label;
      StringToPoseStamped( index );
    }
    ROS_INFO_STREAM( "Copying graph edges." );
    // Copy over edges
    for ( unsigned int index = 0;
          index < boost::num_vertices( tvf_graph_string_ ); ++index )
    {
      // Get a list of neighbors from the string pose stamped graph
      neighbors_string_ =
        boost::adjacent_vertices( vertex( index, tvf_graph_string_ ),
                                  tvf_graph_string_ );
      for( ; neighbors_string_.first != neighbors_string_.second;
            ++neighbors_string_.first )
      {
        // Get edge and weight value
        edge_pair_string =
          boost::edge( index,
                       index_map_string [ *neighbors_string_.first ],
                       tvf_graph_string_ );
        edge_string = edge_pair_string.first;
        // Check to see if edge is already present in pose stamped graph
        if ( boost::edge( index,
                          index_map[ *neighbors_string_.first ],
                          tvf_graph_ ).second == 0 )
        {
          // Add edge to pose stamped graph
          boost::add_edge( index,
                           index_map[ *neighbors_string_.first ],
                           edge_weight_map_string[ edge_string ],
                           tvf_graph_ );
        }
        else
        {
          // Do nothing, the edge is already in the graph
        }
      }
    }
    // Output number of vertices and edges
    ROS_INFO_STREAM( "Pose stamped graph recovered, name: "
                    << boost::get_property( tvf_graph_,
                                            boost::graph_name )
                    << " Verticies: " << boost::num_vertices( tvf_graph_ )
                    << " Edges: " << boost::num_edges( tvf_graph_ ) );
  }
  else
  {
    ROS_ERROR_STREAM( "Error loading file." );
  }
  // Fill index map and edge map
  index_map_ = boost::get( boost::vertex_index, tvf_graph_ );
  edge_weight_map_ = boost::get( boost::edge_weight, tvf_graph_ );
}

void TVFGraph::CloudToGraph( )
{
  // Empty the graph before making a new one
  tvf_graph_.clear( );
  // Keep record of the total number of graph vertices through the PC layers
  unsigned int number_nans = 0;
  std::vector< double > layer_vector;
  // Iterate through the vector of PCs
  for ( unsigned int layer = 0; layer < vf_pcs_.size( ); layer++ )
  {
    intralayer_weights_.push_back( layer_vector );
    intralayer_mins_.push_back( layer_vector );
    number_nans += AddVertices( layer );
    // Link the nearest points in the previous and current VF layers
    if ( layer == 0 )
    {
      layer_sizes_.push_back( boost::num_vertices( tvf_graph_ ) );
    }
    else if ( layer > 0 )
    {
      layer_sizes_.push_back( boost::num_vertices( tvf_graph_ ) -
                              std::accumulate( layer_sizes_.begin( ),
                                               layer_sizes_.end( ), 0.0 ) );
      interlayer_weights_.push_back( layer_vector );
      LinkLayersOut( layer );
      LinkLayersIn( layer );
    }
    ROS_INFO_STREAM( "Layer: " << layer + 1 <<
                     " Number of NaNs: " << number_nans <<
                     " Layer size: " << layer_sizes_[ layer ] );
  }
  ROS_INFO_STREAM( "Done converting PC vector to graph." );
  // Fill index map and edge map
  index_map_ = boost::get( boost::vertex_index, tvf_graph_ );
  edge_weight_map_ = boost::get( boost::edge_weight, tvf_graph_ );
}

void TVFGraph::StringToPoseStamped( const unsigned int i )
{
  // Initialize pose stamped message and fill in current time information
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now( );
  // Specified variable beginning and end delimiters
  std::string delimiter_1 = ":", delimiter_2 = ",";
  // Other variables for string segmenting
  size_t position_1, position_2;
  std::string data_string;
  std::string::size_type sz;
   // Iterator to pull out different data types
  unsigned int iter = 0;
  // Vector for storage of position and orientation information
  std::vector< double > data = { };
  // Bring in the string from the loaded pose stamped string graph to convert
  // back into a pose stamped message.
  std::istringstream iss( tvf_graph_string_[ i ].pose_stamped );
  // Loop through the lines in the pose stamped string
  for ( std::string line; std::getline( iss, line ); )
  {
    // Clear delimiter position variables
    position_1 = 0;
    position_2 = 0;
    // If fist line of information pull out sequence data, this was also the
    // vertex name in the saved graph
    if ( iter == 0 )
    {
      if ( ( position_1 = line.find( delimiter_1 ) ) != std::string::npos )
      {
        if ( ( position_2 = line.find( delimiter_2 ) ) != std::string::npos )
        {
          pose_stamped.header.seq =
            std::stoul( line.substr( position_1 + delimiter_1.length( ),
                                     position_2 - position_1 -
                                     delimiter_2.length( ) ), &sz );
        }
        else
        {
          ROS_ERROR_STREAM( "Delimiter 2 not found." );
        }
      }
      else
      {
        ROS_ERROR_STREAM( "Delimiter 1 not found." );
      }
    }
    else if ( iter == 1 )
    {
      if ( ( position_1 = line.find( delimiter_1 ) ) != std::string::npos )
      {
        if ( ( position_2 = line.find( delimiter_2 ) ) != std::string::npos )
        {
          pose_stamped.header.frame_id =
            line.substr( position_1 + delimiter_1.length( ),
                         position_2 - position_1 - delimiter_2.length( ) );
        }
        else
        {
          ROS_ERROR_STREAM( "Delimiter 2 not found." );
        }
      }
      else
      {
        ROS_ERROR_STREAM( "Delimiter 1 not found." );
      }
    }
    else
    {
      if ( ( position_1 = line.find( delimiter_1 ) ) != std::string::npos )
      {
        if ( ( position_2 = line.find( delimiter_2 ) ) != std::string::npos )
        {
          data_string =
            line.substr( position_1 + delimiter_1.length( ),
            position_2 - position_1 - delimiter_2.length( ) );
          data.push_back( std::stod( data_string, &sz ) );
        }
        else
        {
          ROS_ERROR_STREAM( "Delimiter 2 not found." );
        }
      }
      else
      {
        ROS_ERROR_STREAM( "Delimiter 1 not found." );
      }
    }
    iter += 1;
  }
  // Fill in position
  pose_stamped.pose.position.x = data[ 0 ];
  pose_stamped.pose.position.y = data[ 1 ];
  pose_stamped.pose.position.z = data[ 2 ];
  // Fill in orientation
  pose_stamped.pose.orientation.x = data[ 3 ];
  pose_stamped.pose.orientation.y = data[ 4 ];
  pose_stamped.pose.orientation.z = data[ 5 ];
  pose_stamped.pose.orientation.w = data[ 6 ];
  // Load into pose stamped into graph structure
  tvf_graph_[ i ].pose_stamped = pose_stamped;
}

unsigned int TVFGraph::AddVertices( const unsigned int layer )
{
  unsigned int number_nans = 0;
  ROS_INFO_STREAM( "Point Cloud " << layer << " has "
                    << vf_pcs_[ layer ]->points.size( ) << " points." );
  // Loop through point cloud
  for ( unsigned int index = 0; index < vf_pcs_[ layer ]->points.size( );
        ++index )
  {
    if ( std::isnan( vf_pcs_[ layer ]->points[ index ].x ) ||
         std::isnan( vf_pcs_[ layer ]->points[ index ].y ) ||
         std::isnan( vf_pcs_[ layer ]->points[ index ].z ) ||
         std::isnan( vf_pcs_[ layer ]->points[ index ].normal_x ) ||
         std::isnan( vf_pcs_[ layer ]->points[ index ].normal_y ) ||
         std::isnan( vf_pcs_[ layer ]->points[ index ].normal_z ) )
    {
     // Skip this point
     ++number_nans;
    }
    else
    {
      // Store the Stamped Pose for this index in the graph structure
      boost::add_vertex( tvf_graph_ );
      // Get the index for the last vertex
      total_vertices_ = boost::num_vertices( tvf_graph_ ) - 1;
      tvf_graph_[ total_vertices_ ].id = total_vertices_;
      tvf_graph_[ total_vertices_ ].layer = layer;
      tvf_graph_[ total_vertices_ ].curvature =
        vf_pcs_[ layer ]->points[ index ].curvature;
      PCToPoseStamped( layer, index );
      // Add edges to the rest of this VF layer already present in the graph
      AddEdge( layer, index - number_nans );
    }
  }
  return number_nans;
}

void TVFGraph::PCToPoseStamped( const unsigned int layer,
                                const unsigned int index )
{
  // Get the index for the last vertex
  total_vertices_ = boost::num_vertices( tvf_graph_ ) - 1;
  // Initialize function variables
  geometry_msgs::PoseStamped pose_stamped;
  Eigen::Quaterniond quaternion;
  Eigen::Vector3d up_vector( 1.0, 0.0, 0.0 ), normal_vector;
  // Update header
  pose_stamped.header.stamp = ros::Time::now( );
  pose_stamped.header.frame_id = vf_frame_;
  pose_stamped.header.seq = total_vertices_;
  // Fill in position
  pose_stamped.pose.position.x = vf_pcs_[ layer ]->points[ index ].x;
  pose_stamped.pose.position.y = vf_pcs_[ layer ]->points[ index ].y;
  pose_stamped.pose.position.z = vf_pcs_[ layer ]->points[ index ].z;
  // Get rotation between two vectors
  normal_vector << ( Eigen::Vector3d( )
                << vf_pcs_[ layer ]->points[ index ].normal_x,
                   vf_pcs_[ layer ]->points[ index ].normal_y,
                   vf_pcs_[ layer ]->points[ index ].normal_z ).finished( );
  quaternion = Eigen::Quaterniond( ).setFromTwoVectors( up_vector,
                                                        normal_vector );
  // Fill in orientation
  pose_stamped.pose.orientation.x = quaternion.x( );
  pose_stamped.pose.orientation.y = quaternion.y( );
  pose_stamped.pose.orientation.z = quaternion.z( );
  pose_stamped.pose.orientation.w = quaternion.w( );
  // Store pose stamped in graph structure
  tvf_graph_[ total_vertices_ ].pose_stamped = pose_stamped;
}

void TVFGraph::AddEdge( const unsigned int layer, const unsigned int index )
{
  unsigned int counter = 0;
  double angle, dist, min = big_number_;
  tf::Quaternion quat_1, quat_2, quat_3;
  // Get the index for the last vertex
  total_vertices_ = boost::num_vertices( tvf_graph_ ) - 1;
  // Add edges to the rest of this VF layer already present in the graph
  for ( unsigned int iii = total_vertices_ - index;
        iii < total_vertices_; ++iii )
  {
    // Calculate the angle between
    dist =
      sqrtf ( pow( tvf_graph_[ iii ].pose_stamped.pose.position.x -
                   tvf_graph_[ total_vertices_ ].pose_stamped.pose.position.x,
                   2.0 ) +
              pow( tvf_graph_[ iii ].pose_stamped.pose.position.y -
                   tvf_graph_[ total_vertices_ ].pose_stamped.pose.position.y,
                   2.0 ) +
              pow( tvf_graph_[ iii ].pose_stamped.pose.position.z -
                   tvf_graph_[ total_vertices_ ].pose_stamped.pose.position.z,
                   2.0 ) );
    /*
    quaternionMsgToTF( tvf_graph_[ iii ].pose_stamped.pose.orientation, quat_1);
    quaternionMsgToTF(
      tvf_graph_[ total_vertices_ ].pose_stamped.pose.orientation, quat_2);
    angle = quat_1.angleShortestPath( quat_2 );
    */
    // Add to intralayer edge weights
    intralayer_weights_[ layer ].push_back( dist );
    boost::add_edge( total_vertices_, iii, dist, tvf_graph_ );
    counter ++;
    /*
    // Option for a averaged angle and distance edge weight
    boost::add_edge( total_vertices_, iii,
                     ( 0.5 * angle ) + ( 0.5 * dist ), tvf_graph );
    if ( dist < min )
    {
      min = dist;
    }
    */
  }
  /*
  std::cout << "Node: " << index << " Total - index: "
            << total_vertices_ - index
            << " Total: " << total_vertices_ << " Counter: " << counter
            << " Layer connections: " << intralayer_weights_[ layer ].size( )
            << std::endl;
  // Record minimum
  intralayer_mins_[ layer ].push_back( min );
  */
}

void TVFGraph::LinkLayersOut( const unsigned int layer )
{
  // Get the index for the last vertex
  total_vertices_ = boost::num_vertices( tvf_graph_ );
  // Link points on previous VF layer to the nearest vertex in the current VF
  // layer based on xyz distance. This could based on another distance metric.
  #pragma omp parallel for schedule( guided )
  for ( unsigned int index = total_vertices_ -
          ( layer_sizes_[ layer ] + layer_sizes_[ layer - 1 ] );
        index < ( total_vertices_ - layer_sizes_[ layer ] ); ++index )
  {
    unsigned int min_location = 0;
    double dist, min_dist = big_number_;
    for ( unsigned int iii = ( total_vertices_ - layer_sizes_[ layer ] );
          iii < total_vertices_; ++iii )
    {
      // Calculate xyz distance
      dist =
        sqrtf ( pow( tvf_graph_[ index ].pose_stamped.pose.position.x -
                     tvf_graph_[ iii ].pose_stamped.pose.position.x, 2.0 ) +
                pow( tvf_graph_[ index ].pose_stamped.pose.position.y -
                     tvf_graph_[ iii ].pose_stamped.pose.position.y, 2.0 ) +
                pow( tvf_graph_[ index ].pose_stamped.pose.position.z -
                     tvf_graph_[ iii ].pose_stamped.pose.position.z, 2.0 ) );
      // Store minimum distance
      if ( dist < min_dist )
      {
        min_location = iii;
        min_dist = dist;
      }
    }
    // Check to see if edge is already present in pose stamped graph
    if ( boost::edge( index, min_location, tvf_graph_ ).second == 0 )
    {
      // Add edge to pose stamped graph
      #pragma omp critical ( add_edge )
      {
        boost::add_edge( index, min_location, min_dist, tvf_graph_ );
        interlayer_weights_[ layer - 1 ].push_back( min_dist );
      }
    }
    else
    {
      // Do nothing, the edge is already in the graph
    }
  }
}

void TVFGraph::LinkLayersIn( const unsigned int layer )
{
  // Get the index for the last vertex
  total_vertices_ = boost::num_vertices( tvf_graph_ );
  // Link points on current VF layer to the nearest vertex in the previous VF
  // layer based on xyz distance. This could based on another distance metric.
  #pragma omp parallel for schedule( guided )
  for ( int index = ( total_vertices_ - layer_sizes_[ layer ] );
        index < total_vertices_; ++index )
  {
    unsigned int min_location = 0;
    double dist, min_dist = big_number_;
    for ( unsigned int iii = total_vertices_ -
            ( layer_sizes_[ layer ] + layer_sizes_[ layer - 1 ] );
          iii < ( total_vertices_ - layer_sizes_[ layer ] ); ++iii )
    {
      // Calculate xyz distance
      dist =
        sqrtf ( pow( tvf_graph_[ index ].pose_stamped.pose.position.x -
                     tvf_graph_[ iii ].pose_stamped.pose.position.x, 2.0 ) +
                pow( tvf_graph_[ index ].pose_stamped.pose.position.y -
                     tvf_graph_[ iii ].pose_stamped.pose.position.y, 2.0 ) +
                pow( tvf_graph_[ index ].pose_stamped.pose.position.z -
                     tvf_graph_[ iii ].pose_stamped.pose.position.z, 2.0 ) );
      // Store minimum distance
      if ( dist < min_dist )
      {
        min_location = iii;
        min_dist = dist;
      }
    }
    // Check to see if edge is already present in pose stamped graph
    if ( boost::edge( index, min_location, tvf_graph_ ).second == 0 )
    {
      // Add edge to pose stamped graph
      #pragma omp critical ( add_edge )
      {
        boost::add_edge( index, min_location, min_dist, tvf_graph_ );
        interlayer_weights_[ layer - 1 ].push_back( min_dist );
      }
    }
    else
    {
      // Do nothing, the edge is already in the graph
    }
  }
}

void TVFGraph::GetAllNeighbors( const unsigned int index,
                                TVFGraph::UndirectedGraph &graph )
{
  // Empty the graph before making a new one
  graph.clear( );
  AddVertex( index, graph );
  // Get a list of neighbors from the pose stamped graph
  neighbors_ = boost::adjacent_vertices( vertex( index, tvf_graph_ ),
                                         tvf_graph_ );
  // Iterate through the neighbors
  for ( ; neighbors_.first != neighbors_.second; ++neighbors_.first )
  {
    // Add vertex to new graph
    AddVertex( *neighbors_.first, graph );
    // Get edge value
    edge_ = boost::edge( index, index_map_[ *neighbors_.first ],
                         tvf_graph_ ).first;
    // Add edge value to new graph
    boost::add_edge( 0, boost::num_vertices( graph ) - 1,
                     edge_weight_map_[ edge_ ], graph );
  }
}

void TVFGraph::GetNearestNeighbors( const unsigned int index,
                                    const unsigned int nearest,
                                    TVFGraph::UndirectedGraph &graph )
{
  // Vector of pairs for storing nearest neighbor data
  std::vector< std::pair< double, int > > nn_data;
  std::vector< std::pair< double, int > >::iterator it;
  // Empty the graph before making a new one
  graph.clear( );
  AddVertex( index, graph );
  // Get a list of neighbors from the pose stamped graph
  neighbors_ = boost::adjacent_vertices( vertex( index, tvf_graph_ ),
                                         tvf_graph_ );
  // Iterate through the neighbors
  for ( ; neighbors_.first != neighbors_.second; ++neighbors_.first )
  {
    // Add to graph if in a different layer
    if ( graph[ 0 ].layer != tvf_graph_[ *neighbors_.first ].layer )
    {
      // Add vertex to new graph
      AddVertex( *neighbors_.first, graph );
      // Get edge value
      edge_ = boost::edge( index, index_map_[ *neighbors_.first ],
                           tvf_graph_ ).first;
      // Add edge value to new graph
      boost::add_edge( 0, boost::num_vertices( graph ) - 1,
                       edge_weight_map_[ edge_ ], graph );
    }
    else
    {
      // Get edge value
      edge_ = boost::edge( index, index_map_[ *neighbors_.first ],
                           tvf_graph_ ).first;
      if ( nn_data.size( ) < nearest )
      {
        nn_data.push_back( std::make_pair( edge_weight_map_[ edge_ ],
                                           *neighbors_.first ) );
      }
      else
      {
        // Check current edge against previously stored edges
        for ( it = nn_data.begin( ); it != nn_data.end( ); it++ )
        {
          // Check to see if current weight is less than weights in the vector
          if ( edge_weight_map_[ edge_ ] < ( *it ).first )
          {
            nn_data.insert( it, std::make_pair( edge_weight_map_[ edge_ ],
                            *neighbors_.first ) );
            // If too many neighbors have been collected delete the last
            if ( nn_data.size( ) >= nearest )
            {
              nn_data.pop_back( );
            }
            break;
          }
        }
      }
    }
  }
  // Add nearest neighbors
  for ( unsigned int i = 0; i < nn_data.size( ); ++i )
  {
    // Add vertex to new graph
    AddVertex( nn_data[ i ].second, graph );
    // Add edge value to new graph
    boost::add_edge( 0, boost::num_vertices( graph ) - 1,
                     nn_data[ i ].first, graph );
  }
}

void TVFGraph::GetScaledNeighbors( const unsigned int index,
                                   TVFGraph::UndirectedGraph &graph )
{
  // Vector of pairs for storing nearest neighbor data
  std::vector< std::pair< double, int > > nn_data;
  std::vector< std::pair< double, int > >::iterator it;
  // Empty the graph before making a new one
  graph.clear( );
  AddVertex( index, graph );
  // Get a list of neighbors from the pose stamped graph
  neighbors_ = boost::adjacent_vertices( vertex( index, tvf_graph_ ),
                                         tvf_graph_ );
  // Iterate through the neighbors
  for ( ; neighbors_.first != neighbors_.second; ++neighbors_.first )
  {
    // Get edge value
    edge_ = boost::edge( index, index_map_[ *neighbors_.first ],
                         tvf_graph_ ).first;
    // Add to graph if in a different layer
    if ( graph[ 0 ].layer != tvf_graph_[ *neighbors_.first ].layer )
    {
      // Add vertex to new graph
      AddVertex( *neighbors_.first, graph );
      // Add edge value to new graph
      boost::add_edge( 0, boost::num_vertices( graph ) - 1,
                       edge_weight_map_[ edge_ ], graph );
    }
    else
    {
      // Check to see if current weight is less than intralayer distance
      if ( edge_weight_map_[ edge_ ] <= intralayer_dist_ )
      {
        // Add vertex to new graph
        AddVertex( *neighbors_.first, graph );
        // Add edge value to new graph
        boost::add_edge( 0, boost::num_vertices( graph ) - 1,
                         edge_weight_map_[ edge_ ], graph );
      }
      else
      {
        if ( distribution_( generator_ ) *
             intralayer_dist_ / edge_weight_map_[ edge_ ] > 250 )
        {
          // Add vertex to new graph
          AddVertex( *neighbors_.first, graph );
          // Add edge value to new graph
          boost::add_edge( 0, boost::num_vertices( graph ) - 1,
                           edge_weight_map_[ edge_ ], graph );
        }
      }
    }
  }
}

void TVFGraph::AddVertex( const unsigned int index,
                          TVFGraph::UndirectedGraph &graph )
{
  // Add vertex
  boost::add_vertex( graph );
  // Get the index for the last vertex
  double vertices = boost::num_vertices( graph ) - 1;
  // Copy data into graph
  graph[ vertices ].id = tvf_graph_[ index ].id;
  graph[ vertices ].layer = tvf_graph_[ index ].layer;
  graph[ vertices ].curvature = tvf_graph_[ index ].curvature;
  graph[ vertices ].label = tvf_graph_[ index ].label;
  graph[ vertices ].pose_stamped = tvf_graph_[ index ].pose_stamped;
}

void TVFGraph::UpdateVertex( const unsigned int tvf_index,
                             const unsigned int graph_index,
                             geometry_msgs::Pose pose,
                             TVFGraph::UndirectedGraph &graph )
{
  // Update data in both graphs
  tvf_graph_[ tvf_index ].pose_stamped.pose.orientation = pose.orientation;
  graph[ graph_index ].pose_stamped.pose.orientation = pose.orientation;
}

void TVFGraph::AddPathEdges( const unsigned int index,
                             TVFGraph::UndirectedGraph &graph )
{
  // Variable to warn the user if an added vertex is not connected to
  // the rest of the path
  bool connected = false;
  // Graph iterators for pose stamped graph
  std::pair< VertexIterator, VertexIterator > vs1, vs2;
  // Loop through possible edges
  for ( vs1 = vertices( graph ); vs1.first != vs1.second; ++vs1.first )
  {
    for ( vs2 = vertices( graph ); vs2.first != vs2.second; ++vs2.first )
    {
      // Ignore same vertex edges, check for the TVF edge, check to see if
      // edge is already present in the graph
      if ( *vs1.first != *vs2.first &&
           boost::edge( graph[ *vs1.first ].id,
                        graph[ *vs2.first ].id,
                        tvf_graph_ ).second &&
           !boost::edge( *vs1.first, *vs2.first, graph ).second )
      {
        if ( boost::edge( *vs1.first, *vs2.first, graph ).second )
        {
          // Already connected
          connected = true;
        }
        else
        {
          // Add edge value to graph
          boost::add_edge( *vs1.first, *vs2.first,
                           edge_weight_map_[
                             boost::edge( graph[ *vs1.first ].id,
                                          graph[ *vs2.first ].id,
                                          tvf_graph_ ).first ], graph );
          connected = true;
          ROS_INFO_STREAM( "Path edge between " << *vs1.first << " and " <<
                           *vs2.first << " not present. New edge weight: " <<
                           edge_weight_map_[
                             boost::edge( graph[ *vs1.first ].id,
                                          graph[ *vs2.first ].id,
                                          tvf_graph_ ).first ] );
        }
      }
    }
  }
  if (  boost::num_vertices( graph ) > 1 && !connected )
  {
    ROS_WARN_STREAM( "Vertex added to path does not connect to the rest of the path." );
  }
}

bool TVFGraph::RemoveVertex( const unsigned int index,
                             TVFGraph::UndirectedGraph &graph )
{
  // Search whole graph for proper node id from end to beginning
  for ( vs_ = vertices( graph ); vs_.second != vs_.first; --vs_.second )
  {
    // Remove vertex if node id found
    if ( graph[ *vs_.second ].id == index )
    {
      boost::clear_vertex( *vs_.second, graph );
      boost::remove_vertex( *vs_.second, graph );
      return true;
    }
    else if ( boost::num_vertices( graph ) == 1 &&
              graph[ *vs_.first ].id == index )
    {
      boost::clear_vertex( *vs_.second, graph );
      boost::remove_vertex( *vs_.second, graph );
      return true;
    }
  }
  return false;
}

inline std::string
  pose_stamped_to_string( geometry_msgs::PoseStamped pose_stamped )
{
  // Convert pose stamped message to multi-line string message to save. Skip
  // time information. It is filled back in upon loading the graph.
  std::string pose_stamped_string =
    "seq:" + std::to_string( pose_stamped.header.seq ) + ",\n" +
    "frame_id:" + pose_stamped.header.frame_id + ",\n" +
    "position.x:" + std::to_string( pose_stamped.pose.position.x ) + ",\n" +
    "position.y:" + std::to_string( pose_stamped.pose.position.y ) + ",\n" +
    "position.z:" + std::to_string( pose_stamped.pose.position.z ) + ",\n" +
    "orientation.x:" + std::to_string( pose_stamped.pose.orientation.x ) +
    ",\n" +
    "orientation.y:" + std::to_string( pose_stamped.pose.orientation.y ) +
    ",\n" +
    "orientation.z:" + std::to_string( pose_stamped.pose.orientation.z ) +
    ",\n" +
    "orientation.w:" + std::to_string( pose_stamped.pose.orientation.w ) + ",";
  return pose_stamped_string;
}

void TVFGraph::SaveGraph( std::string layers )
{
  ROS_INFO_STREAM( "Saving current graph." );
  // Fill property maps defined in the header
  // Use ref_property_map to turn a graph property into a property map
  GraphNameMap graph_name_map =
    boost::get_property( tvf_graph_, boost::graph_name );
  IndexMap index_map = boost::get( boost::vertex_index, tvf_graph_ );
  IdMap id_map = boost::get( &VertexProperties::id, tvf_graph_ );
  LayerMap layer_map = boost::get( &VertexProperties::layer, tvf_graph_ );
  CurvatureMap curvature_map =
    boost::get( &VertexProperties::curvature, tvf_graph_ );
  LabelMap label_map = boost::get( &VertexProperties::label, tvf_graph_ );
  PoseMap pose_map = boost::get( &VertexProperties::pose_stamped, tvf_graph_ );
  EdgeWeightMap edge_weight_map = boost::get( boost::edge_weight, tvf_graph_ );

  // Set up dynamic properties
  boost::dynamic_properties dyprops;
  dyprops.property( "name", graph_name_map );
  dyprops.property( "node_id", index_map );
  dyprops.property( "id", id_map );
  dyprops.property( "layer", layer_map );
  dyprops.property( "curvature", curvature_map );
  dyprops.property( "label", label_map );
  dyprops.property( "pose_stamped",
                    boost::make_transform_value_property_map(
                      &pose_stamped_to_string, pose_map ) );
  dyprops.property( "edge_weight", edge_weight_map );

  // Output information to terminal
  //boost::write_graphviz_dp( std::cout, tvf_graph_, dyprops );
  // Get save file and save information
  std::ofstream save_file( file_path_ + graph_file_ + layers + ".dot" );
  boost::write_graphviz_dp( save_file, tvf_graph_, dyprops );
  ROS_INFO_STREAM( "Graph written to output file: " + file_path_ +
                   graph_file_ + suffix_ );
}

void TVFGraph::SaveInputGraph( std::string filename,
                               TVFGraph::UndirectedGraph input_tvf_graph )
{
  ROS_INFO_STREAM( "Saving input graph." );
  // Fill property maps defined in the header
  // Use ref_property_map to turn a graph property into a property map
  GraphNameMap graph_name_map =
    boost::get_property( tvf_graph_, boost::graph_name );
  IndexMap index_map = boost::get( boost::vertex_index, tvf_graph_ );
  IdMap id_map = boost::get( &VertexProperties::id, tvf_graph_ );
  LayerMap layer_map = boost::get( &VertexProperties::layer, tvf_graph_ );
  CurvatureMap curvature_map =
    boost::get( &VertexProperties::curvature, tvf_graph_ );
  LabelMap label_map = boost::get( &VertexProperties::label, tvf_graph_ );
  PoseMap pose_map = boost::get( &VertexProperties::pose_stamped, tvf_graph_ );
  EdgeWeightMap edge_weight_map = boost::get( boost::edge_weight, tvf_graph_ );

  // Set up dynamic properties
  boost::dynamic_properties dyprops;
  dyprops.property( "name", graph_name_map );
  dyprops.property( "node_id", index_map );
  dyprops.property( "id", id_map );
  dyprops.property( "layer", layer_map );
  dyprops.property( "curvature", curvature_map );
  dyprops.property( "label", label_map );
  dyprops.property( "pose_stamped",
                    boost::make_transform_value_property_map(
                      &pose_stamped_to_string, pose_map ) );
  dyprops.property( "edge_weight", edge_weight_map );

  // Output information to terminal
  //boost::write_graphviz_dp( std::cout, input_tvf_graph, dyprops );
  // Get save file and save information
  std::ofstream save_file( file_path_ + filename );
  boost::write_graphviz_dp( save_file, input_tvf_graph, dyprops );
  ROS_INFO_STREAM( "Input graph written to output file: " << file_path_ <<
                   filename );
}

void TVFGraph::SaveLayerData( )
{
  double layer_total, layer_size, layer_mean, layer_max, layer_min,
  layer_avg_min, layer_variance, layer_standard_deviation;
  //double layer_min_2, layer_min_3, layer_min_4, layer_min_5, layer_max_2;

  ofstream layer_file;
  layer_file.open( file_path_ + suffix_ + "_layer.csv" );
  layer_file << "ID: " << vf_frame_
             << ",Connections,Total,Average,Minimum,Maximum,Variance,"
             << "Standard Deviation,\n";
  for ( unsigned int i = 0; i < intralayer_weights_.size( ); i++ )
  {
    layer_total = std::accumulate( intralayer_weights_[ i ].begin( ),
                                   intralayer_weights_[ i ].end( ), 0.0 );
    layer_size = intralayer_weights_[ i ].size( );
    layer_mean = layer_total / layer_size;

    for ( unsigned int j = 0; j < intralayer_weights_[ i ].size( ); j++ )
    {
      layer_variance += ( intralayer_weights_[ i ][ j ] - layer_mean ) *
                        ( intralayer_weights_[ i ][ j ] - layer_mean );
    }
    layer_variance /= layer_size;
    layer_standard_deviation = sqrt( layer_variance );
    // Sort to check max and min
    layer_min = *std::min_element( intralayer_weights_[ i ].begin( ),
                                   intralayer_weights_[ i ].end( ) );
    layer_max = *std::max_element( intralayer_weights_[ i ].begin( ),
                                   intralayer_weights_[ i ].end( ) );
    /*
    layer_avg_min = std::accumulate( intralayer_mins_[ i ].begin( ),
                                     intralayer_mins_[ i ].end( ), 0.0 ) /
      intralayer_mins_[ i ].size( );
    std::cout  << intralayer_mins_[ i ].size( ) << std::endl;
    for ( unsigned int j = 0; j < intralayer_mins_[ i ].size( ); j++ )
    {
      std::cout  << intralayer_mins_[ i ][ j ] << std::endl;
    }
    */

    ROS_INFO_STREAM( "Intralayer: " << i << " Connections: " << layer_size
                     << " Total: " << layer_total
                     << " Average: " << layer_mean
                     << " Minimum: " << layer_min
                     << " Maximum: " << layer_max
                     << " Variance: " << layer_variance
                     << " Standard Deviation: " << layer_standard_deviation );

    layer_file << "Intralayer: " << i
               << "," << intralayer_weights_[ i ].size( )
               << "," << layer_total
               << "," << layer_mean
               << "," << layer_min
               << "," << layer_max
               << "," << layer_variance
               << "," << layer_standard_deviation
               << "\n";
  }
  layer_file << "\n";
  for ( unsigned int i = 0; i < interlayer_weights_.size( ); i++ )
  {
    layer_total = std::accumulate( interlayer_weights_[ i ].begin( ),
                                   interlayer_weights_[ i ].end( ), 0.0 );
    layer_size = interlayer_weights_[ i ].size( );
    layer_mean = layer_total / layer_size;
    layer_min = *std::min_element( interlayer_weights_[ i ].begin( ),
                                   interlayer_weights_[ i ].end( ) );
    layer_max = *std::max_element( interlayer_weights_[ i ].begin( ),
                                   interlayer_weights_[ i ].end( ) );
    for ( unsigned int j = 0; j < interlayer_weights_[ i ].size( ); j++ )
    {
      layer_variance += ( interlayer_weights_[ i ][ j ] - layer_mean ) *
                        ( interlayer_weights_[ i ][ j ] - layer_mean );
    }
    layer_variance /= layer_size;
    layer_standard_deviation = sqrt( layer_variance );
    ROS_INFO_STREAM( "Interlayer: " << i << " Total: " << layer_total
                     << " Connections: " << layer_size
                     << " Average: "<< layer_mean
                     << " Minimum: " << layer_min
                     << " Maximum: " << layer_max
                     << " Variance: "<< layer_variance
                     << " Standard Deviation: "<< layer_standard_deviation  );

    layer_file << "Interlayer: " << i
               << "," << interlayer_weights_[ i ].size( )
               << "," << layer_total
               << "," << layer_mean
               << "," << layer_min
               << "," << layer_max
               << "," << layer_variance
               << "," << layer_standard_deviation
               << "\n";
    /*for ( unsigned int j = 0; j < interlayer_weights_[ i ].size( ); j++ )
    {
      interlayer_file << interlayer_weights_[ i ][ j ] << "\n";
    }*/
  }
  layer_file.close( );
}

} // end namespace vf_pc_graph
