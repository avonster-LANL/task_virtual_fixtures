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

#ifndef TVF_GRAPH_H
#define TVF_GRAPH_H

// Random number generation
#include <random>
// Point Cloud Library includes
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
// Boost Graph Library includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_iterator.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
// ROS includes
#include <geometry_msgs/PoseStamped.h>

namespace tvf_graph
{
class TVFGraph
{
public:
TVFGraph( std::vector < pcl::PointCloud< pcl::PointNormal >::Ptr,
          Eigen::aligned_allocator < pcl::PointCloud<
            pcl::PointNormal >::Ptr > > pc_vector );
~TVFGraph( );

// PCL variables
std::vector < pcl::PointCloud<pcl::PointNormal>::Ptr,
  Eigen::aligned_allocator < pcl::PointCloud< pcl::PointNormal >::Ptr > >
  vf_pcs_;
// Boost graph variables
typedef boost::property< boost::graph_name_t, std::string > GraphName;
typedef boost::property< boost::edge_weight_t, double > EdgeWeight;
// Custom boost vertex property structure for pose stamped graph
struct VertexProperties
{
  unsigned int id;
  unsigned int layer;
  double curvature;
  std::string label;
  geometry_msgs::PoseStamped pose_stamped;
};
// Custom boost vertex property structure for pose stamped string graph
struct VertexPropertiesString
{
  unsigned int id;
  unsigned int layer;
  double curvature;
  std::string label;
  std::string pose_stamped;
};

// Boost graph definitions
typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS,
  VertexProperties, EdgeWeight, GraphName > UndirectedGraph;
typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS,
  VertexPropertiesString, EdgeWeight, GraphName > UndirectedGraphString;

// Property maps for pose stamped graphs
typedef boost::ref_property_map< UndirectedGraph *, std::string > GraphNameMap;
typedef boost::property_map< UndirectedGraph,
  boost::vertex_index_t >::type IndexMap;
typedef boost::property_map< UndirectedGraph,
  unsigned int VertexProperties::* >::type IdMap;
typedef boost::property_map< UndirectedGraph,
  unsigned int VertexProperties::* >::type LayerMap;
typedef boost::property_map< UndirectedGraph,
  double VertexProperties::* >::type CurvatureMap;
typedef boost::property_map< UndirectedGraph,
  std::string VertexProperties::* >::type LabelMap;
typedef boost::property_map< UndirectedGraph,
  geometry_msgs::PoseStamped VertexProperties::* >::type PoseMap;
typedef boost::property_map< UndirectedGraph,
  boost::edge_weight_t >::type EdgeWeightMap;
// Iterators for pose stamped graphs
typedef boost::graph_traits< UndirectedGraph >::vertex_iterator VertexIterator;
typedef boost::graph_traits< UndirectedGraph >::adjacency_iterator
  AdjacencyIterator;
typedef boost::graph_traits< UndirectedGraph >::edge_iterator EdgeIterator;

// Property maps for string pose stamped graphs
typedef boost::ref_property_map< UndirectedGraphString *,
  std::string > GraphNameMapString;
typedef boost::property_map< UndirectedGraphString,
  boost::vertex_index_t >::type IndexMapString;
typedef boost::property_map< UndirectedGraphString,
  unsigned int VertexPropertiesString::* >::type IdMapString;
typedef boost::property_map< UndirectedGraphString,
  unsigned int VertexPropertiesString::* >::type LayerMapString;
typedef boost::property_map< UndirectedGraphString,
  double VertexPropertiesString::* >::type CurvatureMapString;
typedef boost::property_map< UndirectedGraphString,
  std::string VertexPropertiesString::* >::type LabelMapString;
typedef boost::property_map< UndirectedGraphString,
  std::string VertexPropertiesString::* >::type PoseMapString;
typedef boost::property_map< UndirectedGraphString,
  boost::edge_weight_t >::type EdgeWeightMapString;
// Iterators for string pose stamped graphs
typedef boost::graph_traits< UndirectedGraphString >::vertex_iterator
  VertexIteratorString;
typedef boost::graph_traits< UndirectedGraphString >::adjacency_iterator
  AdjacencyIteratorString;
typedef boost::graph_traits< UndirectedGraphString >::edge_iterator
  EdgeIteratorString;

// Function for reloading ROS params
void LoadParams( std::string id );
// Run LoadPC, CloudToGraph, and SaveGraph
void Run( unsigned int layers, std::string suffix );
// Load file information into a vector of point cloud pointers
void LoadPC( unsigned int layers );
// Load a pose stamped string graph and convert to a pose stamped graph
void LoadGraph( );
// Convert the vector of point cloud pointers into a pose stamped graph structure
void CloudToGraph( );
// Get all neighbors for provided vertex index
void GetAllNeighbors( const unsigned int index, UndirectedGraph &graph );
// Get k nearest neighbors to provided vertex index
void GetNearestNeighbors( const unsigned int index,
                          const unsigned int nearest, UndirectedGraph &graph );
// Get neighbors with decreasing likelihood beyond the intralayer distance
void GetScaledNeighbors( const unsigned int index, UndirectedGraph &graph );
// Add specified vertex information to provided graph
void AddVertex( const unsigned int index, UndirectedGraph &graph );
// Update orientation of index in provided graph
void UpdateVertex( const unsigned int tvf_index, const unsigned int graph_index,
                   geometry_msgs::Pose pose, UndirectedGraph &graph );
// Add all edges to provided graph, pull from TVF if possible, calculate if not
void AddPathEdges( const unsigned int index, UndirectedGraph &graph );
// Remove specified vertex information from provided graph
bool RemoveVertex( const unsigned int index, UndirectedGraph &graph );
// Save the class stored pose stamped graph to a dot file
void SaveGraph( std::string layers );
// Save the provided pose stamped graph to a dot file
void SaveInputGraph( std::string filename, UndirectedGraph input_tvf_graph );
// Save statistical data on the TVF
void SaveLayerData( );

private:
// Variables
std::string vf_frame_, file_path_, suffix_, graph_file_ = "_tvf_graph";
double min_offset_, max_offset_, interlayer_dist_, intralayer_dist_,
  big_number_ = DBL_MAX;
std::vector< unsigned int > layer_sizes_;
std::vector< std::vector< double > > intralayer_mins_, intralayer_weights_,
  interlayer_weights_;
boost::graph_traits< UndirectedGraph >::vertices_size_type total_vertices_;
// Random number generator
std::default_random_engine generator_;
std::uniform_int_distribution< int > distribution_{ 1, 1000 };
// Graph iterators for pose stamped graph
std::pair< VertexIterator, VertexIterator > vs_;
std::pair< AdjacencyIterator, AdjacencyIterator > neighbors_;
std::pair< EdgeIterator, EdgeIterator > es_;
// Graph iterators for pose stamped string graph
std::pair< VertexIteratorString, VertexIteratorString > vs_string_;
std::pair< AdjacencyIteratorString, AdjacencyIteratorString > neighbors_string_;
std::pair< EdgeIteratorString, EdgeIteratorString > es_string_;
// Index and edge maps
IndexMap index_map_;
EdgeWeightMap edge_weight_map_;
// Create index edge variable
UndirectedGraph::edge_descriptor edge_;
// Make graphs
UndirectedGraph tvf_graph_, loaded_tvf_graph_;
UndirectedGraphString tvf_graph_string_;

// Add a vertex to the graph using the total number of nodes in the graph and
// the node's location in the current layer
unsigned int AddVertices( const unsigned int layer );
// Convert PC point to Pose Stamped message
void PCToPoseStamped( const unsigned int layer, const unsigned int node );
// Calculate the distance between and add an edge between the specified points
void AddEdge( const unsigned int layer, const unsigned int node );
// Link the previous layer of nodes to the closest nodes in the current layer
void LinkLayersOut( const unsigned int layer );
// Link the current layer of nodes to the closest nodes in the previous layer
void LinkLayersIn( const unsigned int layer );
// Convert a loaded string message back into a Pose Stamped message
void StringToPoseStamped( const unsigned int node );
};

}

#endif //VF_PC_GRAPH_H