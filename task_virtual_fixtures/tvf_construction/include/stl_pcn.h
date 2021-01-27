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

#ifndef STL_PCN_H
#define STL_PCN_H

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <stl_parser.h>

namespace stl_pcn
{
class STLPCN
{
public:
STLPCN( pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud );
~STLPCN( );

// Functions
int Run( std::string input_path, std::string suffix, double intralayer_dist,
         double epsilon_1, double epsilon_2);
void SetFilePaths( std::string input_path, std::string suffix );
void PrintFilePaths( );
unsigned int LoadPCD( );
unsigned int LoadPCDNormals( );
unsigned int LoadSTL( );
unsigned int FilterWorkingCloud( );
unsigned int FilterCloudNormals( );
void SetWorkingCloud( );
void SetFilterCloud( );
std::vector< double > AverageSTLNormals( );
std::vector< double > AveragePCNNormals( );
unsigned int InterpolateSTL( const double intralayer_dist );
void STLToPC( );
double STLNormals( );
void EstimateNormals( const unsigned int knn );
void MovingLeastSquaresEstimation( );
void CorrectNormals( );
void ViewCloudNormals( double size );
pcl::PointCloud<pcl::PointNormal>::Ptr GetPointCloud( );
unsigned int SaveCloudNormals( );

private:
std::string file_path_, pcd_path_, stl_path_, cloud_path_, mesh_path_;
double intralayer_dist_;
// STL data
stl_parser::stl_data stl_info_;
// PCL variables
pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_, working_cloud_;
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_,
  filtered_with_normals_;
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
// Functions
bool InterpolateTriangle( const unsigned int i );
void TriangleVertexesDistances( const unsigned int i,
                                const unsigned int j,
                                std::vector< double > &dist );
void InvertCloudNormal( const unsigned int i );

};

}

#endif