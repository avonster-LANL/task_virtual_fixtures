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

#ifndef PCN_LAYERS_H
#define PCN_LAYERS_H

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pcn_layers
{
class PCNLayers
{
public:
PCNLayers( pcl::PointCloud< pcl::PointNormal >::Ptr input_cloud );
~PCNLayers( );

// Functions
void SetFilePaths( std::string input_path, std::string suffix );
unsigned int LoadNormalPCD( );
void Run( double intralayer_dist, double dist, unsigned int layer );
unsigned int LoadVFPCD( );
void ExtendNormals( double dist );
unsigned int InterpolateVoxelize( double &intralayer_dist, double &dist );
double CheckResolution( double &intralayer_dist );
void ConvertToMesh( const unsigned int i );
void ViewVFLayer( );
void ViewCloudNormals( double normal_length );
void SaveVFLayer( const unsigned int i );
void SaveVFMod( const unsigned int i );
pcl::PointCloud< pcl::PointNormal >::Ptr GetVFLayer( );

private:
std::string file_path_, cloud_path_, mesh_path_, vf_path_;
// PCL variables
pcl::PointCloud< pcl::PointNormal >::Ptr cloud_with_normals_, vf_layer_,
  layer_mesh_;
pcl::KdTreeFLANN< pcl::PointNormal > kdtree_;
};

}

#endif