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
Doc = This code is based on code pulled form Github.com but I can't seem to
find the original again. It parses STL file data.

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

#include <stl_parser.h>

namespace stl_parser
{
  std::ostream& operator << ( std::ostream& out, const point p )
  {
    out << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    return out;
  }

  std::ostream& operator << ( std::ostream& out, const triangle& t )
  {
    out << "TRIANGLE: " << std::endl
        << "Normal: " << t.normal
        << "V1: " << t.v1
        << "V2: " << t.v2
        << "V3: " << t.v3;
    return out;
  }

  float parse_float( std::ifstream& s )
  {
    char f_buf[sizeof(float)];
    s.read(f_buf, 4);
    float* fptr = (float*) f_buf;
    return *fptr;
  }

  point parse_point( std::ifstream& s )
  {
    float x = parse_float( s );
    float y = parse_float( s );
    float z = parse_float( s );
    return point( x, y, z );
  }

  stl_data parse_stl( const std::string& stl_path )
  {
    std::ifstream stl_file( stl_path.c_str( ),
                            std::ios::in | std::ios::binary );
    if ( !stl_file )
    {
      std::cout << "ERROR: COULD NOT READ FILE" << std::endl;
      assert( false );
    }

    char header_info[ 80 ] = "";
    char n_triangles[ 4 ];
    stl_file.read( header_info, 80 );
    stl_file.read( n_triangles, 4 );
    std::string h( header_info );
    stl_data info( h );
    unsigned int* r = ( unsigned int* ) n_triangles;
    unsigned int num_triangles = *r;
    for ( unsigned int i = 0; i < num_triangles; i++ )
    {
      auto normal = parse_point( stl_file );
      auto v1 = parse_point( stl_file );
      auto v2 = parse_point( stl_file );
      auto v3 = parse_point( stl_file );
      auto tri = triangle( normal, v1, v2, v3 );

      info.triangles.push_back( tri );
      char dummy[ 2 ];
      stl_file.read( dummy, 2 );
      /*if( std::find(info.triangles.begin( ),
                      info.triangles.end( ), tri ) == info.triangles.end( ) )
      {
        info.triangles.push_back( tri );
        char dummy[ 2 ];
        stl_file.read( dummy, 2 );
      }
      else
      {
      std::cout << "Rejected Triangle: " << tri << std::endl;
      std::cout << "Rejected Triangle 2: "
                << *( std::find( info.triangles.begin( ),
                                 info.triangles.end( ), tri ) )
                << std::endl;
      }*/
    }
    return info;
  }
}