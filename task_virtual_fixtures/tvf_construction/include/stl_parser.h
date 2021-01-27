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

#ifndef STL_PARSER_H
#define STL_PARSER_H

#include <math.h>
#include <string>
#include <vector>

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <algorithm>

namespace stl_parser
{
  struct point
  {
    float x;
    float y;
    float z;

    point() : x(0), y(0), z(0) {}
    point( float xp, float yp, float zp ) : x( xp ), y( yp ), z( zp ) {}

    bool operator == ( const point& other ) const
    {
      double tol = 0.0001;
      if ( fabs( other.x-this->x ) > tol )
        return false;
      if ( fabs( other.y-this->y ) > tol )
        return false;
      if ( fabs( other.z-this->z ) > tol )
        return false;
      return true;
    }
  };

  struct triangle
  {
    point normal;
    point v1;
    point v2;
    point v3;
    triangle( point normalp, point v1p, point v2p, point v3p ) :
      normal( normalp ), v1( v1p ), v2( v2p ), v3( v3p ) {}

    bool operator == ( const triangle& other ) const
    {
      if( !( other.normal == normal ) )
        return false;
      if( !( other.v1 == this->v1 ) )
        return false;
      if( !( other.v1 == this->v2 ) )
        return false;
      if( !( other.v1 == this->v3 ) )
        return false;
      if( !( other.v2 == this->v1 ) )
        return false;
      if( !( other.v2 == this->v2 ) )
        return false;
      if( !( other.v2 == this->v3 ) )
        return false;
      if( !( other.v3 == this->v1 ) )
        return false;
      if( !( other.v3 == this->v2 ) )
        return false;
      if( !( other.v3 == this->v3 ) )
        return false;
      return true;
    }

  };

  std::ostream& operator<<( std::ostream& out, const triangle& t );

  struct stl_data
  {
    std::string name;
    std::vector<triangle> triangles;

    stl_data( std::string namep ) : name( namep ) {}
  };

  stl_data parse_stl( const std::string& stl_path );

}

#endif
