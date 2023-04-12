
#include "Geometries/Arc.h"
#include "Geometries/CubicSpline.h"
#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/RoadGeometry.h"
#include "Geometries/Spiral.h"
#include "Junction.h"
#include "Lane.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Mesh.h"
#include "RefLine.h"
#include "Road.h"
#include "RoadMark.h"
#include "RoadObject.h"
#include <OpenDriveMap.h>

#include <nanobind/make_iterator.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace odr;

NB_MODULE(pyopendrive, m)
{
    init_roadgeometry(m);
    init_lane(m);
    init_mesh3d(m);
    init_refline(m);
    init_lanesection(m);
    init_roadmark(m);
    init_roadobject(m);
    init_road(m);
    init_junction(m);
    init_arc(m);
    init_cubicspline(m);
    init_poly3(m);
    init_line(m);
    init_spiral(m);
    init_routinggraph(m);
    init_roadnetworkmesh(m);
    init_opendrivemap(m);
}