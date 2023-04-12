#include "Geometries/RoadGeometry.h"

namespace odr
{

RoadGeometry::RoadGeometry(double s0, double x0, double y0, double hdg0, double length, GeometryType type) :
    s0(s0), x0(x0), y0(y0), hdg0(hdg0), length(length), type(type)
{
}

#ifdef PYTHON_BINDING
void init_roadgeometry(nb::module_& m)
{
  nb::class_<RoadGeometry>(m, "RoadGeometry")
    /*.def("clone", &RoadGeometry::clone)
    .def("get_xy", &RoadGeometry::get_xy)
    .def("get_grad", &RoadGeometry::get_grad)
    .def("approximate_linear", &RoadGeometry::approximate_linear)*/
    .def_rw("s0", &RoadGeometry::s0)
    .def_rw("x0", &RoadGeometry::x0)
    .def_rw("y0", &RoadGeometry::y0)
    .def_rw("hdg0", &RoadGeometry::hdg0)
    .def_rw("length", &RoadGeometry::length)
    .def_rw("type", &RoadGeometry::type);
}
#endif
} // namespace odr
