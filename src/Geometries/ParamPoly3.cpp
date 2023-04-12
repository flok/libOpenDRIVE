#include "Geometries/ParamPoly3.h"
#include "Geometries/RoadGeometry.h"
#include "Math.hpp"

#ifdef PYTHON_BINDING
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
namespace nb = nanobind;
#endif

#include <array>
#include <cmath>
#include <map>

namespace odr
{

ParamPoly3::ParamPoly3(double s0,
                       double x0,
                       double y0,
                       double hdg0,
                       double length,
                       double aU,
                       double bU,
                       double cU,
                       double dU,
                       double aV,
                       double bV,
                       double cV,
                       double dV,
                       bool   pRange_normalized) :
    RoadGeometry(s0, x0, y0, hdg0, length, GeometryType_ParamPoly3),
    aU(aU), bU(bU), cU(cU), dU(dU), aV(aV), bV(bV), cV(cV), dV(dV), pRange_normalized(pRange_normalized)
{
    if (!pRange_normalized)
    {
        this->bU = bU * length;
        this->bV = bV * length;
        this->cU = cU * length * length;
        this->cV = cV * length * length;
        this->dU = dU * length * length * length;
        this->dV = dV * length * length * length;
    }

    const std::array<Vec2D, 4> coefficients = {{{this->aU, this->aV}, {this->bU, this->bV}, {this->cU, this->cV}, {this->dU, this->dV}}};
    this->cubic_bezier = CubicBezier2D(CubicBezier2D::get_control_points(coefficients));

    this->cubic_bezier.arclen_t[length] = 1.0;
    this->cubic_bezier.valid_length = length;
}

std::unique_ptr<RoadGeometry> ParamPoly3::clone() const { return std::make_unique<ParamPoly3>(*this); }

Vec2D ParamPoly3::get_xy(double s) const
{
    const double p = this->cubic_bezier.get_t(s - s0);
    const Vec2D  pt = this->cubic_bezier.get(p);

    const double xt = (std::cos(hdg0) * pt[0]) - (std::sin(hdg0) * pt[1]) + x0;
    const double yt = (std::sin(hdg0) * pt[0]) + (std::cos(hdg0) * pt[1]) + y0;

    return Vec2D{xt, yt};
}

Vec2D ParamPoly3::get_grad(double s) const
{
    const double p = this->cubic_bezier.get_t(s - s0);
    const Vec2D  dxy = this->cubic_bezier.get_grad(p);

    const double h1 = std::cos(hdg0);
    const double h2 = std::sin(hdg0);
    const double dx = h1 * dxy[0] - h2 * dxy[1];
    const double dy = h2 * dxy[0] + h1 * dxy[1];

    return {{dx, dy}};
}

std::set<double> ParamPoly3::approximate_linear(double eps) const
{
    std::set<double> p_vals = this->cubic_bezier.approximate_linear(eps);

    std::set<double> s_vals;
    for (const double& p : p_vals)
        s_vals.insert(p * length + s0);

    return s_vals;
}

#ifdef PYTHON_BINDING
void init_parampoly3(nb::module_& m)
{
  nb::class_<ParamPoly3, RoadGeometry>(m, "ParamPoly3")
    .def(nb::init<double, double, double, double, double, 
                  double, double, double, double, double, 
                  double, double, double, bool>())
    .def("get_xy", &ParamPoly3::get_xy)
    .def("get_grad", &ParamPoly3::get_grad)
    .def("approximate_linear", &ParamPoly3::approximate_linear)
    //.def("clone", &ParamPoly3::clone)
    .def_rw("s0", &ParamPoly3::s0)
    .def_rw("x0", &ParamPoly3::x0)
    .def_rw("y0", &ParamPoly3::y0)
    .def_rw("hdg0", &ParamPoly3::hdg0)
    .def_rw("length", &ParamPoly3::length)
    .def_rw("aU", &ParamPoly3::aU)
    .def_rw("bU", &ParamPoly3::bU)
    .def_rw("cU", &ParamPoly3::cU)
    .def_rw("dU", &ParamPoly3::dU)
    .def_rw("aV", &ParamPoly3::aV)
    .def_rw("bV", &ParamPoly3::bV)
    .def_rw("cV", &ParamPoly3::cV)
    .def_rw("dV", &ParamPoly3::dV)
    .def_rw("pRange_normalized", &ParamPoly3::pRange_normalized)
    .def_rw("cubic_bezier", &ParamPoly3::cubic_bezier);
}
#endif

} // namespace odr
