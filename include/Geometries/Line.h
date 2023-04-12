#pragma once
#include "Math.hpp"
#include "RoadGeometry.h"

#ifdef PYTHON_BINDING
    #include <nanobind/nanobind.h>
    #include <nanobind/stl/map.h>
    #include <nanobind/stl/string.h>
namespace nb = nanobind;
#endif

#include <memory>
#include <set>

namespace odr
{

struct Line : public RoadGeometry
{
    Line(double s0, double x0, double y0, double hdg0, double length);

    std::unique_ptr<RoadGeometry> clone() const override;

    Vec2D get_xy(double s) const override;
    Vec2D get_grad(double s) const override;

    std::set<double> approximate_linear(double eps) const override;
};

#ifdef PYTHON_BINDING
void init_line(nb::module_& m);
#endif

} // namespace odr