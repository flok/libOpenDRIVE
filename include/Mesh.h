#pragma once
#include "Math.hpp"

#include <cstdint>
#include <string>
#include <vector>

#ifdef PYTHON_BINDING
    #include <nanobind/nanobind.h>
    #include <nanobind/stl/map.h>
    #include <nanobind/stl/string.h>
namespace nb = nanobind;
#endif

namespace odr
{

struct Mesh3D
{
    Mesh3D() = default;

    void        add_mesh(const Mesh3D& other);
    std::string get_obj() const;

    std::vector<Vec3D>    vertices;
    std::vector<uint32_t> indices;
    std::vector<Vec3D>    normals;
    std::vector<Vec2D>    st_coordinates;
};

#ifdef PYTHON_BINDING
void init_mesh3d(nb::module_& m);
#endif
} // namespace odr