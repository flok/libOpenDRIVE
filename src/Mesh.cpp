#include "Mesh.h"

#include <array>
#include <cstddef>
#include <sstream>
#include <string>

namespace odr
{
void Mesh3D::add_mesh(const Mesh3D& other)
{
    const std::size_t idx_offset = this->vertices.size();

    this->vertices.insert(this->vertices.end(), other.vertices.begin(), other.vertices.end());
    this->normals.insert(this->normals.end(), other.normals.begin(), other.normals.end());
    this->st_coordinates.insert(this->st_coordinates.end(), other.st_coordinates.begin(), other.st_coordinates.end());

    for (const uint32_t& idx : other.indices)
        this->indices.push_back(idx + idx_offset);
}

std::string Mesh3D::get_obj() const
{
    std::stringstream ss_obj;
    for (const Vec3D& vt : this->vertices)
    {
        ss_obj << "v " << vt[0] << ' ' << vt[1] << ' ' << vt[2] << std::endl;
    }
    for (const Vec3D& vn : this->normals)
    {
        ss_obj << "vn " << vn[0] << ' ' << vn[1] << ' ' << vn[2] << std::endl;
    }

    for (std::size_t idx = 0; idx < this->indices.size(); idx += 3)
    {
        const std::size_t i1 = indices.at(idx) + 1;
        const std::size_t i2 = indices.at(idx + 1) + 1;
        const std::size_t i3 = indices.at(idx + 2) + 1;
        if (this->normals.size() == this->vertices.size())
            ss_obj << "f " << i1 << "//" << i1 << ' ' << i2 << "//" << i2 << ' ' << i3 << "//" << i3 << std::endl;
        else
            ss_obj << "f " << i1 << ' ' << i2 << ' ' << i3 << std::endl;
    }

    return ss_obj.str();
}

#ifdef PYTHON_BINDING
void init_mesh3d(nb::module_& m)
{
    nb::class_<Vec1D>(m, "Vec1D");
    nb::class_<Vec2D>(m, "Vec2D");
    nb::class_<Vec3D>(m, "Vec3D");
    nb::class_<Line3D>(m, "Line3D");

    nb::class_<Mesh3D>(m, "Mesh3D")
        .def(nb::init<>())
        .def("add_mesh", &Mesh3D::add_mesh)
        .def("get_obj", &Mesh3D::get_obj)
        .def_rw("vertices", &Mesh3D::vertices)
        .def_rw("indices", &Mesh3D::indices)
        .def_rw("normals", &Mesh3D::normals)
        .def_rw("st_coordinates", &Mesh3D::st_coordinates);
}
#endif

} // namespace odr