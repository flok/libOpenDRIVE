#include "RoadObject.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace odr
{

RoadObjectRepeat::RoadObjectRepeat(double s0,
                                   double length,
                                   double distance,
                                   double t_start,
                                   double t_end,
                                   double width_start,
                                   double width_end,
                                   double height_start,
                                   double height_end,
                                   double z_offset_start,
                                   double z_offset_end) :
    s0(s0),
    length(length), distance(distance), t_start(t_start), t_end(t_end), width_start(width_start), width_end(width_end), height_start(height_start),
    height_end(height_end), z_offset_start(z_offset_start), z_offset_end(z_offset_end)
{
}

RoadObjectCorner::RoadObjectCorner(int id, Vec3D pt, double height, Type type) : id(id), pt(pt), height(height), type(type) {}

RoadObjectOutline::RoadObjectOutline(int id, std::string fill_type, std::string lane_type, bool outer, bool closed) :
    id(id), fill_type(fill_type), lane_type(lane_type), outer(outer), closed(closed)
{
}

RoadObject::RoadObject(std::string road_id,
                       std::string id,
                       double      s0,
                       double      t0,
                       double      z0,
                       double      length,
                       double      valid_length,
                       double      width,
                       double      radius,
                       double      height,
                       double      hdg,
                       double      pitch,
                       double      roll,
                       std::string type,
                       std::string name,
                       std::string orientation,
                       std::string subtype,
                       bool        is_dynamic) :
    road_id(road_id),
    id(id), type(type), name(name), orientation(orientation), subtype(subtype), s0(s0), t0(t0), z0(z0), length(length), valid_length(valid_length),
    width(width), radius(radius), height(height), hdg(hdg), pitch(pitch), roll(roll), is_dynamic(is_dynamic)
{
}

Mesh3D RoadObject::get_cylinder(const double eps, const double radius, const double height)
{
    Mesh3D cylinder_mesh;
    cylinder_mesh.vertices.push_back({0, 0, 0});
    cylinder_mesh.vertices.push_back({0, 0, height});

    const double eps_adj = 0.5 * eps; // reduce eps a bit, cylinders more subsceptible to low resolution
    const double eps_angle =
        (radius <= eps_adj) ? M_PI / 6 : std::acos((radius * radius - 4 * radius * eps_adj + 2 * eps_adj * eps_adj) / (radius * radius));

    std::vector<double> angles;
    for (double alpha = 0; alpha < 2 * M_PI; alpha += eps_angle)
        angles.push_back(alpha);
    angles.push_back(2 * M_PI);

    for (const double& alpha : angles)
    {
        const Vec3D circle_pt_bottom = {radius * std::cos(alpha), radius * std::sin(alpha), 0};
        const Vec3D circle_pt_top = {radius * std::cos(alpha), radius * std::sin(alpha), height};
        cylinder_mesh.vertices.push_back(circle_pt_bottom);
        cylinder_mesh.vertices.push_back(circle_pt_top);

        if (cylinder_mesh.vertices.size() > 5)
        {
            const std::size_t     cur_idx = cylinder_mesh.vertices.size() - 1;
            std::array<size_t, 6> top_bottom_idx_patch = {0, cur_idx - 1, cur_idx - 3, 1, cur_idx - 2, cur_idx};
            cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), top_bottom_idx_patch.begin(), top_bottom_idx_patch.end());
            std::array<size_t, 6> wall_idx_patch = {cur_idx, cur_idx - 2, cur_idx - 3, cur_idx, cur_idx - 3, cur_idx - 1};
            cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
        }
    }

    return cylinder_mesh;
}

Mesh3D RoadObject::get_box(const double w, const double l, const double h)
{
    Mesh3D box_mesh;
    box_mesh.vertices = {Vec3D{l / 2, w / 2, 0},
                         Vec3D{-l / 2, w / 2, 0},
                         Vec3D{-l / 2, -w / 2, 0},
                         Vec3D{l / 2, -w / 2, 0},
                         Vec3D{l / 2, w / 2, h},
                         Vec3D{-l / 2, w / 2, h},
                         Vec3D{-l / 2, -w / 2, h},
                         Vec3D{l / 2, -w / 2, h}};
    box_mesh.indices = {0, 3, 1, 3, 2, 1, 4, 5, 7, 7, 5, 6, 7, 6, 3, 3, 6, 2, 5, 4, 1, 1, 4, 0, 0, 4, 7, 7, 3, 0, 1, 6, 5, 1, 2, 6};

    return box_mesh;
}

#ifdef PYTHON_BINDING
void init_roadobject(nb::module_& m)
{
    nb::class_<RoadObjectRepeat>(m, "RoadObjectRepeat")
        .def(nb::init<double, double, double, double, double, double, double, double, double, double, double>())
        .def_rw("s0", &RoadObjectRepeat::s0)
        .def_rw("length", &RoadObjectRepeat::length)
        .def_rw("distance", &RoadObjectRepeat::distance)
        .def_rw("t_start", &RoadObjectRepeat::t_start)
        .def_rw("t_end", &RoadObjectRepeat::t_end)
        .def_rw("width_start", &RoadObjectRepeat::width_start)
        .def_rw("width_end", &RoadObjectRepeat::width_end)
        .def_rw("height_start", &RoadObjectRepeat::height_start)
        .def_rw("height_end", &RoadObjectRepeat::height_end)
        .def_rw("z_offset_start", &RoadObjectRepeat::z_offset_start)
        .def_rw("z_offset_end", &RoadObjectRepeat::z_offset_end);

    nb::enum_<RoadObjectCorner::Type>(m, "RoadCornerType")
        .value("Type_Local_RelZ", RoadObjectCorner::Type::Type_Local_RelZ)
        .value("Type_Local_AbsZ", RoadObjectCorner::Type::Type_Local_AbsZ)
        .value("Type_Road", RoadObjectCorner::Type::Type_Road)
        .export_values();

    nb::class_<RoadObjectCorner>(m, "RoadObjectCorner")
        .def(nb::init<int, Vec3D, double, RoadObjectCorner::Type>())
        .def_rw("pt", &RoadObjectCorner::pt)
        .def_rw("height", &RoadObjectCorner::height)
        .def_rw("type", &RoadObjectCorner::type);

    nb::class_<RoadObjectOutline>(m, "RoadOBjectOutline")
        .def(nb::init<int, std::string, std::string, bool, bool>())
        .def_rw("id", &RoadObjectOutline::fill_type)
        .def_rw("lane_type", &RoadObjectOutline::lane_type)
        .def_rw("outer", &RoadObjectOutline::outer)
        .def_rw("closed", &RoadObjectOutline::closed);

    nb::class_<RoadObject>(m, "RoadObject")
        .def(nb::init<std::string,
                      std::string,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      double,
                      std::string,
                      std::string,
                      std::string,
                      std::string,
                      bool>())
        .def("get_cylinder", &RoadObject::get_cylinder)
        .def("get_box", &RoadObject::get_box)
        .def_rw("road_id", &RoadObject::road_id)
        .def_rw("id", &RoadObject::id)
        .def_rw("type", &RoadObject::type)
        .def_rw("name", &RoadObject::name)
        .def_rw("orientation", &RoadObject::orientation)
        .def_rw("s0", &RoadObject::s0)
        .def_rw("t0", &RoadObject::t0)
        .def_rw("z0", &RoadObject::z0)
        .def_rw("length", &RoadObject::length)
        .def_rw("valid_length", &RoadObject::valid_length)
        .def_rw("width", &RoadObject::width)
        .def_rw("radius", &RoadObject::radius)
        .def_rw("height", &RoadObject::height)
        .def_rw("hdg", &RoadObject::hdg)
        .def_rw("pitch", &RoadObject::pitch)
        .def_rw("roll", &RoadObject::roll)
        .def_rw("repeats", &RoadObject::repeats)
        .def_rw("outline", &RoadObject::outlines);
}
#endif

} // namespace odr
