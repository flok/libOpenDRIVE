#include "OpenDriveMap.h"
#include "Road.h"
#include "Geometries/RoadGeometry.h"
#include "Geometries/Arc.h"
#include "Geometries/Line.h"
#include "Geometries/Spiral.h"
#include "Utils.hpp"
#include "XmlNode.h"
#include "pybind11/include/pybind11/pybind11.h"
#include <pybind11/include/pybind11/stl.h>
#include <pybind11/include/pybind11/stl_bind.h>
namespace py = pybind11;

using namespace odr;

PYBIND11_MODULE(PyOpenDrive, m)
{
    // m.doc() = "Pyton Wrapper for libopendrive library"
    py::class_<OpenDriveMapConfig>(m, "OpenDriveMapConfig")
        .def_readwrite("with_lateralProfile", &OpenDriveMapConfig::with_lateralProfile)
        .def_readwrite("with_laneHeight", &OpenDriveMapConfig::with_laneHeight)
        .def_readwrite("with_road_objects", &OpenDriveMapConfig::with_road_objects)
        .def_readwrite("center_map", &OpenDriveMapConfig::center_map)
        .def_readwrite("abs_z_for_for_local_road_obj_outline", &OpenDriveMapConfig::abs_z_for_for_local_road_obj_outline);
    py::class_<RoadLink>(m, "RoadLink")
        .def(py::init<std::string, RoadLink::Type, RoadLink::ContactPoint>())
        .def_readwrite("id", &RoadLink::id)
        .def_readwrite("type", &RoadLink::type)
        .def_readwrite("contact_point", &RoadLink::contact_point);
    py::class_<SpeedRecord>(m, "SpeedRecord")
        .def(py::init<std::string, std::string>())
        .def_readwrite("max", &SpeedRecord::max)
        .def_readwrite("unit", &SpeedRecord::unit);
    py::enum_<RoadLink::Type>(m, "RoadLinkType")
        .value("Type_None", RoadLink::Type::Type_None)
        .value("Type_Road", RoadLink::Type::Type_Road)
        .value("Type_Junction", RoadLink::Type::Type_Junction)
        .export_values();
    py::enum_<Crossfall::Side>(m, "Side")
        .value("Side_Both", Crossfall::Side::Side_Both)
        .value("Side_Left", Crossfall::Side::Side_Left)
        .value("Side_Right", Crossfall::Side::Side_Right)
        .export_values();

   // py::class_<Vec>(m, "Vec");
    py::class_<Vec1D>(m, "Vec1D");
    py::class_<Vec2D>(m, "Vec2D");
    py::class_<Vec3D>(m, "Vec3D");
    py::class_<Line3D>(m, "Line3D");

    py::class_<Mesh3D>(m, "Mesh3D")
        .def(py::init<>())
        .def("add_mesh", &Mesh3D::add_mesh)
        .def("get_obj", &Mesh3D::get_obj)
        .def_readwrite("vertices", &Mesh3D::vertices)
        .def_readwrite("indices", &Mesh3D::indices)
        .def_readwrite("normals", &Mesh3D::normals)
        .def_readwrite("st_coordinates", &Mesh3D::st_coordinates);

    py::class_<RoadObjectRepeat>(m, "RoadObjectRepeat")
        .def(py::init<double, double, double, double, double, double, double, double, double, double, double>())
        .def_readwrite("s0", &RoadObjectRepeat::s0)
        .def_readwrite("length", &RoadObjectRepeat::length)
        .def_readwrite("distance", &RoadObjectRepeat::distance)
        .def_readwrite("t_start", &RoadObjectRepeat::t_start)
        .def_readwrite("t_end", &RoadObjectRepeat::t_end)
        .def_readwrite("width_start", &RoadObjectRepeat::width_start)
        .def_readwrite("width_end", &RoadObjectRepeat::width_end)
        .def_readwrite("height_start", &RoadObjectRepeat::height_start)
        .def_readwrite("height_end", &RoadObjectRepeat::height_end)
        .def_readwrite("z_offset_start", &RoadObjectRepeat::z_offset_start)
        .def_readwrite("z_offset_end", &RoadObjectRepeat::z_offset_end);

    py::enum_<RoadObjectCorner::Type>(m, "RoadCornerType")
        .value("Type_Local_RelZ", RoadObjectCorner::Type::Type_Local_RelZ)
        .value("Type_Local_AbsZ", RoadObjectCorner::Type::Type_Local_AbsZ)
        .value("Type_Road", RoadObjectCorner::Type::Type_Road)
        .export_values();

    py::class_<RoadObjectCorner>(m, "RoadObjectCorner")
        .def(py::init<Vec3D, double, RoadObjectCorner::Type>())
        .def_readwrite("pt", &RoadObjectCorner::pt)
        .def_readwrite("height", &RoadObjectCorner::height)
        .def_readwrite("type", &RoadObjectCorner::type);

    py::class_<RoadObject>(m, "RoadObject")
        .def(py::init<std::string, std::string, double, double, double, double, double, double, double, double, double, double, double, std::string, std::string, std::string>())
        .def("get_cylinder", &RoadObject::get_cylinder)
        .def("get_box", &RoadObject::get_box)
        .def_readwrite("road_id", &RoadObject::road_id)
        .def_readwrite("id", &RoadObject::id)
        .def_readwrite("type", &RoadObject::type)
        .def_readwrite("name", &RoadObject::name)
        .def_readwrite("orientation", &RoadObject::orientation)
        .def_readwrite("s0", &RoadObject::s0)
        .def_readwrite("t0", &RoadObject::t0)
        .def_readwrite("z0", &RoadObject::z0)
        .def_readwrite("length", &RoadObject::length)
        .def_readwrite("valid_length", &RoadObject::valid_length)
        .def_readwrite("width", &RoadObject::width)
        .def_readwrite("radius", &RoadObject::radius)
        .def_readwrite("height", &RoadObject::height)
        .def_readwrite("hdg", &RoadObject::hdg)
        .def_readwrite("pitch", &RoadObject::pitch)
        .def_readwrite("roll", &RoadObject::roll)
        .def_readwrite("repeats", &RoadObject::repeats)
        .def_readwrite("outline", &RoadObject::outline);

    py::class_<Crossfall>(m, "Crossfall")
        .def(py::init<>()).def("get_crossfall", &Crossfall::get_crossfall)
        .def_readwrite("sides", &Crossfall::sides);
    
    py::class_<RefLine>(m, "RefLine")
        .def(py::init<std::string, double>())
        .def(py::init<const RefLine&>())
        .def_readwrite("road_id", &RefLine::road_id);
        //.def_readwrite("s0_to_geometry", &RefLine::s0_to_geometry);
        //.def("get_geometries", py::overload_cast<>(&RefLine::get_geometries, py::const_))
        //.def("get_geometries", py::overload_cast<>(&RefLine::get_geometries));

    py::enum_<RoadLink::ContactPoint>(m, "ContactPoint")
        .value("ContactPoint_None", RoadLink::ContactPoint::ContactPoint_None)
        .value("ContactPoint_Start", RoadLink::ContactPoint::ContactPoint_Start)
        .value("ContactPoint_End", RoadLink::ContactPoint::ContactPoint_End)
        .export_values();

    py::class_<RoadNeighbor>(m, "RoadNeighbor")
        .def(py::init<std::string, std::string, std::string>())
        .def_readwrite("id", &RoadNeighbor::id)
        .def_readwrite("side", &RoadNeighbor::side)
        .def_readwrite("direction", &RoadNeighbor::direction);
    py::class_<LaneSection>(m, "LaneSection")
        .def(py::init<std::string, double>())
        .def("get_lanes", &LaneSection::get_lanes)
        .def("get_lane_id", &LaneSection::get_lane_id)
        .def("get_lane", &LaneSection::get_lane);
    py::class_<Road>(m, "Road")
        .def(py::init<std::string, double, std::string, std::string>())
        .def("get_lanesections", &Road::get_lanesections)
        .def("get_lanesection", &Road::get_lanesection)
        .def("get_road_objects", &Road::get_road_objects)
        .def("get_lanesection_s0", &Road::get_lanesection_s0)
        .def("get_lanesection_end", py::overload_cast<const LaneSection&>(&Road::get_lanesection_end, py::const_))
        .def("get_lanesection_end", py::overload_cast<const double&>(&Road::get_lanesection_end, py::const_))
        .def("get_lanesection_length", py::overload_cast<const LaneSection&>(&Road::get_lanesection_length, py::const_))
        .def("get_lanesection_length", py::overload_cast<const double&>(&Road::get_lanesection_length, py::const_))
        .def("get_xyz", &Road::get_xyz)
        .def("get_surface_pt", &Road::get_surface_pt)
        .def("get_lane_border_line", py::overload_cast<const Lane&, double, double, double, bool>(&Road::get_lane_border_line, py::const_))
        .def("get_lane_border_line", py::overload_cast<const Lane&, double, bool>(&Road::get_lane_border_line, py::const_))
        .def("get_lane_mesh", py::overload_cast<const Lane&, double, double, double, std::vector<uint32_t>*>(&Road::get_lane_mesh, py::const_))
        .def("get_lane_mesh", py::overload_cast<const Lane&, double, std::vector<uint32_t>*>(&Road::get_lane_mesh, py::const_))
        .def("get_roadmark_mesh", &Road::get_roadmark_mesh)
        .def("get_road_object_mesh", &Road::get_road_object_mesh)

        .def("approximate_lane_border_linear", py::overload_cast<const Lane&, double, double, double, bool>(&Road::approximate_lane_border_linear, py::const_))
        .def("approximate_lane_border_linear", py::overload_cast<const Lane&, double, bool>(&Road::approximate_lane_border_linear, py::const_))
       
        .def_readwrite("length", &Road::length)
        .def_readwrite("id", &Road::id)
        .def_readwrite("junction", &Road::junction)
        .def_readwrite("name", &Road::name)
        .def_readwrite("predecessor", &Road::predecessor)
        .def_readwrite("successor", &Road::successor)
        .def_readwrite("neighbors", &Road::neighbors)
        .def_readwrite("lane_offset", &Road::lane_offset)
        .def_readwrite("superelevation", &Road::superelevation)
        .def_readwrite("crossfall", &Road::crossfall)
        //.def_readwrite("ref_line", &Road::ref_line) // Fix RoadGeometry Delete
        .def_readwrite("s_to_lanesection", &Road::s_to_lanesection)
        .def_readwrite("s_to_type", &Road::s_to_type)
        .def_readwrite("s_to_speed", &Road::s_to_speed)
        .def_readwrite("id_to_object", &Road::id_to_object); // Not working cause of RoadGeometry

    py::class_<OpenDriveMap>(m, "OpenDriveMap")
        .def(py::init<const std::string&, const OpenDriveMapConfig&>(), py::arg("xodr_file") = "", py::arg("config") = OpenDriveMapConfig{})
        .def("get_roads", &OpenDriveMap::get_roads, "get const roads")
        .def("get_junctions", &OpenDriveMap::get_junctions, "get const junctions")
        .def("get_routing_graph", &OpenDriveMap::get_routing_graph, "get routing graph")
        .def_readonly("proj4", &OpenDriveMap::proj4)
        .def_readonly("x_offs", &OpenDriveMap::x_offs)
        .def_readonly("y_offs", &OpenDriveMap::y_offs)
        .def_readonly("id_to_road", &OpenDriveMap::id_to_road)
        .def_readonly("id_to_junction", &OpenDriveMap::id_to_junction)
        .def_readonly("xodr_file", &OpenDriveMap::xodr_file);

    py::class_<HeightOffset>(m, "HeightOffset")
        .def(py::init<double, double>())
        .def_readwrite("inner", &HeightOffset::inner)
        .def_readwrite("outer", &HeightOffset::outer);

    py::class_<LaneKey>(m, "LaneKey")
        .def(py::init<std::string, double, int>())
        .def("to_string", &LaneKey::to_string)
        .def_readwrite("road_id", &LaneKey::road_id)
        .def_readwrite("lanesection_s0", &LaneKey::lanesection_s0)
        .def_readwrite("lane_id", &LaneKey::lane_id);

    py::class_<RoadMarksLine>(m, "RoadMarksLine")
        .def(py::init<std::string, double, int, double, double, double, double, double, double, std::string, std::string>())
        .def_readwrite("road_id", &RoadMarksLine::road_id)
        .def_readwrite("lanesection_s0", &RoadMarksLine::lanesection_s0)
        .def_readwrite("lane_id", &RoadMarksLine::lane_id)
        .def_readwrite("group_s0", &RoadMarksLine::group_s0)
        .def_readwrite("width", &RoadMarksLine::width)
        .def_readwrite("length", &RoadMarksLine::length)
        .def_readwrite("space", &RoadMarksLine::space)
        .def_readwrite("t_offset", &RoadMarksLine::t_offset)
        .def_readwrite("s_offset", &RoadMarksLine::s_offset)
        .def_readwrite("name", &RoadMarksLine::name)
        .def_readwrite("rule", &RoadMarksLine::rule);

    py::class_<RoadMarkGroup>(m, "RoadMarkGroup")
        .def(py::init<std::string, double, int, double, double, double, std::string, std::string, std::string, std::string, std::string>())
        .def_readwrite("road_id", &RoadMarkGroup::road_id)
        .def_readwrite("lanesection_s0", &RoadMarkGroup::lanesection_s0)
        .def_readwrite("lane_id", &RoadMarkGroup::lane_id)
        .def_readwrite("width", &RoadMarkGroup::width)
        .def_readwrite("height", &RoadMarkGroup::height)
        .def_readwrite("s_offset", &RoadMarkGroup::s_offset)
        .def_readwrite("type", &RoadMarkGroup::type)
        .def_readwrite("weight", &RoadMarkGroup::weight)
        .def_readwrite("color", &RoadMarkGroup::color)
        .def_readwrite("material", &RoadMarkGroup::material)
        .def_readwrite("lane_change", &RoadMarkGroup::lane_change)
        .def_readwrite("roadmark_lines", &RoadMarkGroup::roadmark_lines);

    py::class_<RoadMark>(m, "RoadMark")
        .def(py::init<std::string, double, int, double, double, double, double, double, std::string>())
        .def_readwrite("road_id", &RoadMark::road_id)
        .def_readwrite("lanesection_s0", &RoadMark::lanesection_s0)
        .def_readwrite("lane_id", &RoadMark::lane_id)
        .def_readwrite("group_s0", &RoadMark::group_s0)
        .def_readwrite("s_start", &RoadMark::s_start)
        .def_readwrite("s_end", &RoadMark::s_end)
        .def_readwrite("t_offset", &RoadMark::t_offset)
        .def_readwrite("width", &RoadMark::width)
        .def_readwrite("type", &RoadMark::type);

    py::class_<Lane>(m, "Lane")
        .def(py::init<std::string, double, int, bool, std::string>())
        .def("get_roadmarks", &Lane::get_roadmarks)
        .def_readwrite("key", &Lane::key)
        .def_readwrite("id", &Lane::id)
        .def_readwrite("level", &Lane::level)
        .def_readwrite("predecessor", &Lane::predecessor)
        .def_readwrite("successor", &Lane::successor)
        .def_readwrite("type", &Lane::type)
        .def_readwrite("lane_width", &Lane::lane_width)
        .def_readwrite("outer_border", &Lane::outer_border)
        .def_readwrite("inner_border", &Lane::inner_border)
        .def_readwrite("s_to_height_offset", &Lane::s_to_height_offset)
        .def_readwrite("roadmark_groups", &Lane::roadmark_groups);

    py::class_<JunctionLaneLink>(m, "JunctionLaneLink")
        .def(py::init<int, int>())
        .def_readwrite("from", &JunctionLaneLink::from)
        .def_readwrite("to", &JunctionLaneLink::to);

    py::class_<JunctionConnection>(m, "JunctionConnection")
        .def(py::init<std::string, std::string, std::string, JunctionConnection::ContactPoint>())
        .def_readwrite("id", &JunctionConnection::id)
        .def_readwrite("incoming_road", &JunctionConnection::incoming_road)
        .def_readwrite("connecting_road", &JunctionConnection::connecting_road)
        .def_readwrite("contact_point", &JunctionConnection::contact_point)
        .def_readwrite("lane_links", &JunctionConnection::lane_links);

    py::class_<JunctionPriority>(m, "JunctionPriority")
        .def(py::init<std::string, std::string>())
        .def_readwrite("high", &JunctionPriority::high)
        .def_readwrite("low", &JunctionPriority::low);

    py::class_<JunctionController>(m, "JunctionController")
        .def(py::init<std::string, std::string, std::uint32_t>())
        .def_readwrite("id", &JunctionController::id)
        .def_readwrite("type", &JunctionController::type)
        .def_readwrite("sequence", &JunctionController::sequence);

    py::class_<Junction>(m, "Junction")
        .def(py::init<std::string, std::string>())
        .def_readwrite("name", &Junction::name)
        .def_readwrite("id", &Junction::id)
        .def_readwrite("id_to_connection", &Junction::id_to_connection)
        .def_readwrite("id_to_controller", &Junction::id_to_controller)
        .def_readwrite("priorities", &Junction::priorities);

    py::enum_<GeometryType>(m, "GeometryType")
        .value("GeometryType_Line", GeometryType::GeometryType_Line)
        .value("GeometryType_Spiral", GeometryType::GeometryType_Spiral)
        .value("GeometryType_Arc", GeometryType::GeometryType_Arc)
        .value("GeometryType_ParamPoly3", GeometryType::GeometryType_ParamPoly3)
        .export_values();
/*
    class PyRoadGeometry : public RoadGeometry {
    public:
        using RoadGeometry::RoadGeometry;

        std::unique_ptr<RoadGeometry> clone() override {
            PYBIND11_OVERRIDE_PURE(
                std::unique_ptr<RoadGeometry>, 
                RoadGeometry,
                clone
            );
        }
        
    };*/

    // Abstract class, we dont need a py::init function definition
    py::class_<RoadGeometry>(m, "RoadGeometry")
       /* .def("clone", &RoadGeometry::clone)
        .def("get_xy", &RoadGeometry::get_xy)
        .def("get_grad", &RoadGeometry::get_grad)
        .def("approximate_linear", &RoadGeometry::approximate_linear)*/
        .def_readwrite("s0", &RoadGeometry::s0)
        .def_readwrite("x0", &RoadGeometry::x0)
        .def_readwrite("y0", &RoadGeometry::y0)
        .def_readwrite("hdg0", &RoadGeometry::hdg0)
        .def_readwrite("length", &RoadGeometry::length)
        .def_readwrite("type", &RoadGeometry::type);

    // Geometries
    py::class_<Arc, RoadGeometry>(m, "Arc")
        .def(py::init<double, double, double, double, double, double>())
        .def("clone", &Arc::clone)
        .def("get_xy", &Arc::get_xy)
        .def("get_grad", &Arc::get_grad)
        .def("approximate_linear", &Arc::approximate_linear)
        .def_readwrite("curvature", &Arc::curvature);

    py::class_<Line, RoadGeometry>(m, "Line")
        .def(py::init<double, double, double, double, double>())
        .def("clone", &Line::clone)
        .def("get_xy", &Line::get_xy)
        .def("get_grad", &Line::get_grad)
        .def("approximate_linear", &Line::approximate_linear);

    py::class_<Spiral, RoadGeometry>(m, "Spiral")
        .def(py::init<double, double, double, double, double, double, double>())
        .def("clone", &Spiral::clone)
        .def("get_xy", &Spiral::get_xy)
        .def("get_grad", &Spiral::get_grad)
        .def("approximate_linear", &Spiral::approximate_linear)
        .def_readwrite("curv_start", &Spiral::curv_start)
        .def_readwrite("curv_end", &Spiral::curv_end)
        .def_readwrite("s_start", &Spiral::s_start)
        .def_readwrite("s_end", &Spiral::s_end)
        .def_readwrite("c_dot", &Spiral::c_dot);

    py::class_<Poly3>(m, "Poly3")
        .def(py::init<>())
        .def(py::init<double, double, double, double, double>())
        .def("get", &Poly3::get)
        .def("get_grad", &Poly3::get)
        .def("get_max", &Poly3::get_max)
        .def("negate", &Poly3::negate)
        .def("approximate_linear", &Poly3::approximate_linear)
        .def_readwrite("a", &Poly3::a)
        .def_readwrite("b", &Poly3::b)
        .def_readwrite("c", &Poly3::c)
        .def_readwrite("d", &Poly3::d);

    py::class_<CubicSpline>(m, "CubicSpline")
        .def(py::init<>())
        .def("size", &CubicSpline::size)
        .def("get", &CubicSpline::get)
        .def("get_grad", &CubicSpline::get)
        .def("get_max", &CubicSpline::get_max)
        .def("negate", &CubicSpline::negate)
        .def("add", &CubicSpline::add)
        .def("get_poly", &CubicSpline::get_poly)
        .def("approximate_linear", &CubicSpline::approximate_linear)
        .def_readwrite("s0_to_poly", &CubicSpline::s0_to_poly);
}