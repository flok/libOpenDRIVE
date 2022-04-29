#include "OpenDriveMap.h"
#include "Road.h"
#include "Utils.hpp"
#include "XmlNode.h"
#include "pybind11/include/pybind11/pybind11.h"
#include <memory>
#include <pybind11/include/pybind11/stl.h>
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
    py::enum_<RoadLink::Type>(m, "Type")
        .value("Type_None", RoadLink::Type::Type_None)
        .value("Type_Road", RoadLink::Type::Type_Road)
        .value("Type_Junction", RoadLink::Type::Type_Junction)
        .export_values();
    py::enum_<Crossfall::Side>(m, "Side")
        .value("Side_Both", Crossfall::Side::Side_Both)
        .value("Side_Left", Crossfall::Side::Side_Left)
        .value("Side_Right", Crossfall::Side::Side_Right)
        .export_values();
    py::class_<Crossfall>(m, "Crossfall").def(py::init<>()).def("get_crossfall", &Crossfall::get_crossfall).def_readwrite("sides", &Crossfall::sides);
    py::class_<RefLine>(m, "RefLine").def(py::init<std::string, double>()).def(py::init<const RefLine&>());
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
        .def("get_road_objects", &Road::get_road_object_mesh)
        .def("get_lanesection_s0", &Road::get_lanesection_s0)
        .def("get_lanesection_end", py::overload_cast<const LaneSection&>(&Road::get_lanesection_end, py::const_))
        .def("get_lanesection_end", py::overload_cast<const double&>(&Road::get_lanesection_end, py::const_))
        .def("get_lanesection_length", py::overload_cast<const LaneSection&>(&Road::get_lanesection_length, py::const_))
        .def("get_lanesection_length", py::overload_cast<const double&>(&Road::get_lanesection_length, py::const_))
        .def("get_xyz", &Road::get_xyz)
        // Need to implement further types
        /* .def("get_surface_pt", &Road::get_surface_pt)
         .def("get_lane_border_line", &Road::get_lane_border_line)
         .def("get_lane_border_line", &Road::get_lane_border_line)
         .def("get_lane_mesh", &Road::get_lane_mesh)
         .def("get_lane_mesh", &Road::get_lane_mesh)
         .def("get_roadmark_mesh", &Road::get_roadmark_mesh)
         .def("get_road_object_mesh", &Road::get_road_object_mesh)
         .def("approximate_lane_border_linear", &Road::approximate_lane_border_linear)
         .def("approximate_lane_border_linear", &Road::approximate_lane_border_linear)
       */
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
        .def_readwrite("s_to_speed", &Road::s_to_speed);
    //.def_readwrite("id_to_object", &Road::id_to_object); // Not working cause of RoadGeometry

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

    // Geometries
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