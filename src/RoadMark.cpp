#include "RoadMark.h"

namespace odr
{
RoadMarksLine::RoadMarksLine(std::string road_id,
                             double      lanesection_s0,
                             int         lane_id,
                             double      group_s0,
                             double      width,
                             double      length,
                             double      space,
                             double      t_offset,
                             double      s_offset,
                             std::string name,
                             std::string rule) :
    road_id(road_id),
    lanesection_s0(lanesection_s0), lane_id(lane_id), group_s0(group_s0), width(width), length(length), space(space), t_offset(t_offset),
    s_offset(s_offset), name(name), rule(rule)
{
}

RoadMarkGroup::RoadMarkGroup(std::string road_id,
                             double      lanesection_s0,
                             int         lane_id,
                             double      width,
                             double      height,
                             double      s_offset,
                             std::string type,
                             std::string weight,
                             std::string color,
                             std::string material,
                             std::string lane_change) :
    road_id(road_id),
    lanesection_s0(lanesection_s0), lane_id(lane_id), width(width), height(height), s_offset(s_offset), type(type), weight(weight), color(color),
    material(material), lane_change(lane_change)
{
}

RoadMark::RoadMark(std::string road_id,
                   double      lanesection_s0,
                   int         lane_id,
                   double      group_s0,
                   double      s_start,
                   double      s_end,
                   double      t_offset,
                   double      width,
                   std::string type) :
    road_id(road_id),
    lanesection_s0(lanesection_s0), lane_id(lane_id), group_s0(group_s0), s_start(s_start), s_end(s_end), t_offset(t_offset), width(width), type(type)
{
}

#ifdef PYTHON_BINDING
void init_roadmark(nb::module_& m)
{
    nb::class_<RoadMarksLine>(m, "RoadMarksLine")
        .def(nb::init<std::string, double, int, double, double, double, double, double, double, std::string, std::string>())
        .def_rw("road_id", &RoadMarksLine::road_id)
        .def_rw("lanesection_s0", &RoadMarksLine::lanesection_s0)
        .def_rw("lane_id", &RoadMarksLine::lane_id)
        .def_rw("group_s0", &RoadMarksLine::group_s0)
        .def_rw("width", &RoadMarksLine::width)
        .def_rw("length", &RoadMarksLine::length)
        .def_rw("space", &RoadMarksLine::space)
        .def_rw("t_offset", &RoadMarksLine::t_offset)
        .def_rw("s_offset", &RoadMarksLine::s_offset)
        .def_rw("name", &RoadMarksLine::name)
        .def_rw("rule", &RoadMarksLine::rule);

    nb::class_<RoadMarkGroup>(m, "RoadMarkGroup")
        .def(nb::init<std::string, double, int, double, double, double, std::string, std::string, std::string, std::string, std::string>())
        .def_rw("road_id", &RoadMarkGroup::road_id)
        .def_rw("lanesection_s0", &RoadMarkGroup::lanesection_s0)
        .def_rw("lane_id", &RoadMarkGroup::lane_id)
        .def_rw("width", &RoadMarkGroup::width)
        .def_rw("height", &RoadMarkGroup::height)
        .def_rw("s_offset", &RoadMarkGroup::s_offset)
        .def_rw("type", &RoadMarkGroup::type)
        .def_rw("weight", &RoadMarkGroup::weight)
        .def_rw("color", &RoadMarkGroup::color)
        .def_rw("material", &RoadMarkGroup::material)
        .def_rw("lane_change", &RoadMarkGroup::lane_change)
        .def_rw("roadmark_lines", &RoadMarkGroup::roadmark_lines);

    nb::class_<RoadMark>(m, "RoadMark")
        .def(nb::init<std::string, double, int, double, double, double, double, double, std::string>())
        .def_rw("road_id", &RoadMark::road_id)
        .def_rw("lanesection_s0", &RoadMark::lanesection_s0)
        .def_rw("lane_id", &RoadMark::lane_id)
        .def_rw("group_s0", &RoadMark::group_s0)
        .def_rw("s_start", &RoadMark::s_start)
        .def_rw("s_end", &RoadMark::s_end)
        .def_rw("t_offset", &RoadMark::t_offset)
        .def_rw("width", &RoadMark::width)
        .def_rw("type", &RoadMark::type);
}
#endif

} // namespace odr