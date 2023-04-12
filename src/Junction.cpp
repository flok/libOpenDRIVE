#include "Junction.h"

namespace odr
{

JunctionLaneLink::JunctionLaneLink(int from, int to) : from(from), to(to) {}

JunctionConnection::JunctionConnection(std::string id, std::string incoming_road, std::string connecting_road, ContactPoint contact_point) :
    id(id), incoming_road(incoming_road), connecting_road(connecting_road), contact_point(contact_point)
{
}

JunctionPriority::JunctionPriority(std::string high, std::string low) : high(high), low(low) {}

JunctionController::JunctionController(std::string id, std::string type, std::uint32_t sequence) : id(id), type(type), sequence(sequence) {}

Junction::Junction(std::string name, std::string id) : name(name), id(id) {}

#ifdef PYTHON_BINDING
void init_junction(nb::module_& m)
{
    nb::class_<JunctionLaneLink>(m, "JunctionLaneLink")
        .def(nb::init<int, int>())
        .def_rw("from", &JunctionLaneLink::from)
        .def_rw("to", &JunctionLaneLink::to);

    nb::enum_<JunctionConnection::ContactPoint>(m, "ContactPoint")
        .value("ContactPoint_None", JunctionConnection::ContactPoint::ContactPoint_None)
        .value("ContactPoint_Start", JunctionConnection::ContactPoint::ContactPoint_Start)
        .value("ContactPoint_End", JunctionConnection::ContactPoint::ContactPoint_End)
        .export_values();

    nb::class_<JunctionConnection>(m, "JunctionConnection")
        .def(nb::init<std::string, std::string, std::string, JunctionConnection::ContactPoint>())
        .def_rw("id", &JunctionConnection::id)
        .def_rw("incoming_road", &JunctionConnection::incoming_road)
        .def_rw("connecting_road", &JunctionConnection::connecting_road)
        .def_rw("contact_point", &JunctionConnection::contact_point)
        .def_rw("lane_links", &JunctionConnection::lane_links);

    nb::class_<JunctionPriority>(m, "JunctionPriority")
        .def(nb::init<std::string, std::string>())
        .def_rw("high", &JunctionPriority::high)
        .def_rw("low", &JunctionPriority::low);

    nb::class_<JunctionController>(m, "JunctionController")
        .def(nb::init<std::string, std::string, std::uint32_t>())
        .def_rw("id", &JunctionController::id)
        .def_rw("type", &JunctionController::type)
        .def_rw("sequence", &JunctionController::sequence);

    nb::class_<Junction>(m, "Junction")
        .def(nb::init<std::string, std::string>())
        .def_rw("name", &Junction::name)
        .def_rw("id", &Junction::id)
        .def_rw("id_to_connection", &Junction::id_to_connection)
        .def_rw("id_to_controller", &Junction::id_to_controller)
        .def_rw("priorities", &Junction::priorities);
}
#endif

} // namespace odr