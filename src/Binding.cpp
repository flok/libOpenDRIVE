
#include <OpenDriveMap.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/bind_vector.h>

namespace nb = nanobind;
using namespace odr;

NB_MODULE(pyopendrive, m) {

	init_opendrivemap(m);

}