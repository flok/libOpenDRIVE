cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(libOpenDrive VERSION 0.3.0 DESCRIPTION ".xodr library")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(MSVC)
SET(CMAKE_CXX_FLAGS "/EHsc")
SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS 1)
else()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
endif()

include_directories(include)
include_directories(thirdparty)

add_library(OpenDrive SHARED
    src/Geometries/Arc.cpp
    src/Geometries/CubicSpline.cpp
    src/Geometries/Line.cpp
    src/Geometries/ParamPoly3.cpp
    src/Geometries/RoadGeometry.cpp
    src/Geometries/Spiral.cpp
    src/Geometries/Spiral/odrSpiral.cpp
    src/Junction.cpp
    src/Lane.cpp
    src/LaneSection.cpp
    src/Mesh.cpp
    src/OpenDriveMap.cpp
    src/RefLine.cpp
    src/Road.cpp
    src/RoadMark.cpp
    src/RoadNetworkMesh.cpp
    src/RoadObject.cpp
    src/RoutingGraph.cpp
    include/Geometries/Arc.h
    include/Geometries/CubicSpline.h
    include/Geometries/Line.h
    include/Geometries/ParamPoly3.h
    include/Geometries/RoadGeometry.h
    include/Geometries/Spiral.h
    include/Geometries/Spiral/odrSpiral.h
    include/Junction.h
    include/Lane.h
    include/LaneSection.h
    include/Mesh.h
    include/OpenDriveMap.h
    include/RefLine.h
    include/Road.h
    include/RoadMark.h
    include/RoadNetworkMesh.h
    include/RoadObject.h
    include/RoutingGraph.h
    thirdparty/pugixml/pugixml.cpp
)

add_executable(test-xodr test.cpp)
target_link_libraries(test-xodr OpenDrive)


if(PYTHON_BINDING)
message("Python Binding building...")
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nanobind)

nanobind_add_module(pyopendrive
    src/Geometries/Arc.cpp
    src/Geometries/CubicSpline.cpp
    src/Geometries/Line.cpp
    src/Geometries/ParamPoly3.cpp
    src/Geometries/RoadGeometry.cpp
    src/Geometries/Spiral.cpp
    src/Geometries/Spiral/odrSpiral.cpp
    src/Junction.cpp
    src/Lane.cpp
    src/LaneSection.cpp
    src/Mesh.cpp
    src/OpenDriveMap.cpp
    src/RefLine.cpp
    src/Road.cpp
    src/RoadMark.cpp
    src/RoadNetworkMesh.cpp
    src/RoadObject.cpp
    src/RoutingGraph.cpp
    src/Binding.cpp
    include/Geometries/Arc.h
    include/Geometries/CubicSpline.h
    include/Geometries/Line.h
    include/Geometries/ParamPoly3.h
    include/Geometries/RoadGeometry.h
    include/Geometries/Spiral.h
    include/Geometries/Spiral/odrSpiral.h
    include/Junction.h
    include/Lane.h
    include/LaneSection.h
    include/Mesh.h
    include/OpenDriveMap.h
    include/RefLine.h
    include/Road.h
    include/RoadMark.h
    include/RoadNetworkMesh.h
    include/RoadObject.h
    include/RoutingGraph.h
    thirdparty/pugixml/pugixml.cpp
    )

target_compile_definitions(pyopendrive PRIVATE PYTHON_BINDING)

# hack to get around the "Debug" and "Release" directories cmake tries to add on Windows
if ( MSVC )
    message("TESt")
    set_target_properties( pyopendrive PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pyopendrive" )
    set_target_properties( pyopendrive PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/pyopendrive" )
    set_target_properties( pyopendrive PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/pyopendrive" )
    # etc for the other available configuration types (MinSizeRel, RelWithDebInfo)
endif ( MSVC )


#set_target_properties(pyopendrive PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pyopendrive")
install(TARGETS pyopendrive LIBRARY DESTINATION .)

endif()
