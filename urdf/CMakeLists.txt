find_package(Boost REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

# Some PCL packages export host-specific ISA flags (for example, -march=native and -mavx2). Those flags can change
# Eigen's alignment ABI, giving shared Tesseract types different layouts across libraries. PCL is used only by the URDF
# point-cloud parser, so do not allow its compile options to affect this component on GNU/Clang x86 builds. MSVC PCL
# exports alignment requirements that must remain enabled for PCL headers.
if(CMAKE_CXX_COMPILER_ID MATCHES "^(GNU|Clang|AppleClang)$" AND CMAKE_SYSTEM_PROCESSOR MATCHES
                                                                "^(x86_64|AMD64|i[3-6]86)$")
  foreach(
    PCL_TARGET
    pcl_common
    pcl_io
    pcl_io_ply
    pcl_octree)
    if(TARGET ${PCL_TARGET})
      set_property(TARGET ${PCL_TARGET} PROPERTY INTERFACE_COMPILE_OPTIONS "")
    endif()
  endforeach()
endif()

set(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS} "urdf" PARENT_SCOPE)

add_library(
  urdf
  src/box.cpp
  src/calibration.cpp
  src/capsule.cpp
  src/collision.cpp
  src/cone.cpp
  src/cylinder.cpp
  src/dynamics.cpp
  src/geometry.cpp
  src/inertial.cpp
  src/joint.cpp
  src/limits.cpp
  src/link.cpp
  src/material.cpp
  src/mesh.cpp
  src/mimic.cpp
  src/octomap.cpp
  src/octree.cpp
  src/origin.cpp
  src/point_cloud.cpp
  src/safety_controller.cpp
  src/sdf_mesh.cpp
  src/sphere.cpp
  src/urdf_parser.cpp
  src/visual.cpp
  src/utils.cpp)
add_library(tesseract::urdf ALIAS urdf)
set_target_properties(urdf PROPERTIES OUTPUT_NAME tesseract_urdf)
target_link_libraries(
  urdf
  PUBLIC Boost::boost
         Eigen3::Eigen
         tesseract::common
         tesseract::geometry
         tesseract::scene_graph
         tesseract::collision_bullet
         console_bridge::console_bridge)
target_link_libraries(urdf PRIVATE Boost::boost console_bridge::console_bridge)
target_compile_options(urdf PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(urdf PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(urdf PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})

# [t51b P46 2026-07-12] PCL's cmake config leaks arch flags (-msse4.2 -mfpmath=sse -march=native
# -mavx2) into this target via its imported-target interface. That silently switches Eigen's max
# alignment to 32 bytes IN THIS LIBRARY ONLY (sizeof(Collision) 192 here vs 176 everywhere else),
# so scene-graph objects CONSTRUCTED by the URDF parser have a different layout than the code that
# later destroys them -> heap corruption ("munmap_chunk(): invalid pointer" at Release; latent under
# O0 where symbol-binding luck masked it). Strip the arch flags off the imported PCL targets before
# linking them; point-cloud parsing does not need host-specific SIMD codegen.
# (probe-verified: each pcl_* target's INTERFACE_COMPILE_OPTIONS contains ONLY the arch genex
#  "$<$<COMPILE_LANGUAGE:CXX>:-msse4.2;-mfpmath=sse;-march=native;-mavx2>" -> safe to wipe whole.)
foreach(_pcl_tgt pcl_common pcl_io pcl_octree)
  if(TARGET ${_pcl_tgt})
    set_target_properties(${_pcl_tgt} PROPERTIES INTERFACE_COMPILE_OPTIONS "")
  endif()
endforeach()
target_link_libraries(urdf PRIVATE ${PCL_IO_LIBRARIES})
target_include_directories(urdf PRIVATE ${PCL_INCLUDE_DIRS})
# belt-and-braces: even if a flag slips through a non-target path, force the arch baseline LAST.
target_compile_options(urdf PRIVATE -march=x86-64 -mtune=generic -mno-avx2 -mno-avx)
target_compile_definitions(urdf PUBLIC TESSERACT_PARSE_POINT_CLOUDS="ON")

target_include_directories(urdf PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                       "$<INSTALL_INTERFACE:include>")

target_clang_tidy(urdf ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(urdf PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  urdf
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Configure Package
configure_component(
  COMPONENT urdf
  NAMESPACE tesseract
  TARGETS urdf
  DEPENDENCIES
    Boost
    Eigen3
    console_bridge
    "tesseract COMPONENTS common geometry scene_graph collision_bullet")

# Mark cpp header files for installation
install(FILES include/tesseract/urdf/urdf_parser.h include/tesseract/urdf/utils.h DESTINATION include/tesseract/urdf
        COMPONENT urdf)

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_URDF_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_URDF_ENABLE_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    COMPONENT urdf
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract URDF Components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libpcl-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "pcl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
