cmake_minimum_required(VERSION 3.24)  # ExternalProject DOWNLOAD_EXTRACT_TIMESTAMP
project(daqp_vendor C CXX)

# DAQP from its pinned release tarball (Phase 5A spec Section 6.1), built static and position independent into a
# staging prefix; only the static library and the headers (include/daqp) are installed with this package.
find_package(ament_cmake REQUIRED)
include(ExternalProject)

set(DAQP_VERSION 0.10.0)
set(DAQP_SHA256 e0a928376fb2ecc6268be28e766bc876d49fa29ba732b65c339ad7ea1619bbd5)

set(staging ${CMAKE_CURRENT_BINARY_DIR}/staging)
ExternalProject_Add(daqp_external
  URL https://github.com/darnstrom/daqp/archive/refs/tags/v${DAQP_VERSION}.tar.gz
  URL_HASH SHA256=${DAQP_SHA256}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=${staging}
    -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_INCLUDEDIR=include -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DNATIVE_OPTIMIZATION=OFF -DTEST=OFF -DBENCHMARK=OFF
  BUILD_BYPRODUCTS ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}daqpstat${CMAKE_STATIC_LIBRARY_SUFFIX})

install(DIRECTORY ${staging}/include/daqp DESTINATION include)
install(FILES ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}daqpstat${CMAKE_STATIC_LIBRARY_SUFFIX} DESTINATION lib)
# MIT: the copyright notice goes with the binaries.
ExternalProject_Get_Property(daqp_external SOURCE_DIR)
install(FILES ${SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME} RENAME LICENSE.daqp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Smoke test against the staged build: a two-variable QP to its known answer.
  file(MAKE_DIRECTORY ${staging}/include/daqp)
  add_library(staged_daqp STATIC IMPORTED)
  set_target_properties(staged_daqp PROPERTIES
    IMPORTED_LOCATION ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}daqpstat${CMAKE_STATIC_LIBRARY_SUFFIX}
    INTERFACE_INCLUDE_DIRECTORIES ${staging}/include/daqp INTERFACE_LINK_LIBRARIES m)
  ament_add_gtest(test_daqp_smoke test/test_daqp_smoke.cpp)
  add_dependencies(test_daqp_smoke daqp_external)
  target_link_libraries(test_daqp_smoke staged_daqp)
endif()

ament_package(CONFIG_EXTRAS daqp_vendor-extras.cmake.in)
