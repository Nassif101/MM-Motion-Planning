cmake_minimum_required(VERSION 3.24)  # ExternalProject DOWNLOAD_EXTRACT_TIMESTAMP
project(mujoco_vendor CXX)

# MuJoCo from its pinned prebuilt release (MuJoCo plant spec Section 2.1), downloaded only: the shared library, the
# MuJoCo headers (include/mujoco) and the licences are installed with this package. linux aarch64 and x86_64.
find_package(ament_cmake REQUIRED)
include(ExternalProject)

set(MUJOCO_VERSION 3.17.0)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(MUJOCO_ARCH aarch64)
  set(MUJOCO_SHA256 55467a37b74e7d943899768aee820e6754a66811ae5ffa17d2768f0c40fc983a)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  set(MUJOCO_ARCH x86_64)
  set(MUJOCO_SHA256 b3246fc698ca27e05ce36047fe2b6866fe23c9e95a26b3a84dd48f011b85e3e4)
else()
  message(FATAL_ERROR "mujoco_vendor: no pinned MuJoCo release for ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(MUJOCO_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/mujoco-${MUJOCO_VERSION})
ExternalProject_Add(mujoco_external
  SOURCE_DIR ${MUJOCO_SOURCE_DIR}
  URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${MUJOCO_ARCH}.tar.gz
  URL_HASH SHA256=${MUJOCO_SHA256}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS ${MUJOCO_SOURCE_DIR}/lib/libmujoco.so.${MUJOCO_VERSION})
set(SOURCE_DIR ${MUJOCO_SOURCE_DIR})

install(DIRECTORY ${SOURCE_DIR}/include/mujoco DESTINATION include)
install(FILES ${SOURCE_DIR}/lib/libmujoco.so.${MUJOCO_VERSION} ${SOURCE_DIR}/lib/libmujoco.so DESTINATION lib)
install(FILES ${SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME} RENAME LICENSE.mujoco)
install(FILES ${SOURCE_DIR}/THIRD_PARTY_NOTICES DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Smoke test against the downloaded release.
  file(MAKE_DIRECTORY ${SOURCE_DIR}/include)
  add_library(staged_mujoco SHARED IMPORTED)
  set_target_properties(staged_mujoco PROPERTIES
    IMPORTED_LOCATION ${SOURCE_DIR}/lib/libmujoco.so.${MUJOCO_VERSION}
    INTERFACE_INCLUDE_DIRECTORIES ${SOURCE_DIR}/include)
  ament_add_gtest(test_mujoco_smoke test/test_mujoco_smoke.cpp)
  add_dependencies(test_mujoco_smoke mujoco_external)
  target_link_libraries(test_mujoco_smoke staged_mujoco)
  target_compile_definitions(test_mujoco_smoke PRIVATE MUJOCO_VERSION="${MUJOCO_VERSION}")
endif()

ament_package(CONFIG_EXTRAS mujoco_vendor-extras.cmake.in)
