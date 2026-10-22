cmake_minimum_required(VERSION 3.24)  # ExternalProject DOWNLOAD_EXTRACT_TIMESTAMP
project(hpipm_vendor C CXX)

# BLASFEO and HPIPM from pinned release tarballs (Phase 4B-B spec Section 4.1), built static and position independent
# into a staging prefix and installed with this package. Headers under include/blasfeo and include/hpipm.
find_package(ament_cmake REQUIRED)
include(ExternalProject)

set(BLASFEO_VERSION 0.1.4.3)
set(BLASFEO_SHA256 92e33e00be7d2a814ecc982b23bfb86a50b71a168abb9009766801fb957e9046)
set(HPIPM_VERSION 0.1.4)
set(HPIPM_SHA256 1af1041fb6610f5bfcde3e8d8c02731dc52a937e589d65db73e28dcfa8c4cef3)

# The BLASFEO kernels per architecture (native linux/amd64 and linux/arm64); GENERIC forces plain C for both libraries.
option(HPIPM_VENDOR_GENERIC "Build BLASFEO and HPIPM with the GENERIC target" OFF)
if(HPIPM_VENDOR_GENERIC)
  set(blasfeo_target GENERIC)
  set(hpipm_target GENERIC)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
  # X64_AUTOMATIC picks BLASFEO's kernels by running probes on the build host (HASWELL down to GENERIC); HPIPM's AVX
  # target is used only if AVX code runs here too, so the two cannot disagree. Both are then specific to this host's
  # CPU features (README).
  set(blasfeo_target X64_AUTOMATIC)
  include(CheckCSourceRuns)
  set(CMAKE_REQUIRED_FLAGS -mavx)
  check_c_source_runs("#include <immintrin.h>
int main(void) { __m256d a = _mm256_set1_pd(1.0); a = _mm256_add_pd(a, a); return _mm256_cvtsd_f64(a) == 2.0 ? 0 : 1; }"
    HPIPM_VENDOR_HOST_RUNS_AVX)
  unset(CMAKE_REQUIRED_FLAGS)
  if(HPIPM_VENDOR_HOST_RUNS_AVX)
    set(hpipm_target AVX)
  else()
    set(hpipm_target GENERIC)
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(blasfeo_target ARMV8A_ARM_CORTEX_A57)
  set(hpipm_target GENERIC)  # HPIPM's own targets are AVX and GENERIC; BLASFEO carries the NEON kernels
else()
  set(blasfeo_target GENERIC)
  set(hpipm_target GENERIC)
endif()
message(STATUS "hpipm_vendor: BLASFEO ${BLASFEO_VERSION} target ${blasfeo_target}, HPIPM ${HPIPM_VERSION} target ${hpipm_target}")

set(staging ${CMAKE_CURRENT_BINARY_DIR}/staging)
set(common_args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE=-O2 -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF
  -DCMAKE_INSTALL_PREFIX=${staging} -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER})

ExternalProject_Add(blasfeo_external
  URL https://github.com/giaf/blasfeo/archive/refs/tags/${BLASFEO_VERSION}.tar.gz
  URL_HASH SHA256=${BLASFEO_SHA256}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  CMAKE_ARGS ${common_args} -DTARGET=${blasfeo_target} -DBLAS_API=OFF -DBLASFEO_EXAMPLES=OFF -DBLASFEO_TESTING=OFF
    -DBLASFEO_HEADERS_INSTALLATION_DIRECTORY=include/blasfeo
  BUILD_BYPRODUCTS ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}blasfeo${CMAKE_STATIC_LIBRARY_SUFFIX})

ExternalProject_Add(hpipm_external
  URL https://github.com/giaf/hpipm/archive/refs/tags/${HPIPM_VERSION}.tar.gz
  URL_HASH SHA256=${HPIPM_SHA256}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  DEPENDS blasfeo_external
  CMAKE_ARGS ${common_args} -DTARGET=${hpipm_target} -DHPIPM_TESTING=OFF -DBLASFEO_PATH=${staging}
    -DBLASFEO_INCLUDE_DIR=${staging}/include/blasfeo -DHPIPM_BLASFEO_LIB=Static
    -DHPIPM_HEADERS_INSTALLATION_DIRECTORY=include/hpipm
  BUILD_BYPRODUCTS ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}hpipm${CMAKE_STATIC_LIBRARY_SUFFIX})

install(DIRECTORY ${staging}/include/ DESTINATION include)
# BSD-2-Clause: the copyright notices go with the binaries.
ExternalProject_Get_Property(blasfeo_external SOURCE_DIR)
install(FILES ${SOURCE_DIR}/LICENSE.txt DESTINATION share/${PROJECT_NAME} RENAME LICENSE.blasfeo)
ExternalProject_Get_Property(hpipm_external SOURCE_DIR)
install(FILES ${SOURCE_DIR}/LICENSE.txt DESTINATION share/${PROJECT_NAME} RENAME LICENSE.hpipm)
install(FILES ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}hpipm${CMAKE_STATIC_LIBRARY_SUFFIX}
  ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}blasfeo${CMAKE_STATIC_LIBRARY_SUFFIX} DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Smoke test against the staged build: a two-stage OCP QP solved to its known answer.
  file(MAKE_DIRECTORY ${staging}/include/blasfeo ${staging}/include/hpipm)
  add_library(staged_blasfeo STATIC IMPORTED)
  set_target_properties(staged_blasfeo PROPERTIES
    IMPORTED_LOCATION ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}blasfeo${CMAKE_STATIC_LIBRARY_SUFFIX}
    INTERFACE_INCLUDE_DIRECTORIES ${staging}/include/blasfeo INTERFACE_LINK_LIBRARIES m)
  add_library(staged_hpipm STATIC IMPORTED)
  set_target_properties(staged_hpipm PROPERTIES
    IMPORTED_LOCATION ${staging}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}hpipm${CMAKE_STATIC_LIBRARY_SUFFIX}
    INTERFACE_INCLUDE_DIRECTORIES ${staging}/include/hpipm INTERFACE_LINK_LIBRARIES staged_blasfeo)
  ament_add_gtest(test_hpipm_smoke test/test_hpipm_smoke.cpp)
  add_dependencies(test_hpipm_smoke hpipm_external)
  target_link_libraries(test_hpipm_smoke staged_hpipm)
endif()

ament_package(CONFIG_EXTRAS hpipm_vendor-extras.cmake.in)
