# daqp_vendor

DAQP for the medium layer's dense-QP backend (Phase 5A, `docs/superpowers/specs/2026-10-29-phase5a-medium-horizon-design.md`
Section 6.1; benchmarked against HPIPM dense and OSQP in plan Task 9).

| Library | Release | SHA256 of the GitHub tag tarball |
|---|---|---|
| DAQP | 0.10.0 (2026-09-26) | `e0a928376fb2ecc6268be28e766bc876d49fa29ba732b65c339ad7ea1619bbd5` |

The newest release tag when it was vendored (2026-10-30). The build downloads the tarball (network needed at build
time), checks the hash, builds DAQP into a staging prefix with its own options (`NATIVE_OPTIMIZATION` off, so a build
runs on any host of the architecture; `PROFILING` off, no timer calls in the solve), and installs only the static library `libdaqpstat.a` and the headers under
`include/daqp`. Consumers use `find_package(daqp_vendor REQUIRED)`, link `daqp_vendor::daqp` and include
`"daqp/api.h"` (C headers: inside `extern "C"`).

If GitHub regenerates the tag's archive, the hash check fails the build: verify the tag upstream, then update the
hash here and in `CMakeLists.txt`.

The package files are the project's (Apache-2.0); DAQP is MIT, and its license is installed to
`share/daqp_vendor/LICENSE.daqp`.
