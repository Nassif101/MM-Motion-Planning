# hpipm_vendor

BLASFEO and HPIPM for the B5 MPC's HPIPM backend (Phase 4B-B,
`docs/superpowers/specs/2026-10-22-phase4b-b-performance-sweep-design.md` Section 4.1).

| Library | Release | SHA256 of the GitHub tag tarball |
|---|---|---|
| BLASFEO | 0.1.4.3 | `92e33e00be7d2a814ecc982b23bfb86a50b71a168abb9009766801fb957e9046` |
| HPIPM | 0.1.4 | `1af1041fb6610f5bfcde3e8d8c02731dc52a937e589d65db73e28dcfa8c4cef3` |

Both are the newest release tags (2026-10-22) and build together. The build downloads the tarballs (network needed
at build time), checks the hashes, and builds static, position-independent libraries into a staging prefix that is
installed with the package. Headers are under `include/blasfeo` and `include/hpipm`. Consumers use
`find_package(hpipm_vendor REQUIRED)` and link `hpipm_vendor::hpipm`.

If GitHub regenerates a tag's archive (it has happened), the hash check fails the build: verify the tag upstream,
then update the hash here and in `CMakeLists.txt`.

The package files are the project's (Apache-2.0); BLASFEO and HPIPM are BSD-2-Clause, and their licenses are
installed to `share/hpipm_vendor/LICENSE.blasfeo` and `LICENSE.hpipm`.

Targets by architecture:

| Host | BLASFEO | HPIPM |
|---|---|---|
| linux/amd64 | `X64_AUTOMATIC` (probed on the build host) | `AVX` if AVX code runs on the build host, else `GENERIC` |
| linux/arm64 | `ARMV8A_ARM_CORTEX_A57` (NEON kernels) | `GENERIC` (HPIPM's own targets are AVX and GENERIC) |
| `-DHPIPM_VENDOR_GENERIC=ON` | `GENERIC` | `GENERIC` |

**x86 builds are specific to the build host's CPU.** BLASFEO's `X64_AUTOMATIC` runs probes at configure time and keeps
the first kernel set the host executes (HASWELL with AVX2/FMA, then BULLDOZER, SANDY_BRIDGE, CORE, GENERIC), and HPIPM
uses AVX when the host runs it. The libraries can stop with an illegal instruction on an x86 machine with fewer CPU
features: rebuild there, or build with `-DHPIPM_VENDOR_GENERIC=ON` for portable plain-C code. The chosen BLASFEO kernel
is the `TARGET_X64_*` define in the installed `include/blasfeo/blasfeo_target.h`; record it with timings compared
across hosts.

Checked on 2026-10-22: native linux/arm64 build in the dev container with the smoke test
(`test_hpipm_smoke`: a two-stage OCP QP against its Riccati solution) passing, and a one-off emulated linux/amd64
compile in `ros:jazzy-ros-base-noble` (both libraries built). The emulation was a compile check only; normal
development uses native images.
