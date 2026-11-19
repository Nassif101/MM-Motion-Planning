# Third-party notices: the Smac Lattice port

The `smac/` sources of this package are a C++ port of upstream code (spec 2 Amendment 1, ADR 0019). Each ported file
keeps its upstream copyright line and names the upstream files it follows. The upstream sources are:

| Upstream | Version | Licence |
|---|---|---|
| [navigation2](https://github.com/ros-navigation/navigation2) `nav2_smac_planner` | tag `1.3.13`, commit `f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501` | Apache-2.0 (full text: `../data/LICENSE.nav2`) |
| navigation2 `nav2_costmap_2d` | same tag | BSD-3-Clause (below); `footprint_collision_checker.cpp` is Apache-2.0 (Intel Corporation, 2019) |
| navigation2 `nav2_util` (`line_iterator.hpp`) | same tag | BSD-3-Clause (Willow Garage, 2012; same terms as below) |
| [OMPL](https://github.com/ompl/ompl) `DubinsStateSpace` | tag `1.7.0`, commit `eabb4d0f825666e4553a2ec0ecb9c7a5a33637dd` | BSD-3-Clause (below) |

The container's installed packages are `ros-jazzy-nav2-smac-planner` and `ros-jazzy-nav2-costmap-2d`
`1.3.13-1noble.20260921`, and `ros-jazzy-ompl` `1.7.0-2noble.20260305`. The installed Smac headers equal the tag's
byte for byte. The SHA-256 of each followed upstream file is recorded in
`docs/experiments/plant-routing/implementation.md` (section "Smac Lattice port").

Changes from upstream are listed in spec 2 Amendment 1, section A1.2. In short: ROS types, messages and logging are
removed; static members become per-planner state; the search loop can pause between iterations.

## nav2_costmap_2d (BSD-3-Clause)


Software License Agreement (BSD License)

Copyright (c) 2008, 2013, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.
* Neither the name of Willow Garage, Inc. nor the names of its
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

## OMPL (BSD-3-Clause)


Software License Agreement (BSD License)

Copyright (c) 2010, Rice University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.
* Neither the name of the Rice University nor the names of its
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
