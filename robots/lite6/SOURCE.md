# Robot description source: UFACTORY Lite 6

This folder contains the URDF and meshes of the UFACTORY Lite 6 used by the
collision-safety layer. The files are third-party. Do not edit them by hand
without recording the change under **Modifications** below. To update them,
re-download from the source, re-apply the modifications and update this file.

## Provenance

| Item | Value |
|---|---|
| Direct source | <https://github.com/hygradme/lite6_urdf> |
| Commit | `c2bd8e5dfaf11a39201c6aa6b5c2429dde0695ac` (2024-12-23) |
| Upstream (original) | <https://github.com/xArm-Developer/xarm_ros2> (`xarm_description/urdf/lite6`) |
| Upstream commit used for verification | `ac736c92b02de7ca7e0be6f737d77d46707464d8` (branch `humble`, 2026-10-20) |

`lite6.urdf` was generated with xacro from the upstream `lite6.urdf.xacro`.

## Verification (2026-11-16)

- `lite6.urdf` (before the modifications below): identical to the direct
  source except line endings (normalized to LF by `.gitattributes`).
- Joint origins, axes and limits of `joint1`–`joint6` and `joint_eef` matched a
  URDF generated from the upstream xacro at the commit above.
- Robot meshes `visual/base.stl` and `visual/link1.stl`–`visual/link6.stl` are
  byte-identical to upstream `xarm_description/meshes/lite6/visual/`.
- The URDF uses the **visual** meshes as collision geometry (upstream also
  ships simplified collision meshes; they are not used here).
- `visual/gripper_lite.stl` and `visual/vacuum_gripper_lite.stl` come from the
  direct source and are **not referenced** by `lite6.urdf`.

## Modifications

### 2026-11-17: joint origins aligned to the controller kinematics

**Why.** The controller computes kinematics with the Modified DH parameters in
the Lite 6 user manual (Appendix 7). The upstream URDF uses slightly different
link lengths and rounds pi, so its forward kinematics differed from the
controller's by up to 1.35 mm (measured in 20 configurations against the
simulator, firmware v2.4.0) with a 1.41 mm analytic bound. The collision layer
must predict link positions for the joint angles the controller will use, so
the model was aligned to the controller.

| Joint | Field | Before | After | Manual DH |
|---|---|---|---|---|
| `joint1` | origin xyz z | 0.2435 | 0.2433 | d1 = 243.3 mm |
| `joint3` | origin xyz x | 0.2002 | 0.2 | a2 = 200 mm |
| `joint4` | origin xyz y | -0.22761 | -0.2276 | d4 = 227.6 mm |
| `joint6` | origin xyz y | 0.0625 | 0.0615 | d6 = 61.5 mm |
| `joint2`–`joint6` | origin rpy | 1.5708 / 3.1416 | pi/2 / pi to full double precision | — |

Meshes are unchanged; they follow their link frames, so they shift by at most
1 mm relative to each other, which is negligible for collision checking.

**Verification.**
- Offline: forward kinematics of the modified URDF vs. the manual DH table over
  2000 random configurations: max error 0.000000 mm (was 1.41 mm).
- Against the controller: `tests/test_fk_simulator.py` (tolerance 0.05 mm,
  0.01 deg).

| File | SHA-1 |
|---|---|
| `lite6.urdf` direct source (original) | `2b357c21d5f11e9e7ed8e9070a8322e5a8f7ef58` |
| `lite6.urdf` after this modification | `36d6bb4480d7fc670c8ed709af051d9c53ebb011` |

### SHA-1 checksums (meshes)

```
be109ae0a2195bb2544325ade5a94dc74ae484af  visual/base.stl
2fd20f164dae91ad36f5151ff9cb7f9d3a24ca8f  visual/link1.stl
a7393179d0947fb94fa13089f29352895d1cff16  visual/link2.stl
4bae5b09a7d1adb8321324a42d164368c6e6e403  visual/link3.stl
6a96d7e24ecee360158fa9c9d7d5458791f1f769  visual/link4.stl
455c8d563722d820e93bc119c0425199e03568e2  visual/link5.stl
1703f86d635a731cb558c7e06e3b05d7ecdda883  visual/link6.stl
336e22e2e1a8d24605b085036ee5ef0c46eff2d5  visual/gripper_lite.stl
ab7d7d44c35a9116a633a6c114bc70a30340ab82  visual/vacuum_gripper_lite.stl
```

## Licenses

Both sources are distributed under BSD-style licenses, which require keeping
the copyright notices, conditions and disclaimers below.

### hygradme/lite6_urdf

```
BSD 3-Clause License

Copyright (c) 2024, hygradme

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### xArm-Developer/xarm_ros2 (UFACTORY)

```
Copyright (c) 2018, UFACTORY Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
