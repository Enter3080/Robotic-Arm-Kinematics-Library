# Robotic-Arm-Kinematics-Library
# my_lib

A lightweight, robot-agnostic C++ kinematics library for 6-DOF serial
manipulators, packaged as a ROS 2 `ament_cmake` package. It provides
forward kinematics, a position Jacobian, and SVD-based pseudoinverse
inverse kinematics using Craig's **Modified Denavit–Hartenberg (MDH)**
convention.

No robot-specific data is hard-coded: DH parameters, joint-axis offsets,
and joint limits are all injected by the caller, so the same library can
drive any 6-DOF arm.

## Features

- Forward kinematics with Modified DH convention:
  `A_i = Rx(α_{i−1}) · Tx(a_{i−1}) · Rz(θ_i) · Tz(d_i)`
- Position Jacobian (3×6) via the vector-cross-product method
- Numerical inverse kinematics with Moore–Penrose pseudoinverse (SVD)
- Joint-limit handling with **boundary projection** (prevents the solver
  from stalling at a limit) and continuous-joint normalization to [−π, π]
- Generic by design: `set_theta_offsets()` and `set_joint_limits()` let
  callers adapt the library to their own robot
- Clean `ament` package with exported targets and include directories

## Repository layout

```
my_lib/
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── include/
│   └── my_lib/
│       └── kinematics.h
└── src/
    └── kinematics.cpp
```

## Requirements

- Ubuntu 22.04 / 24.04
- ROS 2 Humble / Jazzy
- Eigen3 (`libeigen3-dev`)

## Building

```bash
mkdir -p ~/kinematics_ws/src
cd ~/kinematics_ws/src
git clone [<repository-url>](https://github.com/Enter3080/Robotic-Arm-Kinematics-Library.git) my_lib
cd ~/kinematics_ws
colcon build --packages-select my_lib
source install/setup.bash
```

## Using it in your own package

Add to your package's `package.xml`:

```xml
<depend>my_lib</depend>
<depend>eigen</depend>
```

In your `CMakeLists.txt`:

```cmake
find_package(my_lib REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(my_ik_demo src/my_ik_demo.cpp)

target_include_directories(my_ik_demo PRIVATE
  ${my_lib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)
target_link_libraries(my_ik_demo ${my_lib_LIBRARIES})
```

> If `find_package(my_lib)` resolves but the include path or library is
> not picked up in your environment, the deterministic fallback is:
> `find_library(MY_LIB_LIBRARY NAMES my_lib REQUIRED)` and linking
> `${MY_LIB_LIBRARY}` directly.

## Quick start

```cpp
#include "my_lib/kinematics.h"

#include <Eigen/Dense>

#include <array>
#include <iostream>

int main()
{
    constexpr double PI = kinematics::Kinematics::PI;

    const std::array<double, 6> alpha = {0.0, -PI / 2, 0.0, PI / 2, -PI / 2, PI / 2};
    const std::array<double, 6> a     = {0.0, 0.0, 0.6, 0.0, 0.0, 0.0};
    const std::array<double, 6> d     = {0.6, 0.0, 0.0, 0.6, 0.0, 0.05};
    const std::array<double, 6> q0    = {0.0, PI / 2, PI / 4, 0.0, 0.0, 0.0};

    const std::array<Eigen::Vector3d, 3> targets = {
        Eigen::Vector3d(0.9, 0.1, 0.2),
        Eigen::Vector3d(0.7, 0.1, 0.3),
        Eigen::Vector3d(0.6, 0.2, 0.4)
    };

    for (const auto& t : targets) {
        kinematics::Kinematics kin(alpha, a, d);
        kin.set_theta_offsets({0.0, -PI / 2, PI / 2, 0.0, 0.0, 0.0});
        kin.set_joint_limits(
            {-3.14, 0.0, 0.0, -3.14, -1.57, -1e9},
            { 3.14, 2.5, 2.5,  3.14,  1.57,  1e9});
        kin.set_joint_angles(q0);

        const bool ok = kin.solve_inverse_kinematics(t, 1000, 1e-4, 0.2, 0.1);
        const double err = (kin.end_position() - t).norm();

        std::cout << "target [" << t.transpose() << "] -> "
                  << (ok ? "converged" : "FAILED")
                  << ", residual " << err << " m\n";
    }
    return 0;
}
```

## Key API

| Method | Purpose |
|---|---|
| `Kinematics(alpha, a, d)` | Constructor with MDH parameters |
| `set_joint_angles(q)` / `get_joint_angles()` | Set / read joint angles (rad) |
| `set_theta_offsets(offsets)` | Fixed MDH θ offsets (robot-specific) |
| `set_joint_limits(min, max)` | Joint limits; `±1e9` = continuous joint |
| `total_transform()` | Base → end-effector 4×4 transform |
| `end_position()` | End-effector xyz position |
| `position_jacobian()` | 3×6 position Jacobian |
| `pseudo_inverse(M, tol)` | Moore–Penrose pseudoinverse via SVD |
| `solve_inverse_kinematics(target, ...)` | Iterative IK; returns convergence |
| `apply_joint_limits()` | Clamp / normalize current q |

## Algorithm notes

- **MDH convention** — each link transform is
  `Rx(α_{i−1}) · Tx(a_{i−1}) · Rz(θ_i) · Tz(d_i)`, with
  `θ_i = q_i + offset_i`. Offsets let the library match URDF joints that
  carry a fixed ±π/2 origin rotation.
- **Jacobian** — for each revolute joint:
  `J_v,i = z_i × (p_e − p_i)`, where `z_i` is the joint axis expressed in
  the base frame and `p_e` the current end-effector position.
- **Pseudoinverse** — computed with `JacobiSVD`; singular values below
  `relative_tolerance × max(m,n) × σ_max` are truncated.
- **IK update** — `Δq = η · J⁺ · e`, with the per-step update clamped to
  `max_joint_step` to prevent oscillation.
- **Limit projection** — before applying `Δq`, any component pushing a
  joint further past its limit is zeroed. This prevents the classic
  failure mode where the solver keeps stepping into a limit and stalls.

## Validation

The library was validated on a 6-DOF arm (robot model and ROS nodes kept
in a separate, private project):

- 20 reachable workspace targets, all converged with **final end-effector
  error < 1e-4 m**;
- unreachable targets return `false` after the iteration budget is
  exhausted (no hang).

## Limitations

- Position-only IK (3×6 Jacobian); end-effector orientation is not
  controlled.
- No redundancy resolution / null-space optimisation.
- Numerical IK convergence depends on the starting pose; targets near the
  fully-vertical / singular configuration may exhaust the iteration budget
  even when reachable.
- This is a kinematics library only — no dynamics, no trajectory
  generation, no ROS nodes.

## Roadmap

- Damped least squares (DLS) with adaptive damping
- Orientation-aware IK (6×6 Jacobian)
- Unit tests for FK/IK round-trip verification

## License

Apache-2.0. See [LICENSE](LICENSE).

Copyright © 2026 Entan Zhang
