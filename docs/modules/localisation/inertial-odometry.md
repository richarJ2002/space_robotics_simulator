# Inertial odometry

> Code is truth — values below describe; YAML/source linked wins on conflict.

Purpose: standalone inertial estimator for diagnostics. Low-pass filters the
IMU, calibrates bias at startup, removes lunar gravity, and integrates pose.
Its filtered acceleration and integrated pose are **not** fused — the ESKF
consumes raw `/alpha/imu` so visual attitude corrections affect all
subsequent gravity projection. A windowed-averaging experiment
(`use_averaged_imu`, default off) can replace the low-pass filter for
comparison; it averages bias-corrected samples only (calibrate on raw,
then average — never the reverse without retune evidence), and either it
replaces the LPF with retune evidence or it is dropped.

Where: `src/localisation/inertial_odometry_node/` plus the experiment
library `src/localisation/inertial_averaging/`
([node](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/localisation/inertial_odometry_node/),
[averager](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/localisation/inertial_averaging/)).

## Topics

| Direction | Name |
|---|---|
| In | `/alpha/imu` |
| Out | `/alpha/localisation/inertial/odometry` |
| Out | `/alpha/localisation/inertial/filtered_imu` |
| Out | `/alpha/diagnostics` (`DiagnosticArray` readiness) |

Names verified from `InertialOdometryNodeClass.h` topic defaults
(`system_name` rooted, default `alpha`). No QoS claims — see source.

## Key files

| Class / unit | Path | GitHub |
|---|---|---|
| `InertialOdometryNode` | `src/localisation/inertial_odometry_node/objects/InertialOdometryNodeClass.h` | [link](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/localisation/inertial_odometry_node/objects/InertialOdometryNodeClass.h) |
| IMU callback | `src/localisation/inertial_odometry_node/methods/handleImuCallBack.cc` | [link](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/localisation/inertial_odometry_node/methods/handleImuCallBack.cc) |
| Gravity removal | `src/localisation/inertial_odometry_node/methods/calculateGravityFreeAccelerationFixed.cc` | [link](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/localisation/inertial_odometry_node/methods/calculateGravityFreeAccelerationFixed.cc) |
| Odometry publish | `src/localisation/inertial_odometry_node/methods/publishOdometry.cc` | [link](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/localisation/inertial_odometry_node/methods/publishOdometry.cc) |
| `InertialAverager` | `src/localisation/inertial_averaging/objects/InertialAveragerClass.h` | [link](https://github.com/richarJ2002/space_robotics_simulator/blob/main/src/localisation/inertial_averaging/objects/InertialAveragerClass.h) |

No function signatures in v1 — see source.

## Parameters

Full authority:
[inertial_odometry.yaml](https://github.com/richarJ2002/space_robotics_simulator/blob/main/parameters/systems/alpha/alpha_localisation/inertial_odometry.yaml).
What each tunes: IMU/odometry/filtered-IMU topic names; odom and base frame
names; low-pass cutoff (Hz); gravity-removal toggle; maximum IMU step (s);
startup calibration sample count; angular-rate deadband (rad/s, floors gyro
noise before integration); averager switch (default raw), count bound
(samples) and time bound (ms) — whichever fills first drops oldest within
the 32-sample cap. Averager state (`averager_samples`, `averager_overflows`)
rides the single `inertial_odometry` diagnostics status and never gates
READY; ground truth is never fused; RPE stays report-only.

```mermaid
graph LR
    imu["/alpha/imu"] --> cal[calibrate on raw]
    cal --> lpf[low-pass - default]
    cal --> avg[windowed average - experiment]
    lpf --> grav[remove lunar gravity]
    avg --> grav
    grav --> integ[integrate pose]
    integ --> odom["/localisation/inertial/odometry"]
    grav --> fimu["/localisation/inertial/filtered_imu"]
    odom -.->|not fused| eskf[ESKF uses raw IMU]
```
