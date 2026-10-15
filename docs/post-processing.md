# Post-processing

> Code is truth — this page describes; the scripts and run layout on `main` win on conflict.

Turn one captured test run's bag and logs into a portable, interactive
report site. Source tool and generated site share a name but live in
different places — do not mix them up.

Where: `post_processing/` source-tree tool (not a ROS executable)
([GitHub](https://github.com/richarJ2002/space_robotics_simulator/blob/main/post_processing/)).
Generated site: `test_runs/<run>/post_processing/` alongside that run's
other artifacts (never committed; never embed bag output here).

Core telemetry: every launch auto-records to
`test_runs/<run>/ros/bags/localisation` with `manifest.json` alongside
it. The launcher exits nonzero rather than completing a run whose
recorder failed to start or crashed immediately.

## Generate

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pip install -r post_processing/requirements.txt
python3 post_processing/post_processing.py --test-run test_runs/<run>
```

Each subsystem script also renders its own page plus shared assets (run
any script with `-h` for `--output-dir`, `--bag`, and image-frame flags;
defaults point at the run's own `post_processing/` and
`ros/bags/localisation`).

## Images are opt-in

`./scripts/launch_simulator.sh --record-images` additionally records the
LocCam / annotated-feature image topics. Leave it off for normal runs:
stereo images at full camera rate add multiple gigabytes to a run.
Without it the visual-odometry page still renders every other section
and says how to recapture with images.

## Relative-pose error (RPE)

Complement to the absolute (ATE) error on each subsystem page:
fixed-window drift per interval instead of accumulated offset.

- Definition: for each estimator sample, the first later sample at
  least one window ahead forms a pair; translational RPE is
  `|estimate displacement − ground-truth displacement|` across the
  pair (metres), rotational RPE is the geodesic angle between the two
  relative rotations (degrees, double-cover safe, so pairs straddling
  the ±π yaw branch cut stay small when both legs agree physically).
  A constant position/attitude offset cancels exactly. Ground truth is
  interpolated to the estimator's own clock first, so 50 Hz filters
  and 2 Hz visual producers score on the same footing.
- Window: `RPE_WINDOW_S = 10.0` s, tolerance `RPE_TOLERANCE_S = 1.0` s
  (`post_processing/python_tools/data/metrics.py`, unit-tested in
  `test/post_processing/test_metrics.py`). Ten seconds carries about
  0.05–0.13 m of ground-truth travel on the circle/yaw/square
  datasets — an order of magnitude above centimetre-level publication
  noise, yet short enough to measure drift per interval.
- Scoreboard: EKF, UKF, visual and keyframe-BA each versus ground
  truth on the circle, yaw and square bags; report RMSE with
  median/p95/max (a single-epoch outlier inflates RMSE while the
  median still shows nominal tracking — see the WP06-08 closeout
  note in `.agents/plans/WP-06/`).
