# Frontier Geometry Demo Support Evidence Ray V2 Baseline

This rosbag is the visual baseline captured after replacing the Detector's
three-dimensional support envelope with a fixed-Z, single-voxel inward evidence
ray. It is a replayable RViz artifact, not detector input and not an algorithm
acceptance oracle.

## Capture Contract

- Capture source: support-v2 working tree with the Stage 4 label-layout and
  `support_anchor` marker fixes
- Capture date: 2026-07-18
- ROS distribution: Jazzy
- Scene mode: `combined`
- Composed stages: `true`
- Cave seed: `42`
- Selected phi: `0.0 deg`
- Republish rate: `0.0 Hz`
- Support model: `inward_evidence_ray_v2`
- Anchor marker: one `support_anchor` sphere at the selected direction attempt anchor
- Storage: MCAP with the `zstd_small` storage preset

The Demo node published each transient-local stage exactly once. With no RViz
subscribed, the recorder then joined and received the cached stages. The bag
contains five `visualization_msgs/msg/MarkerArray` messages, one for each topic:

```text
/frontier_geometry_demo/stages/standard_tunnel_geometry/markers
/frontier_geometry_demo/stages/single_ring/markers
/frontier_geometry_demo/stages/bootstrap_yaw_sweep/markers
/frontier_geometry_demo/stages/validated_observation_hop/markers
/frontier_geometry_demo/stages/accumulated_frontier/markers
```

MCAP SHA-256:

```text
d6e8d51fe717f92ec1525598454f8d37fdc0412ede327ba517d0c9c946aa8fda
```

The MCAP file is `163,802` bytes. The five arrays contain 36,825 elements:
36,820 scene markers plus one `DELETEALL` marker per stage. Stage 4 contains
exactly one `support_anchor` at `(4.65, 0.15, 0.05)`.

## Inspect And Replay

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info docs/bags/frontier-geometry-demo-support-evidence-ray-v2
ros2 bag play docs/bags/frontier-geometry-demo-support-evidence-ray-v2
```

For a before/after comparison, replay this bag beside
`frontier-geometry-demo-support-envelope-v1` using separate remapped topics.
Keep both RViz windows on the same saved camera view and render size. Play each
bag once after its RViz subscriber is ready; marker lifetime is infinite, while
looping this short bag needlessly floods RViz with large MarkerArrays.

This bag captures Marker output only. Quantitative behavior changes must still
be evaluated with the real `/global_map` bag analyzer CSV.
