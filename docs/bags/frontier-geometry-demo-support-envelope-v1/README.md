# Frontier Geometry Demo Support Envelope V1 Baseline

This rosbag is the visual baseline captured before replacing the Detector's
three-dimensional support envelope with an inward evidence ray. It is a replayable
RViz artifact, not detector input and not an algorithm acceptance oracle.

## Capture Contract

- Capture source: the Demo implementation in the commit containing this bag
- ROS distribution: Jazzy
- Scene mode: `combined`
- Composed stages: `true`
- Cave seed: `42`
- Selected phi: `0.0 deg`
- Republish rate: `0.0 Hz`
- Support model: `volume_envelope_v1`
- Anchor marker: one `support_anchor` sphere at the selected direction attempt anchor
- Storage: MCAP with the `zstd_small` storage preset

The recorder was started before the Demo node. The node published each transient-local
stage exactly once without starting RViz. The bag contains five
`visualization_msgs/msg/MarkerArray` messages, one for each topic:

```text
/frontier_geometry_demo/stages/standard_tunnel_geometry/markers
/frontier_geometry_demo/stages/single_ring/markers
/frontier_geometry_demo/stages/bootstrap_yaw_sweep/markers
/frontier_geometry_demo/stages/validated_observation_hop/markers
/frontier_geometry_demo/stages/accumulated_frontier/markers
```

MCAP SHA-256:

```text
52f0d8e3a364b68ed360b506d8e63efe2c7a9e8fce83a755282b9cdb0e46ab4e
```

## Inspect And Replay

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info docs/bags/frontier-geometry-demo-support-envelope-v1
ros2 bag play docs/bags/frontier-geometry-demo-support-envelope-v1
```

For a before/after comparison, replay the baseline into remapped topics and point a
second set of RViz MarkerArray displays at those topics. Keep both RViz windows on the
same saved camera view and render size. The post-change Demo continues to publish its
normal topics. Play this short bag once after RViz has subscribed; looping it repeatedly
publishes large MarkerArrays without adding comparison evidence.

This bag captures the old visualization exactly. Quantitative behavior changes must
still be evaluated with the real `/global_map` bag analyzer CSV.
