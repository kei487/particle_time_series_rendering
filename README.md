# particle_time_series_rendering
地図画像上にパーティクルの分布を時系列で重畳描画する ROS 2 ノードです。

## Features
- `/map` を背景画像として保持し、`/particlecloud` のパーティクル分布を時系列で重ね描画します。
- `overlay_count` 回の描画ごとに、青から赤への色変化で履歴を可視化します。
- `output_mode=publish` では `/particle_trajectory_image` に画像をパブリッシュします。
- `output_mode=save` では `~/save_image` サービスが呼ばれた時に PNG を保存します。

## Parameters
- `output_mode` (`publish` or `save`)
- `overlay_count` (default: `5`)
- `time_interval` (default: `1.0`)
- `save_directory` (default: `/tmp`)
- `particle_topic` (default: `/particlecloud`)
- `particle_topic_type` (default: `geometry_msgs/PoseArray`)
- `output_topic` (default: `/particle_trajectory_image`)

## Build
```bash
colcon build --packages-select particle_time_series_rendering
source install/setup.bash
```

## Run
```bash
ros2 launch particle_time_series_rendering particle_time_series_rendering.launch.py
```

`nav2_msgs/ParticleCloud` を使う場合は、以下のように型パラメータを切り替えてください。

```bash
ros2 run particle_time_series_rendering particle_time_series_rendering_node \
  --ros-args \
  -p particle_topic_type:=nav2_msgs/ParticleCloud
```

## Save Service
```bash
ros2 service call /particle_time_series_rendering/save_image std_srvs/srv/Trigger
```
