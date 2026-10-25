-- LeKiwi 2D SLAM, IMU 없음 (BMI160이 없는 장비)
-- 출처: yeon-03/lekiwi-pill-pickup @ 9f4a00cfd9dd15e31e73f96825eb86193ef2a20a
-- 변경 근거: LeKiwi 저장소 docs/hardware/slam_navigation.md

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",            -- Cartographer는 map → odom만 발행한다
  odom_frame = "odom",
  provide_odom_frame = false,          -- odom → base_footprint는 베이스 컨트롤러만 발행한다
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 자기 반사(0.19 m 미만)는 Pi의 lekiwi_scan_filter가 NaN으로 이미 지운다.
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 9.0

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.0

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- Pi 4 코어 4개 중 2개는 모터 드라이버 · 베이스 컨트롤러 · 센서 노드 몫으로 남긴다
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2

return options
