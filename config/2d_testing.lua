-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser_frame",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
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

--[[
MAP_BUILDER.use_trajectory_builder_2d = true

MAX_LASER_RANGE = 5.0 
TRAJECTORY_BUILDER_2D.use_imu_data = false -- default true
TRAJECTORY_BUILDER_2D.min_range = 0.4
TRAJECTORY_BUILDER_2D.max_range = MAX_LASER_RANGE
TRAJECTORY_BUILDER_2D.missing_data_ray_length = MAX_LASER_RANGE -- default 5   set as unoccupied space  was 2, increase helped
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- default 1 (360/120 degrees?)

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 8.0 -- default 50 (compared to max range 30)   (5, 8 no diff)
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5 -- default 0.5
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 8.0 -- default 50 (5, 8 no diff)

-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30. -- default 10  100+ makes tracking unresponsive
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40. -- default 40

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3  -- default 5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02  -- default 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.) -- default math.rad(1.)  lower helped?

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.035 -- default 0.05 
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150 -- default 90  (submap size)  should be 5-20 seconds message amount?

POSE_GRAPH.optimize_every_n_nodes = 0 -- default 90  turn global on/off    lower = more runaway? or- less runaway?
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.2 -- default 0.2  lower=lower latency   increasing causes runaway
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 7. -- default 10.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.4 -- default 1.0
POSE_GRAPH.constraint_builder.min_score = 0.55 -- default 0.55  higher=lower latency
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 -- default 0.3

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 12. -- default 7. (14 seemed a bit much)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(22.)  -- math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 10 -- default 7.

-- POSE_GRAPH.optimization_problem.huber_scale = 1e2 -- default 1e2    higher 1e3 and up = bad results

-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8 -- default 0.6  

-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e2 -- default 1.1e4
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e3   -- default 1e5
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20 -- default 10

-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  -- default 1e5
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e3   -- default 1e5
-- POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false -- default true
]]

--[[
-- ** TURTLEBOT ** 
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false -- default true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
]]

-- ** revo lds ** 
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 6.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65


-- ** backpack2D **
-- MAP_BUILDER.use_trajectory_builder_2d = true
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10


return options

