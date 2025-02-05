// Copyright (c) 2022, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef NAV2_SMOOTHER__SMOOTHER_UTILS_HPP_
#define NAV2_SMOOTHER__SMOOTHER_UTILS_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace smoother_utils
{

/**
 * @class nav2_smoother::PathSegment
 * @brief A segment of a path in start/end indices
 */
struct PathSegment
{
  unsigned int start;
  unsigned int end;
};

typedef std::vector<geometry_msgs::msg::PoseStamped>::iterator PathIterator;
typedef std::vector<geometry_msgs::msg::PoseStamped>::reverse_iterator ReversePathIterator;

inline std::vector<PathSegment> findDirectionalPathSegments(
  const nav_msgs::msg::Path & path)
{
  std::vector<PathSegment> segments;
  PathSegment curr_segment;
  curr_segment.start = 0;

  // Iterating through the path to determine the position of the cusp
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    double oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    double ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    double ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existance of cusp, in the path, using the dot product.
    double dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }

    // Checking for the existance of a differential rotation in place.
    double cur_theta = tf2::getYaw(path.poses[idx].pose.orientation);
    double next_theta = tf2::getYaw(path.poses[idx + 1].pose.orientation);
    double dtheta = angles::shortest_angular_distance(cur_theta, next_theta);
    if (fabs(ab_x) < 1e-4 && fabs(ab_y) < 1e-4 && fabs(dtheta) > 1e-4) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }
  }

  curr_segment.end = path.poses.size() - 1;
  segments.push_back(curr_segment);
  return segments;
}

inline void updateApproximatePathOrientations(
  nav_msgs::msg::Path & path,
  bool & reversing_segment)
{
  double dx, dy, theta, pt_yaw;
  reversing_segment = false;

  // Find if this path segment is in reverse
  dx = path.poses[2].pose.position.x - path.poses[1].pose.position.x;
  dy = path.poses[2].pose.position.y - path.poses[1].pose.position.y;
  theta = atan2(dy, dx);
  pt_yaw = tf2::getYaw(path.poses[1].pose.orientation);
  if (fabs(angles::shortest_angular_distance(pt_yaw, theta)) > M_PI_2) {
    reversing_segment = true;
  }

  // Find the angle relative the path position vectors
  for (unsigned int i = 0; i != path.poses.size() - 1; i++) {
    dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    theta = atan2(dy, dx);

    // If points are overlapping, pass
    if (fabs(dx) < 1e-4 && fabs(dy) < 1e-4) {
      continue;
    }

    // Flip the angle if this path segment is in reverse
    if (reversing_segment) {
      theta += M_PI;  // orientationAroundZAxis will normalize
    }

    path.poses[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
  }
}

geometry_msgs::msg::PoseStamped arcCenter(const geometry_msgs::msg::PoseStamped &pt_prev, const geometry_msgs::msg::PoseStamped &pt, const geometry_msgs::msg::PoseStamped &pt_next)
{
  // Compute direction vectors
  double d1_x = pt.pose.position.x - pt_prev.pose.position.x;
  double d1_y = pt.pose.position.y - pt_prev.pose.position.y;
  double d2_x = pt_next.pose.position.x - pt.pose.position.x;
  double d2_y = pt_next.pose.position.y - pt.pose.position.y;

  // Normalized direction vectors
  double d1_norm = std::sqrt(d1_x * d1_x + d1_y * d1_y);
  double d2_norm = std::sqrt(d2_x * d2_x + d2_y * d2_y);
  double d1_unit_x = d1_x / d1_norm;
  double d1_unit_y = d1_y / d1_norm;
  double d2_unit_x = d2_x / d2_norm;
  double d2_unit_y = d2_y / d2_norm;

  // Compute the cosine of the angle between the two vectors
  double cos_angle = d1_unit_x * d2_unit_x + d1_unit_y * d2_unit_y;

  if (cos_angle < -0.7)
  {
    // In case of cusp, reverse the direction of d2
    d2_x = -d2_x;
    d2_y = -d2_y;
  }

  // Calculate the determinant for the intersection of the two lines.
  double det = d1_x * d2_y - d1_y * d2_x;
  if (std::abs(det) < 1e-4)
  {
    // In case of straight line, return an invalid result (infinity).
    geometry_msgs::msg::PoseStamped inf_pose;
    inf_pose.pose.position.x = std::numeric_limits<double>::infinity();
    inf_pose.pose.position.y = std::numeric_limits<double>::infinity();
    return inf_pose;
  }

  // Calculate midpoints
  geometry_msgs::msg::Pose mid1;
  mid1.position.x = (pt_prev.pose.position.x + pt.pose.position.x) / 2.0;
  mid1.position.y = (pt_prev.pose.position.y + pt.pose.position.y) / 2.0;

  geometry_msgs::msg::Pose mid2;
  mid2.position.x = (pt.pose.position.x + pt_next.pose.position.x) / 2.0;
  mid2.position.y = (pt.pose.position.y + pt_next.pose.position.y) / 2.0;

  // Calculate normal vectors
  double n1_x = -d1_y;
  double n1_y = d1_x;
  double n2_x = -d2_y;
  double n2_y = d2_x;

  // Calculate the circle center
  geometry_msgs::msg::PoseStamped center;
  center.pose.position.x = (mid1.position.x * n2_y - mid2.position.x * n1_y) / det;
  center.pose.position.y = (mid1.position.y * n2_x - mid2.position.y * n1_x) / det;

  return center;
}

inline double calculateAverageTurningRadius(const nav_msgs::msg::Path &path)
{
  std::vector<double> radii;

  for (size_t i = 2; i < path.poses.size(); ++i)
  {
    const auto &pt_prev = path.poses[i - 2];
    const auto &pt = path.poses[i - 1];
    const auto &pt_next = path.poses[i];

    // Calculate the center of the circle formed by three consecutive points.
    auto center = arcCenter(pt_prev, pt, pt_next);

    if (center.pose.position.x != std::numeric_limits<double>::infinity())
    {
      // Compute the turning radius (distance from the center to the current point)
      double radius = std::sqrt((pt.pose.position.x - center.pose.position.x) *
                                    (pt.pose.position.x - center.pose.position.x) +
                                (pt.pose.position.y - center.pose.position.y) *
                                    (pt.pose.position.y - center.pose.position.y));
      radii.push_back(radius);
    }
  }

  // Return the average of the computed radii
  if (!radii.empty())
  {
    double sum = std::accumulate(radii.begin(), radii.end(), 0.0);
    return sum / radii.size();
  }
  return 0.0;
}

inline double calculateAverageTurningRadius(const nav_msgs::msg::Path &path,double& min_turn_r)
{
  std::vector<double> radii;

  for (size_t i = 2; i < path.poses.size(); ++i)
  {
    const auto &pt_prev = path.poses[i - 2];
    const auto &pt = path.poses[i - 1];
    const auto &pt_next = path.poses[i];

    // Calculate the center of the circle formed by three consecutive points.
    auto center = arcCenter(pt_prev, pt, pt_next);

    if (center.pose.position.x != std::numeric_limits<double>::infinity())
    {
      // Compute the turning radius (distance from the center to the current point)
      double radius = std::sqrt((pt.pose.position.x - center.pose.position.x) *
                                    (pt.pose.position.x - center.pose.position.x) +
                                (pt.pose.position.y - center.pose.position.y) *
                                    (pt.pose.position.y - center.pose.position.y));
      radii.push_back(radius);
    }
  }

  // Return the average of the computed radii
  if (!radii.empty())
  {
    double sum = std::accumulate(radii.begin(), radii.end(), 0.0);
    min_turn_r = *std::min_element(radii.begin(), radii.end());
    return sum / radii.size();
  }
  return 0.0;
}

double getSmoothness(const geometry_msgs::msg::PoseStamped &pt_prev,
                     const geometry_msgs::msg::PoseStamped &pt,
                     const geometry_msgs::msg::PoseStamped &pt_next)
{
  // Vector d1 and d2 represent the difference between consecutive points.
  double d1_x = pt.pose.position.x - pt_prev.pose.position.x;
  double d1_y = pt.pose.position.y - pt_prev.pose.position.y;
  double d2_x = pt_next.pose.position.x - pt.pose.position.x;
  double d2_y = pt_next.pose.position.y - pt.pose.position.y;

  // Calculate the difference between the direction vectors (d1 and d2).
  double diff_x = d2_x - d1_x;
  double diff_y = d2_y - d1_y;

  // Return the squared magnitude of the difference vector (this is the smoothness metric).
  return diff_x * diff_x + diff_y * diff_y;
}

double calculatePathSmoothness(const nav_msgs::msg::Path &path)
{
  double smoothness = 0.0;

  for (size_t i = 2; i < path.poses.size(); ++i)
  {
    const auto &pt_prev = path.poses[i - 2];
    const auto &pt = path.poses[i - 1];
    const auto &pt_next = path.poses[i];
    smoothness += getSmoothness(pt_prev, pt, pt_next);
  }

  return smoothness;
}

}  // namespace smoother_utils

#endif  // NAV2_SMOOTHER__SMOOTHER_UTILS_HPP_
