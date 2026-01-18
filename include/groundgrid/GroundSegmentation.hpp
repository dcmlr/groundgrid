#pragma once
/*
Copyright 2025 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// ros
#include <sensor_msgs/msg/point_cloud2.h>
#include <filters/filter_chain.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Pcl
//#include <pcl_ros/point_cloud.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "velodyne_pointcloud/point_types.h"

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.h>
#include <grid_map_cv/GridMapCvConverter.hpp>

namespace groundgrid {

  struct GroundGrid_Config{
    // Filter point cloud based on the map
    bool map_filtering = true;

    // Use median filter on ground estimation
    bool median_ground_filter = false;

    // if a cell has at least this many points, the variance of just that cell is used instead of the variance of 3x3 or 5x5 patch
    int point_count_cell_variance_thres = 30;

    // maximum laser ring for ground detection consideration
    int max_ring = 1024;

    // if the ground patch layer at a cell is below this value, a cell without the minimum point count can be classified as ground
    double groundpatch_detection_minimum_thres = 1.0;

    // compensates for the geometric dilution of the point density with the distance
    double distance_factor = 0.0001;

    // minimum value for the distance factor
    double min_distance_factor = 0.0005;

    // Points lower than ground height + threshold are considered ground points [m]
    double min_point_height_thres = 0.3;

    // Points close to obstacles which are lower than ground height + threshold are considered ground points [m]
    double min_point_height_obstacle_thres = 0.1;

    // Outlier detection tolerance [m]
    double outlier_tolerance = 0.1;

    // Minimum point count for ground patch detection in percent of expected point count
    double min_ground_patch_detection_point_count_thres = 0.25;

    // Distance from the center from which on the patch size is increased [m]
    double patch_size_change_distance = 20.0;

    // Occupied cells decrease factor [100/x]%
    double occupied_cells_decrease_factor = 5;

    // Occupied cells point count factor [100/x]%
    double occupied_cells_point_count_factor = 20;

    // Minimum ground confidence to consider lower points outliers (5x5 patch)
    double min_outlier_detection_ground_confidence = 1.25;

    // Maximum thread count
    unsigned short max_threads = 4;

    // The LiDAR's horizontal resolution [rad]
    double horizontal_point_ang_dist = 0.00174532925;

    // Ignore points closer than the sqrt of this value [m^2]
    double min_dist_squared = 12.0;
  };

  
class GroundSegmentation {
  public:
    typedef velodyne_pointcloud::PointXYZIR PCLPoint;

    GroundSegmentation() : filter_chain_("grid_map::GridMap") {};
    void init(const size_t dimension, const float& resolution, const GroundGrid_Config& config = GroundGrid_Config(), const bool visualize_segmentation = false);
    sensor_msgs::msg::PointCloud2::SharedPtr filter_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, const PCLPoint& cloudOrigin, const geometry_msgs::msg::TransformStamped& mapToBase, grid_map::GridMap &map);
    void insert_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, const size_t start, const size_t end, const PCLPoint& cloudOrigin, std::vector<std::pair<size_t, grid_map::Index> >& point_index, std::vector<std::pair<size_t, grid_map::Index> >& ignored, std::vector<size_t>& outliers, grid_map::GridMap &map);
    // section defines the section of the map to process (0: top-left, 1: top-right, 2: bottom-left, 3: bottom-right)
    // used for parallel execution
    void detect_ground_patches(grid_map::GridMap &map, unsigned short section) const;
    template<int S> void detect_ground_patch(grid_map::GridMap &map, size_t i, size_t j) const;
    void spiral_ground_interpolation(grid_map::GridMap &map, const geometry_msgs::msg::TransformStamped &toBase) const;
    void interpolate_cell(grid_map::GridMap &map, const size_t x, const size_t y) const;

  protected:
    filters::FilterChain<grid_map::GridMap> filter_chain_;

    grid_map::Matrix expected_points_;

    GroundGrid_Config config_;

    bool visualize_segmentation_ = false;

    rclcpp::Logger logger_ = rclcpp::get_logger("GroundSegmentation");
};
}
