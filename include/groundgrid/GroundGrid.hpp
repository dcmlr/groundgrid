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

// Pcl
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.h>

// ros msgs
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// tf
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace groundgrid {

/**
 **
 ** @ingroup @@
 */
class GroundGrid {
   public:

    typedef pcl::PointXYZ PCLPoint;

    /** Constructor.
     */
    GroundGrid(rclcpp::Clock::SharedPtr clock);

    /** Destructor.
     */
    virtual ~GroundGrid();

    void onInit();
    void init(const nav_msgs::msg::Odometry::ConstSharedPtr &inOdom);
    std::shared_ptr<grid_map::GridMap> update(const nav_msgs::msg::Odometry::ConstSharedPtr& inOdom);

    const float mResolution = .33f;
    const float mDimension = 120.0f;

   private:
    // tf
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTf2_listener;
    const rclcpp::Logger  mLogger = rclcpp::get_logger("GroundGrid");

    std::shared_ptr<grid_map::GridMap> mMap_ptr;
    geometry_msgs::msg::TransformStamped mTfPosition, mTfUtm, mTfMap;
    geometry_msgs::msg::PoseWithCovarianceStamped mLastPose;
};
}
