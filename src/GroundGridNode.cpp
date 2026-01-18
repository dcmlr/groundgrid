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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>

// ros msgs
#include <nav_msgs/msg/odometry.hpp>

// Pcl
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/io/pcd_io.h> // point cloud reading/writing
#include <pcl_conversions/pcl_conversions.h> // convert from/to ros


// ros opencv transport
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>

// ros tf
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <sstream>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.h>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// grid map
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/empty.hpp>

#include <groundgrid/GroundGrid.hpp>
#include <groundgrid/GroundSegmentation.hpp>

namespace groundgrid{

enum DATASET {
    KITTI,
    KITTI_360,
    MULRAN,
    HELIPR,
    PCD
};
enum LiDAR {
    VELODYNE_32,
    VELODYNE_64,
    VELODYNE_128,
    AEVA,
    AVIA,
    OUSTER_64,
    OUSTER_128
};
/**
 **
 **
 ** @ingroup @@
 */
class GroundGridNode : public rclcpp::Node {
   public:
    typedef velodyne_pointcloud::PointXYZIR PCLPoint;

    /** Constructor.
     */
    GroundGridNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) : 
        rclcpp::Node("GroundGridNode", node_options),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_){ 
        
        grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("/groundgrid/grid_map", rclcpp::SensorDataQoS());
        filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/groundgrid/filtered_cloud", rclcpp::ServicesQoS());

        groundgrid_ = std::make_shared<GroundGrid>(get_clock());

        // retrieve dataset parameters for evaluation
        auto param_dataset_name = rcl_interfaces::msg::ParameterDescriptor{};
        param_dataset_name.description = "Name of the dataset (MulRan, KITTI, ...)";
        dataset_name = declare_parameter<std::string>("groundgrid/dataset_name", "kitti", param_dataset_name);
        // convert to lower case
        std::transform(dataset_name.begin(), dataset_name.end(), dataset_name.begin(), [](unsigned char c){return std::tolower(c);});
        auto param_dataset_path = rcl_interfaces::msg::ParameterDescriptor{};
        param_dataset_path.description = "Path to the dataset";
        dataset_path = declare_parameter<std::string>("groundgrid/dataset_path", "", param_dataset_path);
        auto param_seq = rcl_interfaces::msg::ParameterDescriptor{};
        param_seq.description = "Selected sequence of the dataset";
        sequence = declare_parameter<std::string>("groundgrid/sequence", "00", param_seq);
        if(sequence.length() < 2)
            sequence.insert(sequence.begin(), '0');
        auto param_sensor = rcl_interfaces::msg::ParameterDescriptor{};
        param_sensor.description = "Selected sensor of the dataset";
        sensor = declare_parameter<std::string>("groundgrid/sensor", "velodyne", param_seq);
        auto param_poses = rcl_interfaces::msg::ParameterDescriptor{};
        param_poses.description = "Odometry poses file";
        poses_path = declare_parameter<std::string>("groundgrid/poses_file", "poses.txt", param_poses);
        auto param_poses_delim = rcl_interfaces::msg::ParameterDescriptor{};
        param_poses_delim.description = "Odometry poses file delimiter character";
        poses_delimiter = declare_parameter<std::string>("groundgrid/poses_delimiter", " ", param_poses_delim);
        auto param_frame_cap = rcl_interfaces::msg::ParameterDescriptor{};
        param_frame_cap.description = "Cap the processing frame rate to the specified frequency (0 = unlimited)";
        frame_rate_cap_ = declare_parameter<double>("groundgrid/frame_rate_cap", 0, param_frame_cap);

        // GroundGrid parameter handling
        GroundGrid_Config config_gg;
        auto param_pc_cell_var_thres = rcl_interfaces::msg::ParameterDescriptor{};
        param_pc_cell_var_thres.description = "[int] If a cell has at least this many points, the variance of just that cell is used instead of the variance of 3x3 or 5x5 patch";
        config_gg.point_count_cell_variance_thres = declare_parameter<int>("groundgrid/point_count_cell_variance_thres", 30, param_pc_cell_var_thres);
        auto param_max_ring = rcl_interfaces::msg::ParameterDescriptor{};
        param_max_ring.description = "[int] Maximum laser ring for ground detection consideration";
        config_gg.max_ring = declare_parameter<int>("groundgrid/max_ring", 1024, param_max_ring);
        auto param_gp_min_thres = rcl_interfaces::msg::ParameterDescriptor{};
        param_gp_min_thres.description = "[double] If the ground patch layer at a cell is below this value, a cell without the minimum point count can be classified as ground";
        config_gg.groundpatch_detection_minimum_thres = declare_parameter<double>("groundgrid/groundpatch_detection_minimum_thres", 1.0, param_gp_min_thres);
        auto param_dist_factor = rcl_interfaces::msg::ParameterDescriptor{};
        param_dist_factor.description = "[double] Compensates for the geometric dilution of the point density with the distance";
        config_gg.distance_factor = declare_parameter<double>("groundgrid/distance_factor", 0.0001, param_dist_factor);
        auto param_min_dist_factor = rcl_interfaces::msg::ParameterDescriptor{};
        param_min_dist_factor.description = "[double] Minimum value for the distance factor";
        config_gg.min_distance_factor = declare_parameter<double>("groundgrid/min_distance_factor", 0.0005, param_min_dist_factor);
        auto param_min_point_height_thres = rcl_interfaces::msg::ParameterDescriptor{};
        param_min_point_height_thres.description = "[double] Points lower than ground height + threshold are considered ground points [m]";
        config_gg.min_point_height_thres = declare_parameter<double>("groundgrid/min_point_height_thres", 0.3, param_min_point_height_thres);
        auto param_min_point_height_obstacle_thres = rcl_interfaces::msg::ParameterDescriptor{};
        param_min_point_height_obstacle_thres.description = "[double] Points close to obstacles which are lower than ground height + threshold are considered ground points [m]";
        config_gg.min_point_height_obstacle_thres = declare_parameter<double>("groundgrid/min_point_height_obstacle_thres", 0.1, param_min_point_height_obstacle_thres);
        auto param_outlier_tolerance = rcl_interfaces::msg::ParameterDescriptor{};
        param_outlier_tolerance.description = "[double] Outlier detection tolerance [m]";
        config_gg.outlier_tolerance = declare_parameter<double>("groundgrid/outlier_tolerance", 0.1, param_outlier_tolerance);
        auto param_min_ground_patch_detection_point_count_thres = rcl_interfaces::msg::ParameterDescriptor{};
        param_min_ground_patch_detection_point_count_thres.description = "[double] Minimum point count for ground patch detection in percent of expected point count";
        config_gg.min_ground_patch_detection_point_count_thres = declare_parameter<double>("groundgrid/min_ground_patch_detection_point_count_thres", 0.25, param_min_ground_patch_detection_point_count_thres);
        auto param_patch_size_change_distance = rcl_interfaces::msg::ParameterDescriptor{};
        param_patch_size_change_distance.description = "[double] Distance from the center from which on the patch size is increased [m]";
        config_gg.patch_size_change_distance = declare_parameter<double>("groundgrid/patch_size_change_distance", 20.0, param_patch_size_change_distance);
        auto param_occupied_cells_dec_factor = rcl_interfaces::msg::ParameterDescriptor{};
        param_occupied_cells_dec_factor.description = "[double] Occupied cells decrease factor [100/x]%";
        config_gg.occupied_cells_decrease_factor = declare_parameter<double>("groundgrid/occupied_cells_decrease_factor", 5.0, param_occupied_cells_dec_factor);
        auto param_occupied_cells_point_count_factor = rcl_interfaces::msg::ParameterDescriptor{};
        param_occupied_cells_point_count_factor.description = "[double] Occupied cells point count factor [100/%]";
        config_gg.occupied_cells_point_count_factor = declare_parameter<double>("groundgrid/occupied_cells_point_count_factor", 20.0, param_occupied_cells_point_count_factor);
        auto param_min_outlier_detection_ground_confidence = rcl_interfaces::msg::ParameterDescriptor{};
        param_min_outlier_detection_ground_confidence.description = "[double] Minimum ground confidence to consider lower points outliers (5x5 patch)";
        config_gg.min_outlier_detection_ground_confidence = declare_parameter<double>("groundgrid/min_outlier_detection_ground_confidence", 1.25, param_min_outlier_detection_ground_confidence);
        auto param_max_threads = rcl_interfaces::msg::ParameterDescriptor{};
        param_max_threads.description = "[int] Maximum thread count";
        config_gg.max_threads = declare_parameter<int>("groundgrid/max_threads", 4, param_max_threads);
        auto param_horizontal_point_ang_dist = rcl_interfaces::msg::ParameterDescriptor{};
        param_horizontal_point_ang_dist.description = "[double] The LiDAR's horizontal resolution [rad]";
        config_gg.horizontal_point_ang_dist = declare_parameter<double>("groundgrid/horizontal_point_ang_dist", 0.00174532925, param_horizontal_point_ang_dist);
        auto param_min_dist_squared = rcl_interfaces::msg::ParameterDescriptor{};
        param_min_dist_squared.description = "[double] Ignore points closer than the sqrt of this value [m^2]";
        config_gg.min_dist_squared = declare_parameter<double>("groundgrid/min_dist_squared", 12.0, param_min_dist_squared);

        auto param_visualize = rcl_interfaces::msg::ParameterDescriptor{};
        param_visualize.description = "Real-time visualization (impacts run-time performance)";
        bool visualize = declare_parameter<bool>("groundgrid/visualize", false, param_visualize);
        auto param_eval = rcl_interfaces::msg::ParameterDescriptor{};
        param_eval.description = "Evaluation mode: wait for clouds to be processed";
        eval_ = declare_parameter<bool>("groundgrid/evaluation", false, param_eval);

        ground_segmentation_.init(groundgrid_->mDimension, groundgrid_->mResolution, config_gg, visualize);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // odometry in
        pos_sub_ = create_subscription<nav_msgs::msg::Odometry>("/groundgrid/odometry_in", rclcpp::SensorDataQoS(), std::bind(&groundgrid::GroundGridNode::odom_callback, this, std::placeholders::_1));
        sync_sub_ = create_subscription<std_msgs::msg::Empty>("/groundgrid/next_cloud", rclcpp::ServicesQoS(), std::bind(&groundgrid::GroundGridNode::next_cloud_trigger, this, std::placeholders::_1));
        
        // input point cloud
        points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/pointcloud", rclcpp::SystemDefaultsQoS(), std::bind(&groundgrid::GroundGridNode::points_callback, this, std::placeholders::_1));

        groundgrid_->onInit();
   }
   

    /** Destructor.
     */
    virtual ~GroundGridNode() {
    }

    virtual void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& inOdom){
        auto start = std::chrono::steady_clock::now();
        if(inOdom->header.frame_id != "odom"){
            geometry_msgs::msg::TransformStamped transform;
            getTransform(inOdom->header.frame_id, "odom", transform);
            geometry_msgs::msg::PoseStamped ps;
            tf2::doTransform(inOdom->pose.pose, ps.pose, transform);
            nav_msgs::msg::Odometry odom = *inOdom;
            odom.pose.pose = ps.pose;
            map_ptr_ = groundgrid_->update(std::make_shared<nav_msgs::msg::Odometry>(odom));
        }
        else
            map_ptr_ = groundgrid_->update(inOdom);
        auto end = std::chrono::steady_clock::now();
        RCLCPP_DEBUG_STREAM(get_logger(), "grid map update took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
    }

    virtual void next_cloud_trigger(const std_msgs::msg::Empty::ConstSharedPtr& msg){
        ++current_cloud;
    }
    
    void setTransform(const geometry_msgs::msg::TransformStamped& transform_in){
        if(!tf_buffer_.setTransform(transform_in, "user"))
            RCLCPP_ERROR(get_logger(), "Failed to set transform!");
    }

    void setLiDAR(const LiDAR& lidar){
        if(lidar == VELODYNE_64){ //KITTI
            RCLCPP_INFO(get_logger(), "Using KITTI parameters");
            variance_factor_ = 0.35f;
            normals_factor_ = 0.09f;
            intensity_factor_ = 0.375f;
            intensity_shift_ = 0.075f;
        }
        else if(lidar == OUSTER_64){ //MulRan
            RCLCPP_INFO(get_logger(), "Using MulRan parameters");
            variance_factor_ = 0.35f;
            normals_factor_ = 0.1f;
            intensity_factor_ = 1.0f;
            intensity_shift_ = 0.075f;
        }
        else if(lidar == OUSTER_128){ // HeLIPR Ouster 128
            RCLCPP_INFO(get_logger(), "Using HeLIPR Ouster parameters");
            variance_factor_ = 0.35f;
            normals_factor_ = 0.1f;
            intensity_factor_ = 200.0f;
            intensity_shift_ = 0.0f;
        }
        else if(lidar == AVIA){ // HeLIPR Avia
            RCLCPP_INFO(get_logger(), "Using HeLIPR Avia parameters");
            variance_factor_ = 0.35f;
            normals_factor_ = 0.09f;
            intensity_factor_ = 50.0f;
            intensity_shift_ = 0.0f;
        }
        else if(lidar == AEVA){ // HeLIPR Aeva
            RCLCPP_INFO(get_logger(), "Using HeLIPR Aeva parameters");
            variance_factor_ = 0.35f;
            normals_factor_ = 0.1f;
            intensity_factor_ = 1.0f;
            intensity_shift_ = 0.0f;
        }
        else if(lidar == VELODYNE_32){ // HeLIPR Velo16
            RCLCPP_INFO(get_logger(), "Using HeLIPR Aeva parameters");
            variance_factor_ = 0.35f;
            normals_factor_ = 0.1f;
            intensity_factor_ = 64.0f;
            intensity_shift_ = 0.0f;
        }
        else if(lidar == VELODYNE_128){ // Velodyne Alpha Prime 128
            RCLCPP_INFO(get_logger(), "Using Velodyne Alpha Prime parameters");
            variance_factor_ = 0.35f;
            normals_factor_ = 0.1f;
            intensity_factor_ = 50.0f;
            intensity_shift_ = 0.0f;
        }
        else
            RCLCPP_ERROR(get_logger(), "No parameter set for LiDAR found!");
    }

    bool getTransform(const std::string& source_frame_id, const std::string& target_frame_id,  
                      geometry_msgs::msg::TransformStamped& transform_out, const tf2::TimePoint& stamp = tf2::TimePointZero){
        try{
            transform_out = tf_buffer_.lookupTransform(target_frame_id, source_frame_id, stamp);
        }
        catch(tf2::TransformException& ex){
            RCLCPP_DEBUG_STREAM(get_logger(), "Failed to get requested transform from " << source_frame_id << " to " << target_frame_id << " reason: " << ex.what());
            return false;
        }
        return true;
    }

    virtual void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg){
        auto start = std::chrono::steady_clock::now();
        static size_t time_vals = 0;
        static double avg_time = 0.0;
        static double avg_cpu_time = 0.0;
        static size_t cloud_count = 0;
        ++cloud_count;
        geometry_msgs::msg::TransformStamped mapToBaseTransform, cloudOriginTransform;

        // Map not initialized yet, this means the node hasn't received any odom message so far.
        if(!map_ptr_)
            return;

        try{
            tf_buffer_.canTransform("base_link", "odom", cloud_msg->header.stamp, rclcpp::Duration(1,std::nano::den/10));
            mapToBaseTransform = tf_buffer_.lookupTransform("odom", "base_link", cloud_msg->header.stamp);
            tf_buffer_.canTransform(cloud_msg->header.frame_id, "odom", cloud_msg->header.stamp, rclcpp::Duration(0,std::nano::den/10));
            cloudOriginTransform = tf_buffer_.lookupTransform("odom", "velodyne", cloud_msg->header.stamp);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Could not get transform for cloud %s",ex.what());
            return;
        }


        geometry_msgs::msg::PointStamped origin;
        origin.header = cloud_msg->header;
        origin.header.frame_id = "velodyne";
        origin.point.x = 0.0f;
        origin.point.y = 0.0f;
        origin.point.z = 0.0f;        
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_transformed = sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2());
        cloud_transformed->header = cloud_msg->header;
        cloud_transformed->header.frame_id = "odom";

        tf2::doTransform(origin, origin, cloudOriginTransform);

        geometry_msgs::msg::TransformStamped revtransformStamped;
        // Transform cloud into map coordinate system
        if(cloud_msg->header.frame_id != "odom"){
            // Transform to map
            geometry_msgs::msg::TransformStamped transformStamped;

            try{
                transformStamped = tf_buffer_.lookupTransform("odom", cloud_msg->header.frame_id, cloud_msg->header.stamp, rclcpp::Duration(0,std::nano::den/10));
                revtransformStamped = tf_buffer_.lookupTransform(cloud_msg->header.frame_id, "odom", cloud_msg->header.stamp, rclcpp::Duration(0,std::nano::den/10));
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "Failed to get map transform for point cloud transformation: %s",ex.what());
                return;
            }
            tf2::doTransform(*cloud_msg, *cloud_transformed, transformStamped);
        }

        auto end = std::chrono::steady_clock::now();
        RCLCPP_DEBUG_STREAM(get_logger(), "cloud transformation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");

        auto start2 = std::chrono::steady_clock::now();
        const auto& mapSize = map_ptr_->getSize();
	      std::clock_t c_clock = std::clock();
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_out;
        PCLPoint origin_pclPoint;
        origin_pclPoint.x = origin.point.x;
        origin_pclPoint.y = origin.point.y;
        origin_pclPoint.z = origin.point.z;

        // Ground Grid - Ground Point Removal
        cloud_msg_out = ground_segmentation_.filter_cloud(cloud_transformed, origin_pclPoint, mapToBaseTransform, *map_ptr_);
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start2;
        const double milliseconds = elapsed_seconds.count() * 1000;
        const double c_millis = double(std::clock() - c_clock)/CLOCKS_PER_SEC * 1000;
        avg_time = (milliseconds + time_vals * avg_time)/(time_vals+1);
        avg_cpu_time = (c_millis + time_vals * avg_cpu_time)/(time_vals+1);
        ++time_vals;
        RCLCPP_DEBUG_STREAM(get_logger(), "groundgrid took " << milliseconds << "ms (avg: " << avg_time << "ms)");
        RCLCPP_DEBUG_STREAM(get_logger(), "total cpu time used: " << c_millis << "ms (avg: " << avg_cpu_time << "ms)") ;

        cloud_msg_out->header = cloud_msg->header;
        if(filtered_cloud_pub_->get_subscription_count())
        {
            //put cloud back to sensor frame
            tf2::doTransform(*cloud_msg_out, *cloud_msg_out, revtransformStamped);
            filtered_cloud_pub_->publish(*cloud_msg_out);
        }

        auto& map = *map_ptr_;

        // ros2 removed the header sequence field, so we have to count manually.
        static size_t seq = 0;
        ++seq;

        if(grid_map_pub_->get_subscription_count()){    
            grid_map_msgs::msg::GridMap::UniquePtr grid_map_msg = grid_map::GridMapRosConverter::toMessage(*map_ptr_);
            grid_map_msg->header.stamp = cloud_msg->header.stamp;
            grid_map_pub_->publish(std::move(grid_map_msg));
        }

        image_transport::ImageTransport it(shared_from_this());

        for(const auto& layer : map_ptr_->getLayers()){
            if(layer_pubs_.find(layer) == layer_pubs_.end()){
                layer_pubs_[layer] = it.advertise("/groundgrid/grid_map_cv_"+layer, 10);
            }
            publish_grid_map_layer(layer_pubs_.at(layer), layer, seq, cloud_msg->header.stamp);
        }

        static size_t send_imgs = 0;
        ++send_imgs;

        end = std::chrono::steady_clock::now();
        RCLCPP_DEBUG_STREAM(get_logger(), "overall " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms - send " << send_imgs << " imgs");
    }

    // transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // next cloud to process, used for evalutation
    long current_cloud = -1;

    // name of the dataset
    std::string dataset_name;
    // path to the dataset
    std::string dataset_path;
    // path to the odometry poses file
    std::string poses_path;
    // delimiter of the pose file
    std::string poses_delimiter;
    // sequence number
    std::string sequence;
    // LiDAR sensor
    std::string sensor;
    // frame rate cap
    float frame_rate_cap_ = 0.0f;
    // evaluation mode: groundgrid will wait for the evaluation script
    bool eval_ = false;

    
   protected:
    void publish_grid_map_layer(const image_transport::Publisher& pub, const std::string& layer_name, const int seq = 0, const rclcpp::Time& stamp = rclcpp::Clock(RCL_ROS_TIME).now()){
        cv::Mat img, normalized_img, color_img, mask;
        if(pub.getNumSubscribers()){
            auto& map = *map_ptr_;
            grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map,layer_name,CV_8UC1,img);
            cv::applyColorMap(img, color_img, cv::COLORMAP_JET);

            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "8UC3", color_img).toImageMsg();
            msg->header.stamp = stamp;
            pub.publish(msg);
        }
    }

   private:
    /// subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
    // Only used to synchronize the evaluation node with this one
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sync_sub_;

    /// publisher
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    std::unordered_map<std::string, image_transport::Publisher> layer_pubs_;

    /// pointer to the functionality class
    std::shared_ptr<GroundGrid> groundgrid_;

    /// grid map
    std::shared_ptr<grid_map::GridMap> map_ptr_;

    /// Filter class for grid map
    GroundSegmentation ground_segmentation_;

    /// tf stuff
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;


    float variance_factor_ = 0.35f;
    float normals_factor_ = 0.01f;
    float intensity_factor_ = 0.375f;
    float intensity_shift_ = .075f;
};
}

// includes for dataset file handling
#include <Eigen/Core>
#include <fstream>
#include <filesystem>
#include <vector>
#include <iomanip>
#include <cstdint>
#include <regex>

std::vector<uint32_t> readLabels(const std::string& filename) {
    // Construct full file path
    std::filesystem::path labelPath = std::filesystem::path(filename);
    
    // Open the file
    std::ifstream file(labelPath, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Could not open label file: " + labelPath.string());
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    std::streampos fileSize = file.tellg();
    file.seekg(0, std::ios::beg);
    
    // Read the entire file
    std::vector<uint32_t> labels(fileSize / sizeof(uint32_t));
    file.read(reinterpret_cast<char*>(labels.data()), fileSize);
    
    // Extract semantic labels (lower 16 bits)
    for (auto& label : labels) {
        label &= 0xFFFF;
    }
    
    return labels;
}


// adapted from: https://github.com/yanii/kitti-pcl/blob/master/src/kitti2pcd.cpp
sensor_msgs::msg::PointCloud2::SharedPtr readNextCloud(const groundgrid::DATASET dataset, const std::string& path,
                                                       const groundgrid::LiDAR lidar =
                                                       groundgrid::LiDAR::VELODYNE_64,
                                                       const size_t rings = 64){
    using namespace groundgrid;
    auto start = std::chrono::steady_clock::now();
    sensor_msgs::msg::PointCloud2::SharedPtr cloud(new sensor_msgs::msg::PointCloud2);
    sensor_msgs::msg::PointField x, y, z, intensity, ring;
    x.name = "x";
    x.offset = 0;
    x.datatype=sensor_msgs::msg::PointField::FLOAT32;
    x.count = 1;
    cloud->fields.push_back(x);
    y.name = "y";
    y.offset = 4;
    y.datatype=sensor_msgs::msg::PointField::FLOAT32;
    y.count = 1;
    cloud->fields.push_back(y);
    z.name = "z";
    z.offset = 8;
    z.datatype=sensor_msgs::msg::PointField::FLOAT32;
    z.count = 1;
    cloud->fields.push_back(z);
    intensity.name = "intensity";
    intensity.offset = 12;
    intensity.datatype=sensor_msgs::msg::PointField::FLOAT32;
    intensity.count = 1;
    cloud->fields.push_back(intensity);
    ring.name = "ring";
    ring.offset = 16;
    ring.datatype=sensor_msgs::msg::PointField::UINT16;
    ring.count = 1;
    cloud->fields.push_back(ring);
    cloud->point_step = 18;
    cloud->height = 1; // unorganized

    std::vector<uint32_t> labels;

    if(dataset == KITTI){
        std::string label_path = path;
        label_path = std::regex_replace(label_path, std::regex("velodyne"), "labels");
        label_path = std::regex_replace(label_path, std::regex("bin"), "label");
        labels = readLabels(label_path);
    }
    
    // use pcd
    if(dataset == PCD){
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR> pcl_cloud;
        if (pcl::io::loadPCDFile<velodyne_pointcloud::PointXYZIR>(path, pcl_cloud) == -1) {
            std::cerr << "Could not read file: " << path << std::endl;
        		return cloud;
        }
        pcl::toROSMsg(pcl_cloud, *cloud);
        return cloud;
    }

    std::fstream input(path, std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << path << std::endl;
    		return cloud;
    }


    while(input.good() && !input.eof()){
    		std::vector<char> point(cloud->point_step);
    		input.read((char *) point.data(), 3*sizeof(float)); // Coords 
    		if(lidar != AVIA)
        		input.read((char *) point.data()+intensity.offset, sizeof(float)); //reflectivity/intensity
        if(lidar == AEVA){
     		    input.ignore(sizeof(float) + sizeof(int32_t)); // skip 12 bytes ( velocity, time_ns)
         		input.read((char *) point.data()+ring.offset, sizeof(uint8_t)); //TODO ring is uint16

            if(std::atol(path.substr(path.length()-23, path.length()-4).c_str()) > 1691936557946849179) // this field is missing before this timestamp
         		    input.ignore(sizeof(float)); // skip 12 bytes (reflectivity, velocity, time_ns)
            		float* intensity_val = reinterpret_cast<float*>(point.data() + intensity.offset); // reflectivity to intensity
            		*intensity_val /= 75.0f;
     		}
        else if(lidar == AVIA){
     		    uint8_t intensity_byte;
        		input.read((char *) &intensity_byte, sizeof(uint8_t)); //intensity
        		float* intensity_val = reinterpret_cast<float*>(point.data() + intensity.offset);
        		*intensity_val = intensity_byte;
     		    input.ignore(sizeof(uint8_t)); // skip 1 byte (tag)
         		input.read((char *) point.data()+ring.offset, sizeof(uint8_t)); //TODO ring is uint16
         		input.ignore(sizeof(uint32_t)); // skip 4 bytes (offset_time)
     		}
    		if(dataset == MULRAN){
        		// Normalize MulRan intensity values
        		float* intensity_val = reinterpret_cast<float*>(point.data() + intensity.offset);
        		*intensity_val *= (std::abs(*reinterpret_cast<const float*>(point.data())) +
        		                  std::abs(*reinterpret_cast<const float*>(point.data() + y.offset)) +
        		                  std::abs(*reinterpret_cast<const float*>(point.data() + z.offset)))/5000.0f;
    	  }
        if(dataset != HELIPR){
            // add reconstructed ring field for compatibility
            *reinterpret_cast<unsigned short*>(point.data() + ring.offset) = (rings-1) - cloud->width % rings;
        }
        else if(lidar == OUSTER_128){
    		    input.ignore(sizeof(uint32_t) + sizeof(uint16_t)); // skip 6 bytes (time, reflectivity)
        		input.read((char *) point.data()+ring.offset, sizeof(uint16_t));
    		    input.ignore(sizeof(uint16_t)); // skip 2 bytes (ambient)
    		}
        else if(lidar == VELODYNE_32){
        		input.read((char *) point.data()+ring.offset, sizeof(uint16_t));
            input.ignore(sizeof(float)); // skip time
        }
    		// replace ring field with labels
        if(dataset == KITTI)
            *reinterpret_cast<unsigned short*>(point.data() + ring.offset) = labels[cloud->width];
    		cloud->data.insert(cloud->data.end(), point.begin(), point.end());
    		cloud->width++;
    }
    input.close();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    const int milliseconds = elapsed_seconds.count() * 1000;
    return cloud;
}

// adapted from: https://gist.github.com/LimHyungTae/2211175148867d003f1bc4d8cb483d50
void processPoses(const groundgrid::DATASET dataset, const std::string& poses_filename, const char poses_delimiter,
                  std::vector<geometry_msgs::msg::Pose>& poses_out){
    using namespace groundgrid;
    std::ifstream poses_file;
    poses_file.open(poses_filename);
    Eigen::Matrix4d calibration_matrix = Eigen::Matrix4d::Identity();
    if(dataset == KITTI)
        calibration_matrix << 0.00042768, -0.99996725, -0.00808449, -0.0119846,
                            -0.00721063, 0.0080812, -0.99994132, -0.05403985,
                            0.99997386, 0.00048595, -0.00720693, -0.29219686,
                            0, 0, 0, 1;

    Eigen::Matrix4d calibration_matrix_inv = calibration_matrix.inverse();
    geometry_msgs::msg::Pose lastPose;
    size_t last_frame_id = 0;
    for(std::string line; std::getline(poses_file, line); ){
        std::vector<double> pose;
        std::string field;
        std::stringstream ss(line);
        size_t frame_id = 0;
        if(dataset == KITTI_360){
            // first column is frame number
            std::getline(ss, field, poses_delimiter);
            frame_id = std::atol(field.c_str());
            // KITTI-360 skips poses for standing parts, we add the same pose again to make up for this 
            for(size_t i=1; i<frame_id-last_frame_id; ++i)
                poses_out.push_back(poses_out.back());
            last_frame_id = frame_id;
        }
        while(std::getline(ss, field, poses_delimiter)){
            pose.push_back(std::stod(field));
        }

        geometry_msgs::msg::Pose pose_msg;
        // Assume that the pose matrix is stored in the last 12 fields.
        // If there are more fields <number of fields>-12 will be skipped
        int offset = pose.size() - 12;
        // 8 fields, assume TUM format
        if(pose.size() == 8){
            pose_msg.position.x = pose[1];
            pose_msg.position.y = pose[2];
            pose_msg.position.z = pose[3];
            pose_msg.orientation.x = pose[4];
            pose_msg.orientation.y = pose[5];
            pose_msg.orientation.z = pose[6];
            pose_msg.orientation.w = pose[7];
        }
        else if(offset < 0){
            std::cerr << "Pose file invalid: Expected 12 fields, got " << pose.size() <<
                        "! Is the delimiter correct? Delimiter used: \"" << poses_delimiter << "\"" << std::endl;
            return;
        }
        else{ // KITTI format
            tf2::Matrix3x3 rot_matrix;
            Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
            for (int idx = 0; idx < 12; ++idx){
                int i = idx / 4;
                int j = idx % 4;
                pose_matrix(i, j) = pose[idx + offset];
            }
            // apply calibration matrix only to kitti dataset
            if(dataset == KITTI){
                pose_matrix *= calibration_matrix;
                pose_matrix = calibration_matrix_inv * pose_matrix;
            } 
            for (int idx = 0; idx < 12; ++idx){
                int i = idx / 4;
                int j = idx % 4;
                if(j < 3)
                    rot_matrix[i][j] = pose_matrix(i, j);
                else if(i == 0)
                    pose_msg.position.x = pose_matrix(i, j);
                else if(i == 1)
                    pose_msg.position.y = pose_matrix(i, j);
                else if(i == 2)
                    pose_msg.position.z = pose_matrix(i, j);
            }

            double roll = 0, pitch = 0, yaw = 0;
            rot_matrix.getRPY(roll, pitch, yaw);
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            pose_msg.orientation = tf2::toMsg(quat);
        }

        // for KITTI360 we have to fill the poses until the first frame
        if(dataset == KITTI_360 && poses_out.empty())
            for(int i=0; i<frame_id; ++i)
                poses_out.push_back(pose_msg);
        poses_out.push_back(pose_msg);
    }
}

void processMulRanStamps(const std::string& stamp_filename, const char poses_delimiter, std::vector<std::string>& stamps_out){
    std::ifstream stamp_file;
    stamp_file.open(stamp_filename);
    for(std::string line; std::getline(stamp_file, line); ){
        std::stringstream ss(line);
        std::string stamp;
        std::getline(ss, stamp, poses_delimiter);
        stamps_out.push_back(stamp);
    }
}

std::string str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return s;
}

void processHeliprStamps(const std::string& stamp_filename, const char poses_delimiter, const std::string& sensor_name,
                         std::vector<std::string>& stamps_out){
    std::ifstream stamp_file;
    stamp_file.open(stamp_filename);
    std::string sensor_name_adjusted = str_tolower(sensor_name);

    // The Avia sensor is named "livox_avia" in the stamp.csv file
    if(sensor_name == "Avia")
        sensor_name_adjusted = "livox_avia";

    for(std::string line; std::getline(stamp_file, line); ){
        std::stringstream ss(line);
        std::string stamp, sensor;
        std::getline(ss, stamp, poses_delimiter);
        std::getline(ss, sensor, poses_delimiter);
        if(sensor == sensor_name_adjusted)
            stamps_out.push_back(stamp);
    }
}

std::vector<geometry_msgs::msg::Pose> matchCloudPoses(const std::vector<std::string>& ouster_stamps, const std::vector<std::string>& poses_stamps,
                            const std::vector<geometry_msgs::msg::Pose>& poses){
    std::vector<geometry_msgs::msg::Pose> result;
    size_t last_stamp = 0; // start at 0
    size_t pose_idx = 0;
    for(const auto& stamp : ouster_stamps){
        size_t stamp_long = std::stol(stamp);
        size_t diff_to_last = stamp_long - last_stamp;
        while(pose_idx < poses_stamps.size()-1 && stamp_long > std::stol(poses_stamps[pose_idx]))
            ++pose_idx;
        size_t diff_to_next = std::stol(poses_stamps[pose_idx]) - stamp_long;

        if(diff_to_next < diff_to_last){
            result.push_back(poses[pose_idx]);
            last_stamp = std::stol(poses_stamps[pose_idx]);
        }
        else{
            result.push_back(result.back());
        }
    }
    return result;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  auto node = std::make_shared<groundgrid::GroundGridNode>();
  executor->add_node(node);
  using namespace groundgrid;
  LiDAR lidar = VELODYNE_64;

  // live mode, just run as ros2 node without playing a dataset
  // base_link -> odom transform has to be available
  if(node->dataset_name == "live"){
      std::cout << "Using ros node live mode" << std::endl;
      if(node->sensor == "Aeva")
          lidar = AEVA;
      else if(node->sensor == "Avia")
          lidar = AVIA;
      else if(node->sensor == "Ouster64")
          lidar = OUSTER_64;
      else if(node->sensor == "Ouster")
          lidar = OUSTER_128;
      else if(node->sensor == "Velodyne")
          lidar = VELODYNE_64;
      else{
          std::cerr << "Could not determine lidar model " << node->sensor<< " -> defaulting to Ouster 128!" << std::endl;
          lidar = OUSTER_128;
      }
      node->setLiDAR(lidar);
      executor->spin();
      rclcpp::shutdown();
      return 0;
  }

  DATASET dataset = KITTI;

  if(node->dataset_name == "kitti")
      dataset = KITTI;
  else if(node->dataset_name == "kitti360")
      dataset = KITTI_360;
  else if(node->dataset_name == "mulran")
      dataset = MULRAN;
  else if(node->dataset_name == "helipr")
      dataset = HELIPR;
  else if(node->dataset_name == "pcd")
      dataset = PCD;
  else
      std::cerr << "Could not determine dataset name " << node->dataset_name << " defaulting to kitti!" << std::endl;
  
  std::ostringstream ss;
  if(dataset == KITTI)
      ss << node->dataset_path << "/sequences/" << node->sequence << "/velodyne/";
  else if(dataset == KITTI_360){
      ss << node->dataset_path << "/data_3d_raw/2013_05_28_drive_" << std::setw(4) << std::setfill('0') << node->sequence 
          << "_sync/velodyne_points/data/";
  }
  else if(dataset == MULRAN){
      ss << node->dataset_path << "/" << node->sequence << "/Ouster/";
      lidar = OUSTER_64;
  }
  else if(dataset == HELIPR){
      ss << node->dataset_path << "/" << node->sequence << "/LiDAR/" << node->sensor;
      // for helipr we take the lidar model from the sequence
      if(node->sensor == "Aeva")
          lidar = AEVA;
      else if(node->sensor == "Avia")
          lidar = AVIA;
      else if(node->sensor == "Ouster")
          lidar = OUSTER_128;
      else if(node->sensor == "Velodyne")
          lidar = VELODYNE_32;
      else{
          std::cerr << "Could not determine lidar model " << node->sensor<< " -> defaulting to Ouster!" << std::endl;
          lidar = OUSTER_128;
      }
  }
  else if(dataset == PCD){
      ss << node->dataset_path << "/" << node->sequence;
      lidar = VELODYNE_128;
  }

  node->setLiDAR(lidar);
  std::string path = ss.str();
  std::vector<geometry_msgs::msg::Pose> poses;
  std::cout << "Selected dataset type: " << node->dataset_name << std::endl;
  std::cout << "Using point clouds from: " << path << std::endl;
  std::cout << "Using poses from: " << node->poses_path << std::endl;

  // read the stamps file for the MulRan dataset
  std::vector<std::string> ouster_stamps, pose_stamps;
  if(dataset == MULRAN){
      // The MulRan ground truth poses come with timestamps, which are processed here
      if(dataset == MULRAN){
          std::cout << "Processing stamp file: " << node->poses_path << std::endl;
          processMulRanStamps(node->poses_path, node->poses_delimiter[0], pose_stamps);
          std::cout << "Processed " << pose_stamps.size() << " stamps" << std::endl;
      }

      std::ostringstream mulran_stamp_file;
      mulran_stamp_file << node->dataset_path << "/" << node->sequence << "/" << "ouster_front_stamp.csv";
      std::cout << "Processing stamp file: " << mulran_stamp_file.str() << std::endl;
      processMulRanStamps(mulran_stamp_file.str(), node->poses_delimiter[0], ouster_stamps);
      std::cout << "Processed " << ouster_stamps.size() << " stamps" << std::endl;
  }
  else if(dataset == HELIPR){
      std::ostringstream helipr_stamp_file;
      helipr_stamp_file << node->dataset_path << "/" << node->sequence << "/" << "stamp.csv";
      std::cout << "Processing stamp file: " << helipr_stamp_file.str() << std::endl;
      processHeliprStamps(helipr_stamp_file.str(), ',', node->sensor, ouster_stamps);
      std::cout << "Processed " << ouster_stamps.size() << " stamps" << std::endl;
  }
  else if(dataset == PCD){ // Stamps are in the poses file
      std::ostringstream helipr_stamp_file;
      helipr_stamp_file << node->dataset_path << "/" << "stamp.csv";
      std::cout << "Processing stamp file: " << helipr_stamp_file.str() << std::endl;
      processHeliprStamps(helipr_stamp_file.str(), ',', node->sensor, ouster_stamps);
      std::cout << "Processed " << ouster_stamps.size() << " stamps" << std::endl;
  }
  processPoses(dataset, node->poses_path, node->poses_delimiter[0], poses);
  
  if(dataset == MULRAN){// The MulRan ground truth poses must be matched with the corresponding clouds
      poses = matchCloudPoses(ouster_stamps, pose_stamps, poses); // ensure one pose per cloud
  }

  // read first cloud asynchronosly
  std::ostringstream cloud_file_name;
  if(dataset == KITTI)
        cloud_file_name << path << std::setw(6) << std::setfill('0') << 0 << ".bin";
  else if(dataset == KITTI_360)
        cloud_file_name << path << std::setw(10) << std::setfill('0') << 0 << ".bin";
  else if(dataset == MULRAN || dataset == HELIPR)
       cloud_file_name << path << "/" << ouster_stamps[0] << ".bin";
  else if(dataset == PCD)
       cloud_file_name << path << "/" << ouster_stamps[0] << ".pcd";
       
  auto cloud_msg_future = std::async(std::launch::async, &readNextCloud, dataset, cloud_file_name.str(), lidar, 128);

  // wait for initialization of other nodes
  while(node->current_cloud < 0 && node->eval_) 
      executor->spin_some();

  static float avg_frequency = 0;
  static float avg_time = 0;
  for(size_t i=0; i<poses.size(); ++i){
      auto start = std::chrono::steady_clock::now();
      executor->spin_some();
      const auto current_time = node->now();

      // Construct and set transforms
      nav_msgs::msg::Odometry odom;
      static nav_msgs::msg::Odometry lastOdom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      const double x_last = lastOdom.pose.pose.position.x;
      const double y_last = lastOdom.pose.pose.position.y;
      odom.pose.pose = poses[i];
      lastOdom = odom;
      geometry_msgs::msg::TransformStamped t, t_map_odom;
      const double odomTest_x = odom.pose.pose.position.x;
      const double odomTest_y = odom.pose.pose.position.y;
      t_map_odom.header.stamp = current_time;
      t_map_odom.header.frame_id = "map";
      t_map_odom.child_frame_id = "odom";
      t_map_odom.transform.rotation.w = 1.0;
      t.header.stamp = current_time;
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";
      t.transform.translation.x = odom.pose.pose.position.x;
      t.transform.translation.y = odom.pose.pose.position.y;
      t.transform.translation.z = odom.pose.pose.position.z;
      t.transform.rotation = odom.pose.pose.orientation;
      node->setTransform(t);

      // execute odom callback
      node->odom_callback(std::make_shared<nav_msgs::msg::Odometry>(odom));


      std::ostringstream ss;
      sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
      cloud_msg_future.wait();
      cloud_msg = cloud_msg_future.get();
      cloud_msg->header.stamp = current_time;
      cloud_msg->header.frame_id = "velodyne";

      // asynchronosly read the next cloud 
      if(i+1 < poses.size()){
        if(dataset == KITTI)
              ss << path << std::setw(6) << std::setfill('0') << i+1 << ".bin";
        else if(dataset == KITTI_360)
              ss << path << std::setw(10) << std::setfill('0') << i+1 << ".bin";
        else if(dataset == MULRAN || dataset == HELIPR)
             ss << path << "/" << ouster_stamps[i+1] << ".bin";
        else if(dataset == PCD)
             ss << path << "/" << ouster_stamps[i+1] << ".pcd";
        cloud_msg_future = std::async(std::launch::async, &readNextCloud, dataset, ss.str(), lidar, 128);
      }

      // execute cloud callback
      auto gg_start = std::chrono::steady_clock::now();
      node->points_callback(cloud_msg);
      auto gg_end = std::chrono::steady_clock::now();
      std::chrono::duration<double> gg_elapsed_seconds = gg_end-gg_start;
      const int gg_millis = gg_elapsed_seconds.count() * 1000;
      node->tf_broadcaster_->sendTransform(t); // send last transform, so RViz can display the point cloud

      if(i == 0)
          continue;
      // wait for the evaluation results
      while(node->current_cloud <= i && node->eval_) // wait until the sent clouds are processed
          executor->spin_some();

      odom.pose.pose = poses[i-1];

      auto end = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_seconds = end-start;
      const int milliseconds = elapsed_seconds.count() * 1000;
      avg_frequency += (1000.0f/gg_millis - avg_frequency) / (i+1.0f); 
      avg_time += (gg_millis - avg_time) / (i+1.0f); 
      std::cout << "\x1b[1F \x1b[2K\033[1;32mCloud " << i << " of " << poses.size();
      std::cout << std::setprecision(2) << std::fixed << ", GroundGrid mean runtime: " << avg_time << " ms (freq. " << 1000.0f/avg_time << " Hz)" << std::setprecision(6) << "\033[0m" << std::endl;

      if(node->frame_rate_cap_ > 0 && 1000.0/node->frame_rate_cap_ - milliseconds > 0){
          std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(std::round(1000.0/node->frame_rate_cap_ - milliseconds))));
          // set avg_frequency to our frame rate cap
          avg_frequency = std::min(avg_frequency, node->frame_rate_cap_);
      }
  }

  // Wait for the processing of the last cloud 
  while(node->current_cloud < poses.size() && node->eval_)
      executor->spin_some();

  std::cout << "\x1b[1F \x1b[2K\033[1;32mCloud " << poses.size() << " of " << poses.size();
      std::cout << std::setprecision(2) << std::fixed << ", GroundGrid mean runtime: " << avg_time << " ms (freq. " << 1000.0f/avg_time << " Hz)" << std::setprecision(6) << "\033[0m" << std::endl;
  
  rclcpp::shutdown();
  return 0;
}
