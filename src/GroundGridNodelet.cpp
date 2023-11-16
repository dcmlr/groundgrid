/*
Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

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
#include <numeric>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

// ros msgs
#include <nav_msgs/Odometry.h>

// Pcl
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>

// ros opencv transport
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// ros tf
#include <tf2_ros/transform_listener.h>

// grid map
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <dynamic_reconfigure/server.h>
#include <groundgrid/GroundGrid.h>
#include <groundgrid/GroundGridConfig.h>
#include <groundgrid/GroundGridFwd.h>
#include <groundgrid/GroundSegmentation.h>

namespace groundgrid {


class GroundGridNodelet : public nodelet::Nodelet {
   public:
    typedef velodyne_pointcloud::PointXYZIR PCLPoint;

    /** Constructor.
     */
    GroundGridNodelet() : mTfListener(mTfBuffer){ }

    /** Destructor.
     */
    virtual ~GroundGridNodelet() {
    }

    /** Nodelet initialization. Called by nodelet manager on initialization,
     */
    virtual void onInit() override {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        image_transport::ImageTransport it(nh);
        grid_map_cv_img_pub_ = it.advertise("groundgrid/grid_map_cv", 1);
        terrain_im_pub_ = it.advertise("groundgrid/terrain", 1);
        grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>("groundgrid/grid_map", 1);
        filtered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("groundgrid/segmented_cloud", 1);


        groundgrid_ = std::make_shared<GroundGrid>();

        config_server_ = boost::make_shared<dynamic_reconfigure::Server<groundgrid::GroundGridConfig> >(pnh);
        dynamic_reconfigure::Server<groundgrid::GroundGridConfig>::CallbackType f;
        f = boost::bind(&GroundGridNodelet::callbackReconfigure, this, _1, _2);

        ground_segmentation_.init(nh, groundgrid_->mDimension, groundgrid_->mResolution);

        config_server_->setCallback(f);

        // ego-position
        pos_sub_ = nh.subscribe( "/localization/odometry/filtered_map", 1, &groundgrid::GroundGridNodelet::odom_callback, this);

        // input point cloud
        points_sub_ = nh.subscribe("/sensors/velodyne_points", 1, &groundgrid::GroundGridNodelet::points_callback, this);
   }

   protected:
    virtual void odom_callback(const nav_msgs::OdometryConstPtr& inOdom){
        static nav_msgs::OdometryConstPtr lastOdom;
        if(!lastOdom || std::hypot(lastOdom->pose.pose.position.x-inOdom->pose.pose.position.x, 2.0f) + std::hypot(lastOdom->pose.pose.position.y-inOdom->pose.pose.position.y, 2.0f) >= 1.0){
            auto start = std::chrono::steady_clock::now();
            map_ptr_ = groundgrid_->update(inOdom);
            auto end = std::chrono::steady_clock::now();
            ROS_DEBUG_STREAM("grid map update took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
        }
    }

    virtual void points_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
        auto start = std::chrono::steady_clock::now();
        static size_t time_vals = 0;
        static double avg_time = 0.0;
        static double avg_cpu_time = 0.0;
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
        pcl::fromROSMsg (*cloud_msg, *cloud);
        geometry_msgs::TransformStamped mapToBaseTransform, cloudOriginTransform;

        // Map not initialized yet, this means the node hasn't received any odom message so far.
        if(!map_ptr_)
            return;

        try{
            //mTfBuffer.canTransform("base_link", "map", cloud_msg->header.stamp, ros::Duration(0.0));
            mapToBaseTransform = mTfBuffer.lookupTransform("map", "base_link", cloud_msg->header.stamp, ros::Duration(0.0));
            //mTfBuffer.canTransform(cloud_msg->header.frame_id, "map", cloud_msg->header.stamp, ros::Duration(0.0));
            cloudOriginTransform = mTfBuffer.lookupTransform("map", "velodyne", cloud_msg->header.stamp, ros::Duration(0.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Received point cloud but transforms are not available: %s",ex.what());
            return;
        }


        geometry_msgs::PointStamped origin;
        origin.header = cloud_msg->header;
        origin.header.frame_id = "velodyne";
        origin.point.x = 0.0f;
        origin.point.y = 0.0f;
        origin.point.z = 0.0f;

        tf2::doTransform(origin, origin, cloudOriginTransform);

        // Transform cloud into map coordinate system
        if(cloud_msg->header.frame_id != "map"){
            // Transform to map
            geometry_msgs::TransformStamped transformStamped;
            pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr transformed_cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
            transformed_cloud->header = cloud->header;
            transformed_cloud->header.frame_id = "map";
            transformed_cloud->points.reserve(cloud->points.size());

            try{
                mTfBuffer.canTransform("map", cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.0));
                transformStamped = mTfBuffer.lookupTransform("map", cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("Failed to get map transform for point cloud transformation: %s",ex.what());
                return;
            }

            geometry_msgs::PointStamped psIn;
            psIn.header = cloud_msg->header;
            psIn.header.frame_id = "map";

            for(const auto& point : cloud->points){
                psIn.point.x = point.x;
                psIn.point.y = point.y;
                psIn.point.z = point.z;

                tf2::doTransform(psIn, psIn, transformStamped);

                PCLPoint& point_transformed = transformed_cloud->points.emplace_back(point);
                point_transformed.x = psIn.point.x;
                point_transformed.y = psIn.point.y;
                point_transformed.z = psIn.point.z;
            }

            cloud = transformed_cloud;
        }

        auto end = std::chrono::steady_clock::now();
        ROS_DEBUG_STREAM("cloud transformation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");

        auto start2 = std::chrono::steady_clock::now();
        std::clock_t c_clock = std::clock();
        sensor_msgs::PointCloud2 cloud_msg_out;
        PCLPoint origin_pclPoint;
        origin_pclPoint.x = origin.point.x;
        origin_pclPoint.y = origin.point.y;
        origin_pclPoint.z = origin.point.z;
        pcl::toROSMsg(*(ground_segmentation_.filter_cloud(cloud, origin_pclPoint, mapToBaseTransform, *map_ptr_)), cloud_msg_out);

        cloud_msg_out.header = cloud_msg->header;
        cloud_msg_out.header.frame_id = "map";
        filtered_cloud_pub_.publish(cloud_msg_out);
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start2;
        const double milliseconds = elapsed_seconds.count() * 1000;
        const double c_millis = double(std::clock() - c_clock)/CLOCKS_PER_SEC * 1000;
        avg_time = (milliseconds + time_vals * avg_time)/(time_vals+1);
        avg_cpu_time = (c_millis + time_vals * avg_cpu_time)/(time_vals+1);
        ++time_vals;
        ROS_INFO_STREAM("groundgrid took " << milliseconds << "ms (avg: " << avg_time << "ms)");
        ROS_DEBUG_STREAM("total cpu time used: " << c_millis << "ms (avg: " << avg_cpu_time << "ms)") ;

        grid_map_msgs::GridMap grid_map_msg;
        grid_map::GridMapRosConverter::toMessage(*map_ptr_, grid_map_msg);
        grid_map_msg.info.header.stamp = cloud_msg->header.stamp;
        grid_map_pub_.publish(grid_map_msg);

        const ros::NodeHandle& nh = getNodeHandle();
        image_transport::ImageTransport it(nh);

        for(const auto& layer : map_ptr_->getLayers()){
            if(layer_pubs_.find(layer) == layer_pubs_.end()){
                layer_pubs_[layer] = it.advertise("/groundgrid/grid_map_cv_"+layer, 1);
            }
            publish_grid_map_layer(layer_pubs_.at(layer), layer, cloud_msg->header.seq, cloud_msg->header.stamp);
        }

        if(terrain_im_pub_.getNumSubscribers()){
            publish_grid_map_layer(terrain_im_pub_, "terrain", cloud_msg->header.seq, cloud_msg->header.stamp);
        }

        end = std::chrono::steady_clock::now();
        ROS_DEBUG_STREAM("overall " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");
    }

    void publish_grid_map_layer(const image_transport::Publisher& pub, const std::string& layer_name, const int seq = 0, const ros::Time& stamp = ros::Time::now()){
        cv::Mat img, normalized_img, color_img, mask;

        if(pub.getNumSubscribers()){
            if(layer_name != "terrain"){
                const auto& map = *map_ptr_;
                grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map,layer_name,CV_8UC1,img);
                cv::applyColorMap(img, color_img, cv::COLORMAP_TWILIGHT);

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", color_img).toImageMsg();
                msg->header.stamp = stamp;
                pub.publish(msg);
            }
            else{ // special treatment for the terrain evaluation
                const auto& map = *map_ptr_;
                img = cv::Mat(map.getSize()(0), map.getSize()(1), CV_32FC3, cv::Scalar(0,0,0));
                normalized_img = cv::Mat(map.getSize()(0), map.getSize()(1), CV_32FC3, cv::Scalar(0,0,0));
                const grid_map::Matrix& data = map["ground"];
                const grid_map::Matrix& visited_layer = map["pointsRaw"];
		const grid_map::Matrix& gp_layer = map["groundpatch"];
                const float& car_height = data(181,181);
                const float& ground_min =  map["ground"].minCoeff() - car_height;
                const float& ground_max =  map["ground"].maxCoeff() - car_height;
                if(ground_max == ground_min)
                    return;

                for(grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
                    const grid_map::Index index(*iterator);
                    const float& value = data(index(0), index(1));// - car_height;
                    const float& gp = gp_layer(index(0), index(1));
                    const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
                    const float& pointssum = visited_layer.block<3,3>(index(0)-1,index(1)-1).sum();
                    const float& pointcount = visited_layer(index(0), index(1));

                    //img.at<cv::Point3f>(imageIndex(0), imageIndex(1)) = cv::Point3f(value, / *pointcount+1 * /pointcount >= 3 ? 1.0f : 0.0f, gp > .0 ? gp : 0.0f);
                    img.at<cv::Point3f>(imageIndex(0), imageIndex(1)) = cv::Point3f(value, pointssum >= 27 ? 1.0f : 0.0f, pointcount);
                }

                geometry_msgs::TransformStamped baseToUtmTransform;

                try{
                    baseToUtmTransform = mTfBuffer.lookupTransform("utm", "base_link", stamp, ros::Duration(0.0));
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                    return;
                }
                geometry_msgs::PointStamped ps;
                ps.header.frame_id = "base_link";
                ps.header.stamp = stamp;
                tf2::doTransform(ps, ps, baseToUtmTransform);

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "32FC3", img).toImageMsg();
                msg->header.frame_id = std::to_string(seq) + "_" + std::to_string(ps.point.x) + "_" + std::to_string(ps.point.y);
                pub.publish(msg);
            }
        }
    }

   private:

    /** Callback for dynamic_reconfigure.
     **
     ** @param msg
     */
    void callbackReconfigure(groundgrid::GroundGridConfig & config, uint32_t level) {
        groundgrid_->setConfig(config);
        ground_segmentation_.setConfig(config);
    }

    /// subscriber
    ros::Subscriber points_sub_, pos_sub_;

    /// publisher
    image_transport::Publisher grid_map_cv_img_pub_, terrain_im_pub_;
    ros::Publisher grid_map_pub_, filtered_cloud_pub_;
    std::unordered_map<std::string, image_transport::Publisher> layer_pubs_;

    /// pointer to dynamic reconfigure service
    boost::shared_ptr<dynamic_reconfigure::Server<groundgrid::GroundGridConfig> > config_server_;

    /// pointer to the functionality class
    GroundGridPtr groundgrid_;

    /// grid map
    std::shared_ptr<grid_map::GridMap> map_ptr_;

    /// Filter class for grid map
    GroundSegmentation ground_segmentation_;

    /// tf stuff
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(groundgrid::GroundGridNodelet, nodelet::Nodelet);
// nodelet_plugins.xml refers to the parameters as "type" and "base_class_type"
