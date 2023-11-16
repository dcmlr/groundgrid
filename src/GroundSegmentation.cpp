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

#include <groundgrid/GroundSegmentation.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <thread>

using namespace groundgrid;


void GroundSegmentation::init(ros::NodeHandle& nodeHandle, const size_t dimension, const float& resolution){
    const size_t cellCount = std::round(dimension/resolution);

    expectedPoints.resize(cellCount, cellCount);
    for(size_t i=0; i<cellCount; ++i){
        for(size_t j=0; j<cellCount; ++j){
            const float& dist = std::hypot(i-cellCount/2.0,j-cellCount/2.0);
            expectedPoints(i,j) = std::atan(1/dist)/verticalPointAngDist;
        }
    }
    Eigen::initParallel();
}

pcl::PointCloud<GroundSegmentation::PCLPoint>::Ptr GroundSegmentation::filter_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const PCLPoint& cloudOrigin, const geometry_msgs::TransformStamped& mapToBase, grid_map::GridMap &map)
{
    auto start = std::chrono::steady_clock::now();
    static double avg_insertion_time = 0.0;
    static double avg_detection_time = 0.0;
    static double avg_segmentation_time = 0.0;
    static unsigned int time_vals = 0;

    pcl::PointCloud<PCLPoint>::Ptr filtered_cloud (new pcl::PointCloud<PCLPoint>);
    filtered_cloud->points.reserve(cloud->points.size());

    map.add("groundCandidates", 0.0);
    map.add("planeDist", 0.0);
    map.add("m2", 0.0);
    map.add("meanVariance", 0.0);

    // raw point count layer for the evaluation
    map.add("pointsRaw", 0.0);


    map["groundCandidates"].setZero();
    map["points"].setZero();
    map["minGroundHeight"].setConstant(std::numeric_limits<float>::max());
    map["maxGroundHeight"].setConstant(std::numeric_limits<float>::min());

    map.add("variance", 0.0);
    static const grid_map::Matrix& ggv = map["variance"];
    static grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggl = map["ground"];
    const auto& size = map.getSize();
    const size_t threadcount = mConfig.thread_count;


    std::vector<std::pair<size_t, grid_map::Index> > point_index;
    point_index.reserve(cloud->points.size());
    std::vector<std::vector<std::pair<size_t, grid_map::Index> > > point_index_list;
    point_index_list.resize(threadcount);

    // Collect all outliers for the outlier detection evaluation
    std::vector<size_t> outliers;
    std::vector<std::vector<size_t> > outliers_list;
    outliers_list.resize(threadcount);

    // store ignored points to re-add them afterwards
    std::vector<std::pair<size_t, grid_map::Index> > ignored;
    std::vector<std::vector<std::pair<size_t, grid_map::Index> > > ignored_list;
    ignored_list.resize(threadcount);

    // Divide the point cloud into threadcount sections for threaded calculations
    std::vector<std::thread> threads;

    for(size_t i=0; i<threadcount; ++i){
        const size_t start = std::floor((i*cloud->points.size())/threadcount);
        const size_t end = std::ceil(((i+1)*cloud->points.size())/threadcount);
        threads.push_back(std::thread(&GroundSegmentation::insert_cloud, this, cloud, start, end, std::cref(cloudOrigin), std::ref(point_index_list[i]), std::ref(ignored_list[i]),
                                      std::ref(outliers_list[i]), std::ref(map)));
    }

    // wait for results
    std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));

    // join results
    for(const auto& point_index_part : point_index_list)
        point_index.insert(point_index.end(), point_index_part.begin(), point_index_part.end());
    for(const auto& outlier_index_part : outliers_list)
        outliers.insert(outliers.end(), outlier_index_part.begin(), outlier_index_part.end());
    for(const auto& ignored_part : ignored_list)
        ignored.insert(ignored.end(), ignored_part.begin(), ignored_part.end());


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    const double milliseconds = elapsed_seconds.count() * 1000;
    avg_insertion_time = (milliseconds + time_vals * avg_insertion_time)/(time_vals+1);
    ROS_DEBUG_STREAM("ground point rasterization took " << milliseconds << "ms (avg " << avg_insertion_time << " ms)");

    start = std::chrono::steady_clock::now();

    // Divide the grid map into four section for threaded calculations
    threads.clear();
    for(unsigned short section=0; section<4; ++section)
        threads.push_back(std::thread(&GroundSegmentation::detect_ground_patches, this, std::ref(map), section));

    // wait for results
    std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    avg_detection_time = (elapsed_seconds.count() * 1000 + time_vals * avg_detection_time)/(time_vals+1);
    ROS_DEBUG_STREAM("ground patch detection took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms (avg " << avg_detection_time << " ms)");
    ++time_vals;

    start = std::chrono::steady_clock::now();
    spiral_ground_interpolation(map, mapToBase);
    end = std::chrono::steady_clock::now();
    ROS_DEBUG_STREAM("ground interpolation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");

    start = std::chrono::steady_clock::now();
    map["points"].setConstant(0.0);

    // Re-add ignored points
    point_index.insert(point_index.end(), ignored.begin(), ignored.end());


    // Debugging statistics
    const double& min_dist_fac = mConfig.minimum_distance_factor*5;
    const double& min_point_height_thres = mConfig.miminum_point_height_threshold;
    const double& min_point_height_obs_thres = mConfig.minimum_point_height_obstacle_threshold;

    for(const std::pair<size_t, grid_map::Index>& entry : point_index)
    {
        const PCLPoint& point = cloud->points[entry.first];
        const grid_map::Index& gi = entry.second;
        const double& groundheight = ggl(gi(0),gi(1));


        const bool ground_point = point.ring == 40 || point.ring == 44 || point.ring == 48 || point.ring == 49 || point.ring == 60 || point.ring == 72;
        // copy the points intensity because it get's overwritten for evaluation purposes
        const float& variance = ggv(gi(0),gi(1));

        if(size(0) <= gi(0)+3 || size(1) <= gi(1)+3)
            continue;

        const float dist = std::hypot(point.x-cloudOrigin.x, point.y-cloudOrigin.y);
        const double tolerance = std::max(std::min((min_dist_fac*dist)/variance * min_point_height_thres, min_point_height_thres), min_point_height_obs_thres);

        if(tolerance+groundheight < point.z){ // non-ground points
            PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point);
            segmented_point.intensity = 99;
            gpl(gi(0),gi(1)) += 1.0f;
        }
        else{
            PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point); // ground point
            segmented_point.intensity = 49;
        }
    }

    // Re-add outliers to cloud
   for(size_t i : outliers){
        const PCLPoint& point = cloud->points[i];
        const bool ground_point = point.ring == 40 || point.ring == 44 || point.ring == 48 || point.ring == 49 || point.ring == 60 || point.ring == 72;  
        PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point); //ground point
        segmented_point.intensity = 49;
    }

    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    avg_segmentation_time = (elapsed_seconds.count() * 1000 + (time_vals-1) * avg_segmentation_time)/time_vals;
    ROS_DEBUG_STREAM("point cloud segmentation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms (avg " << avg_segmentation_time << " ms)");

    return filtered_cloud;
}


void GroundSegmentation::insert_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const size_t start, const size_t end, const PCLPoint& cloudOrigin, std::vector<std::pair<size_t, grid_map::Index> >& point_index,
                                      std::vector<std::pair<size_t, grid_map::Index> >& ignored, std::vector<size_t>& outliers, grid_map::GridMap &map)
{
    static const grid_map::Matrix& ggp = map["groundpatch"];

    static grid_map::Matrix& gpr = map["pointsRaw"];
    static grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& gmg = map["groundCandidates"];
    static grid_map::Matrix& gmm = map["meanVariance"];
    static grid_map::Matrix& gmx = map["maxGroundHeight"];
    static grid_map::Matrix& gmi = map["minGroundHeight"];
    static grid_map::Matrix& gmd = map["planeDist"];
    static grid_map::Matrix& gm2 = map["m2"];

    const auto& size = map.getSize();

    point_index.reserve(end-start);

    for(size_t i = start; i < end; ++i)
    {
        const PCLPoint& point = cloud->points[i];
        const auto& pos = grid_map::Position(point.x,point.y);
        const float sqdist = std::pow(point.x-cloudOrigin.x, 2.0) + std::pow(point.y-cloudOrigin.y, 2.0);

        bool toSkip=false;

        grid_map::Index gi;
        map.getIndex(pos, gi);

        if(!map.isInside(pos))
            continue;

        // point count map used for evaluation
        gpr(gi(0), gi(1)) += 1.0f;


        if(point.ring > mConfig.max_ring || sqdist < minDistSquared){
            ignored.push_back(std::make_pair(i, gi));
            continue;
        }

        // Outlier detection test
        const float oldgroundheight = ggl(gi(0), gi(1));
        if(point.z < oldgroundheight-0.2){

            // get direction
            PCLPoint vec;
            vec.x = point.x - cloudOrigin.x;
            vec.y = point.y - cloudOrigin.y;
            vec.z = point.z - cloudOrigin.z;

            float len = std::sqrt(std::pow(vec.x, 2.0f) + std::pow(vec.y, 2.0f) + std::pow(vec.z, 2.0f));
            vec.x /= len;
            vec.y /= len;
            vec.z /= len;

            // check for occlusion
            for(int step=3; (std::pow(step*vec.x, 2.0) + std::pow(step*vec.y, 2.0) + std::pow(step*vec.z,2.0)) < std::pow(len,2.0) && vec.z < -0.01f; ++step){
                grid_map::Index intersection, pointPosIndex;
                grid_map::Position intersecPos(step*(vec.x)+cloudOrigin.x, step*(vec.y)+cloudOrigin.y);
                map.getIndex(intersecPos, intersection);

                // Check if inside map borders
                if(intersection(0) <= 0 || intersection(1) <= 0 || intersection(0) >= size(0)-1 || intersection(1) >= size(1)-1)
                    continue;

                // check if known ground occludes the line of sight
                const auto& block = ggp.block<3,3>(std::max(intersection(0)-1, 2), std::max(intersection(1)-1,2));
                if(block.sum() > mConfig.min_outlier_detection_ground_confidence && ggp(intersection(0),intersection(1)) > 0.01f && ggl(intersection(0),intersection(1)) >= step*vec.z+cloudOrigin.z+mConfig.outlier_tolerance){
                    outliers.push_back(i);
                    toSkip=true;
                    break;
                }
            }
        }


        if(toSkip)
            continue;


        float &groundheight = gmg(gi(0),gi(1));
        float &mean = gmm(gi(0), gi(1));


        float planeDist = 0.0;
        point_index.push_back(std::make_pair(i, gi));

        float &points = gpl(gi(0), gi(1));
        float &maxHeight = gmx(gi(0),gi(1));
        float &minHeight = gmi(gi(0),gi(1));
        float &planeDistMap = gmd(gi(0),gi(1));
        float &m2 = gm2(gi(0),gi(1));

        planeDist = point.z - cloudOrigin.z;
        groundheight = (point.z + points * groundheight)/(points+1.0);

        if(mean == 0.0)
            mean = planeDist;
        if(!std::isnan(planeDist)){
            float delta = planeDist - mean;
            mean += delta/(points+1);
            planeDistMap = (planeDist + points * planeDistMap)/(points+1.0);
            m2 += delta * (planeDist - mean);
        }

        maxHeight = std::max(maxHeight, point.z);
        minHeight =std::min(minHeight, point.z-0.0001f); // to make sure maxHeight > minHeight
        points += 1.0;
    }
}


void GroundSegmentation::detect_ground_patches(grid_map::GridMap &map, unsigned short section) const
{
    const grid_map::Matrix& gcl = map["groundCandidates"];
    const static auto& size = map.getSize();
    const static float resolution = map.getResolution();
    static const grid_map::Matrix& gm2 = map["m2"];
    static const grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggv = map["variance"];
    // calculate variance
    ggv = gm2.array().cwiseQuotient(gpl.array()+std::numeric_limits<float>::min());

    int cols_start = 2 + section%2 * (gcl.cols()/2-2);
    int rows_start = section>=2 ? gcl.rows()/2 : 2;
    int cols_end = (gcl.cols())/2 + section%2 * (gcl.cols()/2-2);
    int rows_end = section>=2 ? gcl.rows()-2 : (gcl.rows())/2;

    for(int i=cols_start; i<cols_end; ++i){
        for(int j=rows_start; j<rows_end; ++j){
            const float sqdist = (std::pow(i-(size(0)/2.0),2.0) + std::pow(j-(size(1)/2.0), 2.0)) * std::pow(resolution,2.0);

            if(sqdist <= std::pow(mConfig.patch_size_change_distance, 2.0))
                detect_ground_patch<3>(map, i, j);
            else
                detect_ground_patch<5>(map, i, j);
        }
    }
}


template <int S> void GroundSegmentation::detect_ground_patch(grid_map::GridMap& map, size_t i, size_t j) const
{
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& ggp = map["groundpatch"];
    static grid_map::Matrix& ggv = map["variance"];
    static const grid_map::Matrix& gmi = map["minGroundHeight"];
    static const grid_map::Matrix& gpl = map["points"];
    static const auto& size = map.getSize();
    static const float resolution = map.getResolution();
    const int center_idx = std::floor(S/2);


    const auto& pointsBlock = gpl.block<S,S>(i-center_idx,j-center_idx);
    const float sqdist = (std::pow(i-(size(0)/2.0),2.0) + std::pow(j-(size(1)/2.0), 2.0)) * std::pow(resolution,2.0);
    const int patchSize = S;
    const float& expectedPointCountperLaserperCell = expectedPoints(i,j);
    const float& pointsblockSum = pointsBlock.sum();
    float& oldConfidence = ggp(i,j);
    float& oldGroundheight = ggl(i,j);

    // early skipping of (almost) empty areas
    if(pointsblockSum < std::max(std::floor(mConfig.ground_patch_detection_minimum_point_count_threshold * patchSize * expectedPointCountperLaserperCell), 3.0))
        return;

    // calculation of variance threshold
    // limit the value to the defined minimum and 10 times the defined minimum
    const float varThresholdsq = std::min(std::max(sqdist * std::pow(mConfig.distance_factor,2.0), std::pow(mConfig.minimum_distance_factor,2.0)), std::pow(mConfig.minimum_distance_factor*10, 2.0));
    const auto& varblock = ggv.block<S,S>(i-center_idx,j-center_idx);
    const auto& minblock = gmi.block<S,S>(i-center_idx, j-center_idx);
    const float& variance = varblock(center_idx,center_idx);
    const float& localmin = minblock.minCoeff();
    const float maxVar = pointsBlock(center_idx,center_idx) >= mConfig.point_count_cell_variance_threshold ? variance : pointsBlock.array().cwiseProduct(varblock.array()).sum()/pointsblockSum;
    const float groundlevel = pointsBlock.cwiseProduct(minblock).sum()/pointsblockSum;
    const float groundDiff = std::max((groundlevel - oldGroundheight) * (2.0f*oldConfidence), 1.0f);

    // Do not update known high confidence estimations upward
    if(oldConfidence > 0.5 && groundlevel >= oldGroundheight + mConfig.outlier_tolerance)
        return;

    if(varThresholdsq > std::pow(maxVar, 2.0) && maxVar > 0 && pointsblockSum > (groundDiff * expectedPointCountperLaserperCell * patchSize) * mConfig.ground_patch_detection_minimum_point_count_threshold){
            const float& newConfidence = std::min(pointsblockSum/mConfig.occupied_cells_point_count_factor, 1.0);
            // calculate ground height
            oldGroundheight = (groundlevel*newConfidence + oldConfidence * oldGroundheight*2)/(newConfidence+oldConfidence*2);
            // update confidence
            oldConfidence = std::min((pointsblockSum/(mConfig.occupied_cells_point_count_factor*2.0f) + oldConfidence)/2.0, 1.0);
    }
    else if(localmin < oldGroundheight){
        // update ground height
        oldGroundheight = localmin;
        // update confidence
        oldConfidence = std::min(oldConfidence + 0.1f, 0.5f);
    }
}


void GroundSegmentation::spiral_ground_interpolation(grid_map::GridMap &map, const geometry_msgs::TransformStamped &toBase) const
{
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& gvl = map["groundpatch"];
    const auto& map_size = map.getSize();
    const auto& center_idx = map_size(0)/2-1;

    gvl(center_idx,center_idx) = 1.0f;
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "base_link";
    tf2::doTransform(ps,ps,toBase);

    // Set center to current vehicle height
    ggl(center_idx,center_idx) = ps.point.z;

    for(int i=center_idx-1; i>=1; --i){
        // rectangle_pos = x,y position of rectangle top left corner
        int rectangle_pos = i;

        // rectangle side length
        int side_length = (center_idx-rectangle_pos)*2;

        // top and left side
        for(short side=0; side<2; ++side){
            for(int pos=rectangle_pos; pos<rectangle_pos+side_length; ++pos){
                const int x = side%2 ? pos : rectangle_pos;
                const int y = side%2 ? rectangle_pos : pos;

                interpolate_cell(map, x, y);
            }
        }

        // bottom and right side
        rectangle_pos += side_length;
        for(short side=0; side<2; ++side){
            for(int pos=rectangle_pos; pos>=rectangle_pos-side_length; --pos){
                int x = side%2 ? pos : rectangle_pos;
                int y = side%2 ? rectangle_pos : pos;

                interpolate_cell(map, x, y);
            }
        }
    }
}



void GroundSegmentation::interpolate_cell(grid_map::GridMap &map, const size_t x, const size_t y) const
{
    static const auto& center_idx = map.getSize()(0)/2-1;
    static const size_t blocksize = 3;
    // "groundpatch" layer contains confidence values
    static grid_map::Matrix& gvl = map["groundpatch"];
    // "ground" contains the ground height values
    static grid_map::Matrix& ggl = map["ground"];
    const auto& gvlblock = gvl.block<blocksize,blocksize>(x-blocksize/2,y-blocksize/2);

    float& height = ggl(x,y);
    float& occupied = gvl(x,y);
    const float& gvlSum = gvlblock.sum() + std::numeric_limits<float>::min(); // avoid a possible div by 0
    const float avg = (gvlblock.cwiseProduct(ggl.block<blocksize,blocksize>(x-blocksize/2, y-blocksize/2))).sum()/gvlSum;

    height = (1.0f-occupied) * avg + occupied * height;

    // Only update confidence in cells above min distance
    if((std::pow((float)x-center_idx, 2.0) + std::pow((float)y-center_idx, 2.0)) * std::pow(map.getResolution(), 2.0f) > minDistSquared)
        occupied = std::max(occupied-occupied/mConfig.occupied_cells_decrease_factor, 0.001);
}


void GroundSegmentation::setConfig(const groundgrid::GroundGridConfig &config)
{
    mConfig = config;
}
