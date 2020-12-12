//
// Created by Victor Suciu on 2020-12-09.
//

#ifndef SLAM_VISUALUTILS_H
#define SLAM_VISUALUTILS_H


#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <thread>

using namespace std;
using namespace pcl;

class VisualUtils {
private:

public:
    // Visualizes a point cloud in a 3D viewer
    static void viewPointCloud(PointCloud<PointXYZI>::ConstPtr cloud);

    // Visualizes a range imagein a 2D viewer
    static void viewRangeImage(RangeImage::ConstPtr img);
};


#endif //SLAM_VISUALUTILS_H
