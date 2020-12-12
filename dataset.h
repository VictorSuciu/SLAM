//
// Created by Victor Suciu on 2020-12-01.
//

#ifndef SLAM_DATASET_H
#define SLAM_DATASET_H


#include <string>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//using namespace std;
//using namespace pcl;


class DataSet {

public:
    // Generates a point cloud by parsing a bin file provided in the KITTI data set
    static bool getPointCloud(pcl::PointCloud<pcl::PointXYZI>& cloud, const std::string path);
};


#endif //SLAM_DATASET_H
