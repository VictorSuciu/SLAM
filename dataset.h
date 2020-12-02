//
// Created by Victor Suciu on 2020-12-01.
//

#ifndef SLAM_DATASET_H
#define SLAM_DATASET_H


#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;
using namespace pcl;


class DataSet {

public:
    static void getPointCloud(PointCloud<PointXYZI>& cloud, const string path);
};


#endif //SLAM_DATASET_H
