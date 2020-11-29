//
// Created by Victor Suciu on 2020-11-28.
//

#ifndef SLAM_POINTCLOUD_H
#define SLAM_POINTCLOUD_H


#include "point.h"
#include <vector>
#include <string>

using namespace std;


class PointCloud {
private:
    vector<Point> points;

public:
    PointCloud();
    PointCloud(string path);

    void add(float x, float y, float z, float r);
    Point get(int index);
    int size();
};


#endif //SLAM_POINTCLOUD_H
