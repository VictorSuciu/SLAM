//
// Created by Victor Suciu on 2020-11-28.
//

#ifndef SLAM_POINT_H
#define SLAM_POINT_H

#include <ostream>


using namespace std;


class Point {
public:
    float x;
    float y;
    float z;
    float r;

    Point(float x, float y, float z, float r);

    friend ostream& operator<<(ostream& out, const Point p);
};


#endif //SLAM_POINT_H
