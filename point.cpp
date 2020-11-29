//
// Created by Victor Suciu on 2020-11-28.
//

#include "point.h"

Point::Point(float x, float y, float z, float r) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->r = r;
}

ostream& operator<<(ostream& out, const Point p) {
    out << "(" << p.x << ", " << p.y << ", " << p.z << ", " << p.r << ")";
    return out;
}
