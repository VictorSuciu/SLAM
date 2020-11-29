//
// Created by Victor Suciu on 2020-11-28.
//

#include "pointcloud.h"

PointCloud::PointCloud() {}


PointCloud::PointCloud(string path) {
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen (path.c_str(),"rb");
    num = fread(data,sizeof(float),num,stream)/4;

    for (int32_t i=0; i<num; i++) {
        points.push_back(Point(*px,*py,*pz,*pr));
        px+=4; py+=4; pz+=4; pr+=4;
    }
    free(data);
    fclose(stream);
}


void PointCloud::add(float x, float y, float z, float r) {
    points.push_back(Point(x, y, z, r));
}


Point PointCloud::get(int index) {
    return points[index];
}

int PointCloud::size() {
    return points.size();
}

