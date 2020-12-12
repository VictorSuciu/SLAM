//
// Created by Victor Suciu on 2020-12-01.
//

#include "dataset.h"

/*
 * Purpose: Generates a point cloud by parsing a bin file provided
 * in the KITTI data set
 *
 * Params:
 * cloud: the resulting point cloud is stored/returned here.
 * path: the file path to a point cloud .bin file
 */
bool DataSet::getPointCloud(pcl::PointCloud<pcl::PointXYZI>& cloud, const std::string path) {

    if(!std::ifstream(path)) {
        return false; // file not found
    }


    /*
     * code that parses the KITTI dataset point cloud .bin files provided by:
     * https://github.com/yanii/kitti-pcl/blob/master/KITTI_README.TXT
     */
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
        cloud.push_back(pcl::PointXYZI(*px,*py,*pz,*pr));
        px+=4; py+=4; pz+=4; pr+=4;
    }
    free(data);
    fclose(stream);
    return true;
}

