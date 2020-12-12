//
// Created by Victor Suciu on 2020-12-01.
//

#ifndef SLAM_KEYPOINTEXTRACTION_H
#define SLAM_KEYPOINTEXTRACTION_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>
#include <thread>
using namespace std;
using namespace pcl;


class KeypointExtraction {

private:
    // The bin sizes for the x, y, an z histograms
    static float BIN_SIZE_X_DISTANCE;
    static float BIN_SIZE_Y_DISTANCE;
    static float BIN_SIZE_Z_DISTANCE;

    // the minimum and madimum distance values for the x, y, and z histograms
    static float MIN_VALID_X_DISTANCE;
    static float MAX_VALID_X_DISTANCE;

    static float MIN_VALID_Y_DISTANCE;
    static float MAX_VALID_Y_DISTANCE;

    static float MIN_VALID_Z_DISTANCE;
    static float MAX_VALID_Z_DISTANCE;

    // the minimum and maximum allowed z-height for keypoints
    // used to eliminate keypoints on the flat road surface
    // by setting the minimum z-height above the road's surface.
    // the road is at about -1.7 z
    static float MIN_VALID_Z_HEIGHT;
    static float MAX_VALID_Z_HEIGHT;

    static float SEARCH_RADIUS;


public:
    // Extracts NARF keypoint features from a point cloud
    static void extractNarfKeypoints(PointCloud<PointXYZI>& cloud,
                                     PointCloud<Narf36>::Ptr descriptors,
                                     bool showRangeImage, bool showKeypoints);

    // Estimates the motion vector between two
    // point clouds of NARF keypoint descriptors.
    static vector<double> findMotionVector(PointCloud<Narf36>::Ptr cloud1,
                                           PointCloud<Narf36>::Ptr cloud2);

    // Estimates the motion vector between two
    // point clouds of NARF keypoint descriptors using the limited search radius technique.
    static vector<double> findMotionVectorWithSearchRadius(
            PointCloud<Narf36>::Ptr cloud1, PointCloud<Narf36>::Ptr cloud2);

    // builds an XYZ point cloud from NARF descriptors and builds a PointXYZ -> Narf36 map
    static void buildLocationCloud(
            PointCloud<Narf36>::Ptr narfCloud, PointCloud<PointXYZ>::Ptr locationCloud,
            map<vector<double>, vector<double>>& locToNarf);

    // computes euclidean distance between two NARF descriptors.
    static double distanceBetweenDescriptors(vector<double> descriptor1, vector<double> descriptor2);

};


#endif //SLAM_KEYPOINTEXTRACTION_H
