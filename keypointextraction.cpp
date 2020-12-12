//
// Created by Victor Suciu on 2020-12-01.
//

#include "keypointextraction.h"
#include "histogram1d.h"
#include "histogramxyz.h"
#include "visualutils.h"


float KeypointExtraction::BIN_SIZE_X_DISTANCE = 1;
float KeypointExtraction::BIN_SIZE_Y_DISTANCE = 1;
float KeypointExtraction::BIN_SIZE_Z_DISTANCE = 0.1;

float KeypointExtraction::MIN_VALID_X_DISTANCE = -10;
float KeypointExtraction::MAX_VALID_X_DISTANCE = 10;

float KeypointExtraction::MIN_VALID_Y_DISTANCE = -10;
float KeypointExtraction::MAX_VALID_Y_DISTANCE = 10;

float KeypointExtraction::MIN_VALID_Z_DISTANCE = -5;
float KeypointExtraction::MAX_VALID_Z_DISTANCE = 5;

float KeypointExtraction::MIN_VALID_Z_HEIGHT = -1.5;
float KeypointExtraction::MAX_VALID_Z_HEIGHT = 1.5;

/*
 * Purpose: Extracts NARF keypoint features from a point cloud
 *
 * Params:
 * cloud: the raw point cloud to extract features from
 * desscriptors: the container that holds the descriptors found by this function
 */
void KeypointExtraction::extractNarfKeypoints(PointCloud<PointXYZI>& cloud,
        PointCloud<Narf36>::Ptr descriptors, bool showRangeImage, bool showKeypoints) {

    // parameters of the Valodyne Lidar sensor used to generate the KITTI Lidar dataset
    float angularResolutionHoriz = deg2rad (0.08f);
    float angularResolutionVert = deg2rad (0.4f);
    float angularRangeHoriz = deg2rad (360.0f);
    float angularRangeVert = deg2rad (360.0f);
    float support_size = 0.2f;
    float noiseLevel = 0.0;
    float minRange = 0.0f;

    int borderSize = 1;
    RangeImage::CoordinateFrame coordinateFrame = RangeImage::LASER_FRAME;
    bool setUnseenToMaxRange = false;

    PointCloud<PointWithViewpoint> far_ranges;
    Eigen::Affine3f sensorPose (Eigen::Affine3f::Identity ());

    // position and orientation of the lidar sensor in the point cloud.
    // always initialized to all zeros
    sensorPose = Eigen::Affine3f (Eigen::Translation3f (cloud.sensor_origin_[0],
                                                               cloud.sensor_origin_[1],
                                                               cloud.sensor_origin_[2]))
                                       * Eigen::Affine3f (cloud.sensor_orientation_);


    // create the range image
    RangeImage::Ptr rangeImage_ptr (new RangeImage);
    RangeImage& rangeImage = *rangeImage_ptr;

    rangeImage.createFromPointCloud(
            cloud,
            angularResolutionHoriz,
            angularResolutionVert,
            angularRangeHoriz,
            angularRangeVert,
            sensorPose,
            coordinateFrame,
            noiseLevel,
            minRange,
            borderSize);

    rangeImage.integrateFarRanges (far_ranges);



    // Extract NARF keypoint (x, y, z) locations from the range image
    RangeImageBorderExtractor borderExtractor;
    NarfKeypoint narf_keypoint_detector (&borderExtractor);
    narf_keypoint_detector.setRangeImage (&rangeImage);
    narf_keypoint_detector.getParameters ().support_size = support_size;

    PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";


    vector<int> index_vector;
    for(int i = 0; i < keypoint_indices.points.size(); i++) {
        index_vector.push_back(keypoint_indices.points[i]);
    }

    // Get the NARF 36-dimension descriptors for each keypoint
    NarfDescriptor descriptor_calculator(&rangeImage, &index_vector);
    descriptor_calculator.getParameters().support_size = support_size;
    descriptor_calculator.getParameters().rotation_invariant = true;
    descriptor_calculator.compute(*descriptors);

    if(showRangeImage) {
        VisualUtils::viewRangeImage(rangeImage_ptr);
    }
}


/*
 * Purpose: Estimates the motion vector between two
 * point clouds of NARF keypoint descriptors.
 *
 * Params:
 * cloud1: the first point cloud
 * cloud2: the second point cloud
 */
vector<double> KeypointExtraction::findMotionVector(PointCloud<Narf36>::Ptr cloud1, PointCloud<Narf36>::Ptr cloud2) {
    // kd-tree used to find the best matching NARF descriptor
    KdTreeFLANN<Narf36> kdTree;
    kdTree.setInputCloud(cloud2);

    // only search one point. We only want the best matching (nearest) descriptor
    vector<int> pointSearch(1);
    vector<float> squaredDist(1);

    // Estimated motion vector
    vector<double> motion{0, 0, 0};

    // vector histogram
    HistogramXYZ vectorHist(
        Histogram1D(MIN_VALID_X_DISTANCE, MAX_VALID_X_DISTANCE, BIN_SIZE_X_DISTANCE),
        Histogram1D(MIN_VALID_Y_DISTANCE, MAX_VALID_Y_DISTANCE, BIN_SIZE_Y_DISTANCE),
        Histogram1D(MIN_VALID_Z_DISTANCE, MAX_VALID_Z_DISTANCE, BIN_SIZE_Z_DISTANCE));


    // For each point in the first point cloud, find the find the best matching
    // NARF descriptor in the second cloud, and add the (x, y, z) vector
    // between them to the histogram.
    for(Narf36 curr : cloud1->points) {

        if(kdTree.nearestKSearch(curr, 1, pointSearch, squaredDist) > 0) {
            for(int i = 0; i < pointSearch.size(); i++) {
                if((*cloud2)[pointSearch[i]].z >= MIN_VALID_Z_HEIGHT
                && (*cloud2)[pointSearch[i]].z < MAX_VALID_Z_HEIGHT
                && curr.z >= MIN_VALID_Z_HEIGHT
                && curr.z < MAX_VALID_Z_HEIGHT) {

                    vectorHist.insert(
                            (*cloud2)[pointSearch[i]].x - curr.x,
                            (*cloud2)[pointSearch[i]].y - curr.y,
                            (*cloud2)[pointSearch[i]].z - curr.z);
                }
            }
        }

        pointSearch.clear();
        squaredDist.clear();
    }


    int count = 0;

    // Just like the previous loop, for each point in the first point cloud,
    // find the find the best matching NARF descriptor in the second cloud.
    // However, only add it to the motion vector if it falls within the largest
    // histogram bin.
    for(Narf36 curr : cloud1->points) {
        if(kdTree.nearestKSearch(curr, 1, pointSearch, squaredDist) > 0) {
            for(int i = 0; i < pointSearch.size(); i++) {

                if(vectorHist.isMaxBin(
                        (*cloud2)[pointSearch[i]].x - curr.x,
                        (*cloud2)[pointSearch[i]].y - curr.y,
                        (*cloud2)[pointSearch[i]].z - curr.z)
                   && (*cloud2)[pointSearch[i]].z >= MIN_VALID_Z_HEIGHT
                   && (*cloud2)[pointSearch[i]].z < MAX_VALID_Z_HEIGHT
                   && curr.z >= MIN_VALID_Z_HEIGHT
                   && curr.z < MAX_VALID_Z_HEIGHT) {

                    motion[0] += (*cloud2)[pointSearch[i]].x - curr.x;
                    motion[1] += (*cloud2)[pointSearch[i]].y - curr.y;
                    motion[2] += (*cloud2)[pointSearch[i]].z - curr.z;
                    count++;
                }
            }
        }
        pointSearch.clear();
        squaredDist.clear();
    }
    motion[0] /= count;
    motion[1] /= count;
    motion[2] /= count;

    return motion;
}