//
// Created by Victor Suciu on 2020-12-09.
//

#include "visualutils.h"


/*
 * Purpose: Visualizes a point cloud in a 3D viewer
 *
 * Params:
 * cloud: the point cloud to visualize
 */
void VisualUtils::viewPointCloud(PointCloud<PointXYZI>::ConstPtr cloud) {
    visualization::PCLVisualizer viewer("visual");
    viewer.addCoordinateSystem(8, 0, 0, 0);
    viewer.setBackgroundColor(0, 0, 0);

    viewer.addPointCloud<pcl::PointXYZI>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(
            visualization::PCL_VISUALIZER_POINT_SIZE, 1,
            "cloud");
    viewer.initCameraParameters();
    viewer.setCameraPosition (50, 0, 20, 1, 1, 1);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

/*
 * Purpose: Visualizes a range imagein a 2D viewer
 *
 * Params:
 * img: the range image to visualize
 */
void VisualUtils::viewRangeImage(RangeImage::ConstPtr img) {
    visualization::RangeImageVisualizer range_image_widget(
            "Range image");
    range_image_widget.showRangeImage(*img);
    while (!range_image_widget.wasStopped()) {
        range_image_widget.spinOnce();
        pcl_sleep(0.01);
    }
}




