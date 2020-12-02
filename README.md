# SLAM

Data parser and PointCloud usage example

```
#include "dataset.h"
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;
using namespace pcl;

int main() {
    // initialize point cloud
    pcl::PointCloud<pcl::PointXYZI> testCloud;

    // populate point cloud
    DataSet::getPointCloud(testCloud, "/PATH_TO_DATASET/dataset/sequences/00/velodyne/000000.bin");

    // print size of point cloud
    cout << testCloud.size() << endl;

    // iterate through each point in point cloud
    pcl::PointCloud<pcl::PointXYZI>::iterator it;

    for(it = testCloud.begin(); it != testCloud.end(); it++) {
        cout << *it << endl;
    }

    return 0;
}
```

