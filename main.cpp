#include <iostream>
#include <fstream>
#include "math.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "pointcloud.h"


using namespace std;
using namespace cv;


int main() {
    PointCloud testCloud("/Volumes/ROCKET-PRO/Victor/dataset/sequences/00/velodyne/000000.bin");
    cout << testCloud.size() << endl;
    for(int i = 0; i < 500; i++) {
        cout << testCloud.get(i) << endl;
    }
    return 0;
}