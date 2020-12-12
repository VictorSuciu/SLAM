//
// Created by Victor Suciu on 2020-12-09.
//

#ifndef SLAM_HISTOGRAMXYZ_H
#define SLAM_HISTOGRAMXYZ_H

#include "histogram1d.h"
#include <iostream>

using namespace std;


class HistogramXYZ {
private:
    Histogram1D histx; // the x histogram
    Histogram1D histy; // the y histogram
    Histogram1D histz; // the z histogram

public:

    // Constructs an xyz histogram using three 1d histograms
    HistogramXYZ(Histogram1D histx, Histogram1D histy, Histogram1D histz);

    // Purpose: inserts an (x, y, z) into the histogram
    bool insert(float x, float y, float z);

    // Purpose: Checks whether an (x, y, z) value falls within the largest histogram bin
    bool isMaxBin(float x, float y, float z);
};


#endif //SLAM_HISTOGRAMXYZ_H
