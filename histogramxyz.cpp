//
// Created by Victor Suciu on 2020-12-09.
//

#include "histogramxyz.h"

/*
 * Purpose: Constructs an xyz histogram using three 1d histograms
 *
 * Params:
 * histx: the histogram for the x direction
 * histy: the histogram for the y direction
 * histz: the histogram for the z direction
 */
HistogramXYZ::HistogramXYZ(Histogram1D histx, Histogram1D histy, Histogram1D histz) {
    this->histx = histx;
    this-> histy = histy;
    this-> histz = histz;
}

/*
 * Purpose: inserts an (x, y, z) into the histogram
 *
 * Params:
 * x: the x value
 * y; the y value
 * z: the z value
 */
bool HistogramXYZ::insert(float x, float y, float z) {
    if(histx.withinRange(x) && histy.withinRange(y) && histz.withinRange(z)) {
        histx.insert(x);
        histy.insert(y);
        histz.insert(z);
        return true;
    }
    return false;
}

/*
 * Purpose: Checks whether an (x, y, z) value falls within the
 * largest histogram bin
 *
 * Params:
 * x: the x value
 * y; the y value
 * z: the z value
 */
bool HistogramXYZ::isMaxBin(float x, float y, float z) {
    return histx.isMaxBin(x) && histy.isMaxBin(y) && histz.isMaxBin(z);
}

