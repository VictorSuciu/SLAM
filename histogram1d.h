//
// Created by Victor Suciu on 2020-12-09.
//

#ifndef SLAM_HISTOGRAM_H
#define SLAM_HISTOGRAM_H

#include <iostream>
#include <vector>

using namespace std;

class Histogram1D {
private:
    double minValue; // the minimum value the histogram is allowed to count
    double maxValue; // the maximum value the histogram is allowed to count
    double binSize; // the range of each bin
    int maxIndex; // the index of the current maximum bin. Updated when necessary
    bool maxIsUpToDate; // whether or not maxIndex must be updated in the event it is requested
    vector<int> hist; // the histogram vector. Each element stores the count of a bin

public:
    // Purpose: Constructs a histogram with one bin with the range 0 - 0.999...
    Histogram1D();

    // Constructs a histogram given a minimum value, maximum value, and bin size
    Histogram1D(double minValue, double maxValue, double binSize);

    // Counts a value in one of the bins if it's within the allowed range
    bool insert(double val);

    // Purpose: finds the index of the bin with the most counts and returns its index
    int computeMaxBin();

    // Checks whether a given value would fall within the largest bin
    bool isMaxBin(double val);

    // Returns the current count in one of the bins
    int getBinCount(int index);

    // Checks if a value is within the allowed range of the histogram
    bool withinRange(double val);

};


#endif //SLAM_HISTOGRAM_H
