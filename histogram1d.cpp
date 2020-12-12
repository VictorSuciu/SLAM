//
// Created by Victor Suciu on 2020-12-09.
//

#include "histogram1d.h"


/*
 * Purpose: Constructs a histogram with one bin with the range 0 - 0.999...
 */
Histogram1D::Histogram1D() {
    this->minValue = 0;
    this->maxValue = 1;
    this->binSize = 1;
    this->maxIndex = 0;
    this->maxIsUpToDate = true;
    for(int i = 0; i < (int)((maxValue - minValue) / binSize); i++) {
        hist.push_back(0);
    }

}

/*
 * Purpose: Constructs a histogram given a minimum value, maximum value, and bin size
 *
 * Params:
 * minValue: the minimum allowed value the histogram can count
 * maxValue: the maximum allowed value the histogram can count
 * binSze: the range of each bin
 */
Histogram1D::Histogram1D(double minValue, double maxValue, double binSize) {
    this->minValue = minValue;
    this->maxValue = maxValue;
    this->binSize = binSize;
    this->maxIndex = 0;
    this->maxIsUpToDate = true;
    for(int i = 0; i < (int)((maxValue - minValue) / binSize); i++) {
        hist.push_back(0);
    }

}

/*
 * Purpose: Counts a value in one of the bins if it's within the allowed range
 *
 * Params:
 * val: the value to count
 */
bool Histogram1D::insert(double val) {
    if(withinRange(val)) {
        hist[(int) ((val - minValue) / binSize)]++;
        maxIsUpToDate = false;
        return true;
    }
    return false;
}

/*
 * Purpose: finds the index of the bin with the most counts and returns its index
 */
int Histogram1D::computeMaxBin() {
    int max = 0;
    maxIndex = 0;
    for(int i = 0; i < hist.size(); i++) {
        if(hist[i] > max) {
            max = hist[i];
            maxIndex = i;
        }
    }
    return maxIndex;
}

/*
 * Purpose: Checks whether a given value would fall within the largest bin
 *
 * Params:
 * val: the value to test
 */
bool Histogram1D::isMaxBin(double val) {
    if(!maxIsUpToDate) {
        computeMaxBin();
    }
    if(!withinRange(val)) {
        return false;
    }
    return (int) ((val - minValue) / binSize) == maxIndex;
}

/*
 * Purpose: Returns the current count in one of the bins
 *
 * Params:
 * index: the index of the bin to return
 */
int Histogram1D::getBinCount(int index) {
    if(index >= 0 && index < hist.size()) {
        return hist[index];
    }
    return -1;
}

/*
 * Purpose: Checks if a value is within the allowed range of the histogram
 *
 * Params:
 * val: the value to test
 */
bool Histogram1D::withinRange(double val) {
    return val > minValue && val < maxValue;
}

