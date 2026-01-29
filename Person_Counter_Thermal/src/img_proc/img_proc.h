#ifndef IMG_PROC_H
#define IMG_PROC_H

#include <Arduino.h>
#include <vector>
#include <map>
#include <cmath>
#include <queue>
#include <algorithm>
#include "config.h"
#include "types.h"

extern const float thresholdStep;
extern const float minDistBetweenBlob;
extern const int maxArea;
extern const int inactiveThreshold;
extern IndexBank indexBank;

std::vector<Blob> simpleBlobDetector(float image[]);
std::map<int, Centroid> centroidTracker(std::vector<Blob> &newCentroids, std::map<int, Centroid>& activeCentroids);
void fillSurroundedPixels(float* object, float* filteredPixels);

#endif // IMG_PROC_H