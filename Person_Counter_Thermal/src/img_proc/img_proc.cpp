#include "img_proc.h"
#include "mqtt/mqtt_setup.h"

std::vector<Blob> simpleBlobDetector(float image[]) {
    float minThreshold = 255;
    float maxThreshold = 0;
    std::vector<Blob> blobs;
    int thresholdedImage[64] = {0}; 

    // Set min and max thresholds
    for (int i = 0; i < 64; i++) {
        if (image[i] < minThreshold && image[i] != 0) {
            minThreshold = image[i];
        }
        if (image[i] > maxThreshold) {
            maxThreshold = image[i];
        }
    }

    // Thresholding
    for (float threshold = minThreshold; threshold < maxThreshold; threshold += thresholdStep) {
        std::fill(std::begin(thresholdedImage), std::end(thresholdedImage), 0); // Reset array

        // Threshold image
        for (int i = 0; i < 64; i++) {
            if (image[i] > threshold) {
                thresholdedImage[i] = -1;
            }
        }

        // Find and merge connected pixels using BFS
        int id = 0;
        for (int p = 0; p < 64; p++) {
            if (thresholdedImage[p] == -1) {
                id++;
                std::queue<int> q;
                q.push(p);
                thresholdedImage[p] = id;

                while (!q.empty()) {
                    int current = q.front();
                    q.pop();

                    // Check neighbors
                    int neighbors[] = {current - 9, current - 8, current - 7, current - 1, current + 1, current + 7, current + 8, current + 9};
                    for (int neighbor : neighbors) {
                        if (neighbor >= 0 && neighbor < 64 && thresholdedImage[neighbor] == -1) {
                            thresholdedImage[neighbor] = id;
                            q.push(neighbor);
                        }
                    }
                }
            }
        }

        // Calculate radius and coordinates for each blob
        if (id == 0) continue;
        for (int i = 1; i <= id; i++) {
            float xSum = 0, ySum = 0, size = 0;
            float xMin = 8, xMax = 0, yMin = 8, yMax = 0;

            for (int k = 0; k < 64; k++) {
                if (thresholdedImage[k] == i) {
                    int x = k % 8;
                    int y = k / 8;
                    xSum += x;
                    ySum += y;
                    size++;
                    xMin = std::min(xMin, (float)x);
                    xMax = std::max(xMax, (float)x);
                    yMin = std::min(yMin, (float)y);
                    yMax = std::max(yMax, (float)y);
                }
            }

            if (size > maxArea) continue;

            float x = xSum / size;
            float y = ySum / size;
            float radius = std::min(xMax - xMin, yMax - yMin) / 2 + 1;
            blobs.push_back({x, y, radius});
        }
    }

    // Merging blobs: keep the blob with the smaller radius when two are too close
    size_t i = 0;
    while (i < blobs.size()) {
        bool merged = false;
        for (size_t k = i + 1; k < blobs.size(); ) {
            float distance = hypotf(blobs[i].x - blobs[k].x, blobs[i].y - blobs[k].y);
            if (distance < minDistBetweenBlob) {
                // Keep the blob with the smaller radius
                if (blobs[i].radius > blobs[k].radius) {
                    blobs.erase(blobs.begin() + i);
                    merged = true;
                    break; // Restart for this i
                } else {
                    blobs.erase(blobs.begin() + k);
                    continue; // Stay at same k after erase
                }
            }
            ++k;
        }
        if (!merged) {
            ++i;
        }
    }

    return blobs;
}

std::map<int, Centroid> centroidTracker(std::vector<Blob> &newCentroids, std::map<int, Centroid>& activeCentroids) {

    // Register new centroids if there are no active centroids
    if (activeCentroids.empty()) {
        // For every value in new centroids
        for (int i = 0; i < newCentroids.size(); i++) {
            activeCentroids[indexBank.useIndex()] = {newCentroids[i].x, newCentroids[i].y, newCentroids[i].radius, 0};
        }
    } else { // Else calculate distances between new and active centroids and transfer ids accordingly
        // Set all active centroids update back one
        for (auto& [id, centroid] : activeCentroids) {
            centroid.lastUpdate = centroid.lastUpdate - 1;
        }

        // Update x and y if to the closest points
        // For every active centroid
        for (auto& [id, activeCentroid] : activeCentroids) {
            int closestPointIndex = -1;
            float minDistance = 255;
            if (newCentroids.empty()) {
                break;
            }
            // For every new centroid
            for (int k = 0; k < newCentroids.size(); k++) {
                float distance = hypot(activeCentroid.x - newCentroids[k].x, activeCentroid.y - newCentroids[k].y);
                if (distance < minDistance && distance <= 4) { // Check against max distance
                    minDistance = distance;
                    closestPointIndex = k;
                    // Serial.println("Centroid " + String(id) + " is closest to new centroid " + String(k) + " with distance: " + String(minDistance));
                }
            }
            // Update x, y coordinate for active centroid and delete the new centroid from the list
            if (closestPointIndex != -1) { // Ensure a valid match was found
                activeCentroid.x = newCentroids[closestPointIndex].x;
                activeCentroid.y = newCentroids[closestPointIndex].y;
                activeCentroid.radius = newCentroids[closestPointIndex].radius;
                activeCentroid.lastUpdate = 0;
                newCentroids.erase(newCentroids.begin() + closestPointIndex);
            }
        }

        // Check if there any active centroid that did not get updated, decrease their last update, if its over the threshold then deregister them  
        for (auto it = activeCentroids.begin(); it != activeCentroids.end();) {
            // if (it->second.lastUpdate != 0) {
            //     it->second.lastUpdate = it->second.lastUpdate - 1;
            // }
            if (it->second.lastUpdate < inactiveThreshold) {
                Serial.print("Centroid ");
                Serial.print(it->first);
                Serial.println(" will be deleted");

                indexBank.unuseIndex(it->first); 
                it = activeCentroids.erase(it);
            } else {
                ++it;
            }
        }

        // Check if any new centroid didn't get used, register them 
        if (!newCentroids.empty()) {
            for (int i = 0; i < newCentroids.size(); i++) {
                activeCentroids[indexBank.useIndex()] = {newCentroids[i].x, newCentroids[i].y, newCentroids[i].radius, 0};
            }
        }
    }
    return activeCentroids;
}

void fillSurroundedPixels(float* object, float* filteredPixels) {
    bool changesMade; // Flag to track if changes were made in the current iteration

    do {
        changesMade = false;
        float updatedObject[64] = {0}; // Temporary array to store updated object data

        for (int i = 0; i < 64; i++) {
            // Retain already included pixels
            if (object[i] > 0) {
                updatedObject[i] = object[i];
                continue;
            }

            // Check neighbors (up, down, left, right)
            int neighbors = 0;
            if (i >= 8 && object[i - 8] > 0) neighbors++; // Up
            if (i < 56 && object[i + 8] > 0) neighbors++; // Down
            if (i % 8 != 0 && object[i - 1] > 0) neighbors++; // Left
            if (i % 8 != 7 && object[i + 1] > 0) neighbors++; // Right

            // If the pixel is surrounded by at least 3 object pixels, include it
            if (neighbors >= 2) {
                updatedObject[i] = filteredPixels[i];
                changesMade = true; // Mark that a change was made
            }
        }

        // Copy updatedObject back to the original object array
        for (int i = 0; i < 64; i++) {
            object[i] = updatedObject[i];
        }
    } while (changesMade); // Repeat until no changes are made
}