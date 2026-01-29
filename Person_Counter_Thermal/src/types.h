#ifndef TYPES_H
#define TYPES_H

#include <queue>
#include <vector>
#include <cmath>
#include <stdexcept>

// Structure Definitions
struct Blob { float x, y, radius; };
struct Centroid { float x, y, radius, tempDiff; int lastUpdate; };

// Index management class
class IndexBank {
    private:
        std::queue<int> freeIndices;
        int maxIndex = 0;

    public:
        IndexBank(int size)
        {
            for (int i = 0; i < size; i++)
            {
                freeIndices.push(i);
            }
            maxIndex = size;
        }

        int useIndex()
        {
            if (freeIndices.empty())
            {
                throw std::runtime_error("No free indices available");
            }

            int index = freeIndices.front();
            freeIndices.pop();
            return index;
        }

        void unuseIndex(int index)
        {
            if (index >= maxIndex)
            {
                throw std::runtime_error("Index out of range");
            }

            freeIndices.push(index);
        }
};

#endif // TYPES_H