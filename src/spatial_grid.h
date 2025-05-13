#ifndef SPATIAL_GRID_H
#define SPATIAL_GRID_H

#include <Arduino.h>
#include <vector>
#include "vec2.h"

// A simpler implementation that avoids circular dependencies
// This grid system stores pointers to void which will be cast to Boid*
// This avoids including boid.h and creating circular dependencies
class SpatialGrid {
private:
    int gridWidth;
    int gridHeight;
    float cellWidth;
    float cellHeight;
    std::vector<void*>* grid;
    int maxObjectsPerCell;

public:
    SpatialGrid(int width, int height, float worldWidth, float worldHeight, int maxObjects = 6) {
        this->gridWidth = width;
        this->gridHeight = height;
        this->cellWidth = worldWidth / width;
        this->cellHeight = worldHeight / height;
        this->maxObjectsPerCell = maxObjects;
        
        // Allocate the grid
        grid = new std::vector<void*>[width * height];
        
        // Pre-allocate memory for each cell
        for (int i = 0; i < width * height; i++) {
            grid[i].reserve(maxObjects);
        }
    }
    
    ~SpatialGrid() {
        delete[] grid;
    }
    
    // Clear all cells in the grid
    void clear() {
        for (int i = 0; i < gridWidth * gridHeight; i++) {
            grid[i].clear();
        }
    }
    
    // Add an object to the appropriate cell
    void insert(void* object, float x, float y) {
        int cellX = (int)(x / cellWidth);
        int cellY = (int)(y / cellHeight);
        
        // Clamp to grid boundaries
        cellX = constrain(cellX, 0, gridWidth - 1);
        cellY = constrain(cellY, 0, gridHeight - 1);
        
        // Add to the appropriate cell
        int cellIndex = cellY * gridWidth + cellX;
        if (grid[cellIndex].size() < maxObjectsPerCell) {
            grid[cellIndex].push_back(object);
        }
    }
    
    // Get all objects within a certain radius of a point
    std::vector<void*> getNeighbors(float x, float y, float radius) {
        std::vector<void*> neighbors;
        neighbors.reserve(maxObjectsPerCell * 9); // Worst case: 9 cells, all full
        
        // Calculate the cell range to check
        int minCellX = max(0, (int)((x - radius) / cellWidth));
        int maxCellX = min(gridWidth - 1, (int)((x + radius) / cellWidth));
        int minCellY = max(0, (int)((y - radius) / cellHeight));
        int maxCellY = min(gridHeight - 1, (int)((y + radius) / cellHeight));
        
        // Check all cells in the range
        for (int cellY = minCellY; cellY <= maxCellY; cellY++) {
            for (int cellX = minCellX; cellX <= maxCellX; cellX++) {
                int cellIndex = cellY * gridWidth + cellX;
                
                // Add all objects from this cell
                for (void* obj : grid[cellIndex]) {
                    neighbors.push_back(obj);
                }
            }
        }
        
        return neighbors;
    }
};

#endif // SPATIAL_GRID_H
