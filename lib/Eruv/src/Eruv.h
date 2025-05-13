#ifndef IN_ERUV_H
#define IN_ERUV_H

#include <vector>

// Structure to represent a coordinate with latitude and longitude
struct Coordinate {
    double latitude;
    double longitude;
};

// Function to check if a point is inside a polygon
bool isPointInPolygon(const std::vector<Coordinate>& polygon, const Coordinate& point);
bool isPointInEruv(const std::vector<std::vector<Coordinate>>& eruvs, const std::vector<std::vector<Coordinate>>& exclusions, const Coordinate& point);

#endif // IN_ERUV_H