#ifndef IN_ERUV_H
#define IN_ERUV_H

#include <vector>

// Structure to represent a coordinate with latitude and longitude
struct Coordinate {
    double latitude;
    double longitude;
};

struct EruvStatus {
    bool inEruv;
    double distanceToEdge; // meters, -1 if not in eruv

    // Allow assignment to volatile EruvStatus
    volatile EruvStatus& operator=(const EruvStatus& other) volatile {
        inEruv = other.inEruv;
        distanceToEdge = other.distanceToEdge;
        return *this;
    }

    // Allow assignment from non-volatile to volatile EruvStatus
    EruvStatus& operator=(const EruvStatus& other) {
        inEruv = other.inEruv;
        distanceToEdge = other.distanceToEdge;
        return *this;
    }

    EruvStatus& operator=(const volatile EruvStatus& other) {
        inEruv = other.inEruv;
        distanceToEdge = other.distanceToEdge;
        return *this;
    }
};

// Function to check if a point is inside a polygon
bool isPointInPolygon(const std::vector<Coordinate>& polygon, const Coordinate& point, Print& logger);
EruvStatus isPointInEruv(const std::vector<std::vector<Coordinate>>& eruvs, const std::vector<std::vector<Coordinate>>& exclusions, const Coordinate& point, Print& logger);

#endif // IN_ERUV_H