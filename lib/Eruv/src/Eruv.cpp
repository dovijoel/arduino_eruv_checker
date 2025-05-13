#include <vector>

struct Coordinate {
    double latitude;
    double longitude;
};

bool isPointInPolygon(const std::vector<Coordinate>& polygon, const Coordinate& point) {
    int n = polygon.size();
    if (n < 3) return false; // A polygon must have at least 3 vertices

    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Coordinate& vi = polygon[i];
        const Coordinate& vj = polygon[j];

        // Check if the point is inside the polygon using the ray-casting algorithm
        if (((vi.latitude > point.latitude) != (vj.latitude > point.latitude)) &&
            (point.longitude < (vj.longitude - vi.longitude) * (point.latitude - vi.latitude) / (vj.latitude - vi.latitude) + vi.longitude)) {
            inside = !inside;
        }
    }

    return inside;
}

bool isPointInEruv(const std::vector<std::vector<Coordinate>>& eruvs, const std::vector<std::vector<Coordinate>>& exclusions, const Coordinate& point) {
    // first checking if in exclusion areas for ealy exit
    for (int i = 0; i < exclusions.size(); i++) {
        const std::vector<Coordinate>& exclusion = exclusions[i];
        if (isPointInPolygon(exclusion, point)) {
            return false; // Point is inside an exclusion area
        }
    }

    // then checking if in eruv areas
    for (int i = 0; i < eruvs.size(); i++) {
        const std::vector<Coordinate>& eruv = eruvs[i];
        bool insideEruv = isPointInPolygon(eruv, point);

        if (insideEruv) {
            return true; // Point is inside the Eruv and not in any exclusion area
        }
    }
    
    return false; // Point is not inside any Eruv
}

