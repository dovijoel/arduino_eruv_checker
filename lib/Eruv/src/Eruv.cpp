#include "Arduino.h"
#include <vector>
#include <cmath>
#include "Eruv.h"

double haversine(double lat1, double lon1, double lat2, double lon2) {
    // Returns distance in meters between two lat/lon points
    const double R = 6371000.0; // Earth radius in meters
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

double pointToSegmentDistance(const Coordinate& p, const Coordinate& v, const Coordinate& w) {
    // Returns the minimum distance from point p to the segment vw (in meters)
    double lat1 = v.latitude, lon1 = v.longitude;
    double lat2 = w.latitude, lon2 = w.longitude;
    double lat0 = p.latitude, lon0 = p.longitude;
    // Convert to radians for calculations
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double t = ((lat0 - lat1) * dLat + (lon0 - lon1) * dLon) / (dLat * dLat + dLon * dLon);
    t = fmax(0, fmin(1, t));
    double projLat = lat1 + t * dLat;
    double projLon = lon1 + t * dLon;
    return haversine(lat0, lon0, projLat, projLon);
}

bool isPointInPolygon(const std::vector<Coordinate> &polygonVertices, const Coordinate &testPoint, Print &logger)
{
    size_t vertexCount = polygonVertices.size();
    if (vertexCount < 3)
    {
        return false; // A valid polygon must have at least 3 vertices
    }

    bool isInside = false;

    // Loop through each edge of the polygon
    for (size_t current = 0, previous = vertexCount - 1; current < vertexCount; previous = current++)
    {
        const Coordinate &currentVertex = polygonVertices[current];
        const Coordinate &previousVertex = polygonVertices[previous];

        bool crossesLatitude = (currentVertex.latitude > testPoint.latitude) != (previousVertex.latitude > testPoint.latitude);
        if (crossesLatitude)
        {
            double intersectionLongitude = (previousVertex.longitude - currentVertex.longitude) *
                                               (testPoint.latitude - currentVertex.latitude) /
                                               (previousVertex.latitude - currentVertex.latitude) +
                                           currentVertex.longitude;

            if (testPoint.longitude < intersectionLongitude)
            {
                isInside = !isInside;
            }
        }
    }

    return isInside;
}

EruvStatus isPointInEruv(const std::vector<std::vector<Coordinate>> &eruvs, const std::vector<std::vector<Coordinate>> &exclusions, const Coordinate &point, Print &logger)
{
    EruvStatus status = {false, -1};
    // Check exclusion areas first
    for (int i = 0; i < exclusions.size(); i++)
    {
        const std::vector<Coordinate> &exclusion = exclusions[i];
        if (isPointInPolygon(exclusion, point, logger))
        {
            logger.println("Point is in exclusion area, returning false...\n");
            status.inEruv = false;
            return status;
        }
    }
    logger.println("Nothing in exclusion areas, checking eruvs...\n");
    // Check eruv polygons
    for (int i = 0; i < eruvs.size(); i++)
    {
        const std::vector<Coordinate> &eruv = eruvs[i];
        bool insideEruv = isPointInPolygon(eruv, point, logger);
        if (insideEruv)
        {
            logger.println("Point is inside the Eruv, calculating distance to edge...\n");
            // Calculate minimum distance to any edge of this eruv
            double minDist = -1;
            for (size_t j = 0, k = eruv.size() - 1; j < eruv.size(); k = j++) {
                double dist = pointToSegmentDistance(point, eruv[k], eruv[j]);
                if (minDist < 0 || dist < minDist) minDist = dist;
            }
            status.inEruv = true;
            status.distanceToEdge = minDist;
            return status;
        }
    }
    logger.println("Point is not inside any Eruv, returning false...\n");
    return status;
}
