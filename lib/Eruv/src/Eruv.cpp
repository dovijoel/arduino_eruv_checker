#include "Arduino.h"
#include <vector>

struct Coordinate
{
    double latitude;
    double longitude;
};

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

bool isPointInEruv(const std::vector<std::vector<Coordinate>> &eruvs, const std::vector<std::vector<Coordinate>> &exclusions, const Coordinate &point, Print &logger)
{
    // first checking if in exclusion areas for ealy exit
    for (int i = 0; i < exclusions.size(); i++)
    {
        const std::vector<Coordinate> &exclusion = exclusions[i];
        if (isPointInPolygon(exclusion, point, logger))
        {
            logger.println("Point is in exclusion area, returning false...\n");
            return false; // Point is inside an exclusion area
        }
    }

    logger.println("Nothing in exclusion areas, checking eruvs...\n");
    // then checking if in eruv areas
    for (int i = 0; i < eruvs.size(); i++)
    {
        const std::vector<Coordinate> &eruv = eruvs[i];
        bool insideEruv = isPointInPolygon(eruv, point, logger);

        if (insideEruv)
        {
            logger.println("Point is inside the Eruv, returning true...\n");
            return true; // Point is inside the Eruv and not in any exclusion area
        }
    }

    logger.println("Point is not inside any Eruv, returning false...\n");
    return false; // Point is not inside any Eruv
}
