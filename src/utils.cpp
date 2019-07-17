#include <ed_entity_server/utils.h>

bool isPointInPolygon(const std::vector<geometry_msgs::Point32> &polygon,
                      const geometry_msgs::Point32 &point)
{
    // Based on algorithm here:
    // http://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    //
    // If a horizontal ray from the test point to the right intersects with
    // each segment of the polygon an even number of times, it is not in the polygon
    int num_intersections = 0;
    for (int i = 0; i < polygon.size() - 1; i++)
    {
        if(isPointBetweenPointsInY(point, polygon[i], polygon[i+1]) &&
           doesPointIntersectSegmentOnRight(point, polygon[i], polygon[i+1]))
        {
            num_intersections++;
        }
    }
    // check last segment
    if(isPointBetweenPointsInY(point, polygon[polygon.size() - 1], polygon[0]) &&
       doesPointIntersectSegmentOnRight(point, polygon[polygon.size() - 1], polygon[0]))
    {
        num_intersections++;
    }

    if (num_intersections % 2 == 0)
    {
        return false;
    }
    return true;
}

bool isPointBetweenPointsInY(const geometry_msgs::Point32 &test_point,
                             const geometry_msgs::Point32 &segment_p1,
                             const geometry_msgs::Point32 &segment_p2)
{
    // check if the sign of the difference between the y coordinates of the points is different
    int sign1 = std::copysign(1.0, (test_point.y - segment_p1.y));
    int sign2 = std::copysign(1.0, (test_point.y - segment_p2.y));
    if (sign1 == sign2)
    {
        return false;
    }
    return true;
}


bool doesPointIntersectSegmentOnRight(const geometry_msgs::Point32 &test_point,
                                      const geometry_msgs::Point32 &segment_p1,
                                      const geometry_msgs::Point32 &segment_p2)
{
    // find intersection point and check if t.x is to the left

    // t.x < p1.x + ((p1.x - p1.x)*(t.y - p1.y) / (p2.y - p1.y)
    return (test_point.x <
    segment_p1.x +
    ((segment_p2.x - segment_p1.x) * (test_point.y - segment_p1.y) /
    (segment_p2.y - segment_p1.y)));
}
