#include <geometry_msgs/Point32.h>


bool isPointInPolygon(const std::vector<geometry_msgs::Point32> &polygon,
                      const geometry_msgs::Point32 &point);

bool isPointBetweenPointsInY(const geometry_msgs::Point32 &test_point,
                             const geometry_msgs::Point32 &segment_p1,
                             const geometry_msgs::Point32 &segment_p2);

bool doesPointIntersectSegmentOnRight(const geometry_msgs::Point32 &test_point,
                                      const geometry_msgs::Point32 &segment_p1,
                                      const geometry_msgs::Point32 &segment_p2);
