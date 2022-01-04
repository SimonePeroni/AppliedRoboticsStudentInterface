#include "rm/maxClearance.hpp"
#include "rm/geometry.hpp"

#include "boost/polygon/voronoi.hpp"

template <>
struct boost::polygon::geometry_concept<Point>
{
    typedef point_concept type;
};

template <>
struct boost::polygon::point_traits<Point>
{
    typedef int coordinate_type;

    static inline coordinate_type get(const Point &point, boost::polygon::orientation_2d orient)
    {
        return (orient == boost::polygon::HORIZONTAL) ? point.x : point.y;
    }
};

template <>
struct boost::polygon::geometry_concept<rm::Segment> { typedef segment_concept type; };

template <>
struct boost::polygon::point_traits<rm::Segment> {
  typedef int coordinate_type;
  typedef Point point_type;
    
  static inline point_type get(const rm::Segment& segment, direction_1d dir) {
    return dir.to_int() ? segment.p1 : segment.p0;
  }
};

namespace rm
{
    void maxClearance(const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        // Perform union of overlapping polygons
        // Segment list from geometry data
        // Build voronoi diagram
    }
}