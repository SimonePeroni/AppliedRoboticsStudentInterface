#include "rm/maxClearance.hpp"
#include "rm/geometry.hpp"

#include "boost/polygon/voronoi.hpp"
#include "clipper/clipper.hpp"

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
struct boost::polygon::geometry_concept<rm::Segment>
{
    typedef segment_concept type;
};

template <>
struct boost::polygon::point_traits<rm::Segment>
{
    typedef int coordinate_type;
    typedef Point point_type;

    static inline point_type get(const rm::Segment &segment, direction_1d dir)
    {
        return dir.to_int() ? segment.p1 : segment.p0;
    }
};

namespace rm
{
    void maxClearance(const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        // 1) Perform union of overlapping polygons
        const float scale = 1000;

        ClipperLib::Paths joinedPaths;
        ClipperLib::Clipper clpr;

        for (auto const &obstacle : obstacles)
        {
            ClipperLib::Path srcPath;

            for (auto const &vertex : obstacle)
                srcPath << ClipperLib::IntPoint(vertex.x * scale, vertex.y * scale);

            clpr.AddPath(srcPath, ClipperLib::ptClip, true);
        }

        ClipperLib::Path borderPath;
        for (auto const &vertex : borders)
            borderPath << ClipperLib::IntPoint(vertex.x * scale, vertex.y * scale);
        clpr.AddPath(borderPath, ClipperLib::ptSubject, true);

        clpr.Execute(ClipperLib::ctDifference, joinedPaths);

        // 2) Segment list from geometry data
        std::vector<Segment> map;
        for (auto const &path : joinedPaths)
        {
            Polygon p;
            for (auto const &vertex : path)
                p.push_back(Point(vertex.X / scale, vertex.Y / scale));
            for (auto const &edge : getEdges(p))
                map.push_back(edge);
        }

        // 3) Build voronoi diagram
        
    }
}