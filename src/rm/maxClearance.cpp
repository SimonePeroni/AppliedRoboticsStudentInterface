#include "rm/maxClearance.hpp"
#include "rm/geometry.hpp"

#include "boost/polygon/voronoi.hpp"
#include "clipper/clipper.hpp"

namespace boost
{
    namespace polygon
    {

        template <>
        struct geometry_concept<Point>
        {
            typedef point_concept type;
        };

        template <>
        struct point_traits<Point>
        {
            typedef int coordinate_type;

            static inline coordinate_type get(const Point &point, orientation_2d orient)
            {
                return (orient == HORIZONTAL) ? point.x : point.y;
            }
        };

        template <>
        struct geometry_concept<rm::Segment>
        {
            typedef segment_concept type;
        };

        template <>
        struct segment_traits<rm::Segment>
        {
            typedef int coordinate_type;
            typedef Point point_type;

            static inline point_type get(const rm::Segment &segment, direction_1d dir)
            {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };
    } // polygon
} // boost

namespace rm
{
    RoadMap maxClearance(const std::vector<Polygon> &obstacles, const Polygon &borders)
    {
        const float scale = 1000;

        // 1) Perform union of overlapping polygons
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

        // 2) Scaled segment list from geometry data
        std::vector<Segment> map;
        for (auto const &path : joinedPaths)
        {
            Polygon p;
            for (auto const &vertex : path)
                p.push_back(Point(vertex.X, vertex.Y));
            for (auto const &edge : getEdges(p))
                map.push_back(edge);
        }

        // 3) Build scaled voronoi diagram
        boost::polygon::voronoi_diagram<double> vd;
        boost::polygon::construct_voronoi(map.begin(), map.end(), &vd);

        // 4) Create roadmap draft and revert scale
        RoadMap rm;
        for (auto &edge : vd.edges())
        {
            if (edge.is_finite() && edge.is_primary())
            {
                size_t id0 = rm.addNode(Point(edge.vertex0()->x() / scale, edge.vertex0()->y() / scale));
                size_t id1 = rm.addNode(Point(edge.vertex1()->x() / scale, edge.vertex1()->y() / scale));
                rm.connect(id0, id1);
            }
        }
        return rm;
    }
}