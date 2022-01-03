#include "rm/inflate.hpp"

namespace rm
{
    std::vector<Polygon> inflate(const std::vector<Polygon> &polygons, float offset)
    {
        std::vector<Polygon> inflatedPolygons;

        const float scale = 1000;

        ClipperLib::Paths inflatedPaths;
        ClipperLib::ClipperOffset co;

        for (auto const &polygon : polygons)
        {
            ClipperLib::Path srcPath;

            for (auto const &vertex : polygon)
                srcPath << ClipperLib::IntPoint(vertex.x * scale, vertex.y * scale);

            co.AddPath(srcPath, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        }
        co.Execute(inflatedPaths, offset * scale);

        for (auto const &path : inflatedPaths)
        {
            Polygon p;
            for (auto const &vertex : path)
                p.push_back(Point(vertex.X / scale, vertex.Y / scale));
            
            inflatedPolygons.push_back(p);
        }

        return inflatedPolygons;
    }
}