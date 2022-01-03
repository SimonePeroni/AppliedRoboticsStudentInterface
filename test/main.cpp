#include <iostream>
#include "rm/inflate.hpp"

int main(int argc, char **argv)
{
    Polygon p;
    p.push_back(Point(0, 0));
    p.push_back(Point(0, 1));
    p.push_back(Point(1, 1));
    p.push_back(Point(1, 0));
    std::vector<Polygon> polys;
    polys.push_back(p);
    auto inflatedPolys = rm::inflate(polys, 0.1);
    int count = 1;
    for (auto &poly : inflatedPolys)
    {
      std::cout << "Polygon " << count << std::endl;
      for (auto &vertex : poly)
      {
        std::cout << "  x: " << vertex.x << ", y: " << vertex.y << std::endl;
      }
      count++;
    }

    return 0;
}