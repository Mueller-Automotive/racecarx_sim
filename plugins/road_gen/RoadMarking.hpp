#pragma once

#include <vector>

#include <gz/math/Vector4.hh>
#include <plugins/road_gen/Line.hpp>

virtual class RoadSegment
{
public:
    virtual RoadSegment();
    gz::math::Vector4 position;
    std::vector<Line> lines;
private:
    
};
