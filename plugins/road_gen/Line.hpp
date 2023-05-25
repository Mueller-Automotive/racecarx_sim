#pragma once

#include <gz/math/Vector4.hh>
#include <plugins/road_gen/LineType.hpp>

class Line
{
public:
    gz::math::Vector4 point1;
    gz::math::Vector4 point2;
    LineType lineType;
private:
    
};
