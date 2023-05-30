#pragma once

#include <vector>

#include <gz/math/Vector4.hh>
#include <plugins/road_gen/Line.hpp>

class RoadSegment
{
public:
    virtual RoadSegment(gz::math::Vector4<float> _position)
    {
        position = _position;
    }
    virtual std::string getSdf() = 0;
protected:
    gz::math::Vector4<float> position;
    std::vector<Line> lines;

    std::string document_start = "<?xml version='1.0'?><sdf version='1.7'>";
    std::string document_end = "</sdf>";
};
