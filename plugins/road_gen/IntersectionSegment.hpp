#pragma once

#include <vector>

#include <plugins/road_gen/RoadSegment.hpp>
#include <gz/math/Vector3.hh>
#include <plugins/road_gen/Line.hpp>
#include <plugins/road_gen/DashedLine.hpp>

class IntersectionSegment : public RoadSegment
{
public:
    IntersectionSegment(gz::math::Vector3<float> _pos, gz::math::Vector3<float> _rot) : RoadSegment(_pos, _rot) 
    {
        // bottom left
        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(0.0, 4.0, 0.01),
                gz::math::Vector3<float>(5.0, 9.0, 0.01),
                pos,
                rot,
                -90));

        // bottom right
        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                gz::math::Vector3<float>(5.0, -5.0, 0.01),
                pos,
                rot,
                90));

        // top left
        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(9.0, 9.0, 0.01),
                gz::math::Vector3<float>(14.0, 4.0, 0.01),
                pos,
                rot,
                -90));

        // top right
        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(9.0, -5.0, 0.01),
                gz::math::Vector3<float>(14.0, 0.0, 0.01),
                pos,
                rot,
                90));

        
    }
};
