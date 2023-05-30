#pragma once

#include <vector>

#include <gz/math/Vector3.hh>

#include <plugins/road_gen/RoadSegment.hpp>
#include <plugins/road_gen/Utils.hpp>
#include <plugins/road_gen/Line.hpp>
#include <plugins/road_gen/DashedLine.hpp>

class ParkingSegment : public RoadSegment
{
public:
    ParkingSegment(gz::math::Vector3<float> _pos, gz::math::Vector3<float> _rot) : RoadSegment(_pos, _rot) 
    {
        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(0.0, 4.0, 0.01),
                gz::math::Vector3<float>(2.0, 6.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                0));

        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(2.0, 6.0, 0.01),
                gz::math::Vector3<float>(6.0, 6.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                0));

        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(6.0, 6.0, 0.01),
                gz::math::Vector3<float>(8.0, 4.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                0));

        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(2.0, 4.0, 0.01),
                gz::math::Vector3<float>(2.0, 6.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                0));

        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(6.0, 4.0, 0.01),
                gz::math::Vector3<float>(6.0, 6.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                0));
    }
};
