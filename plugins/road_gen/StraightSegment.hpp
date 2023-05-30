#pragma once

#include <vector>

#include <plugins/road_gen/RoadSegment.hpp>
#include <gz/math/Vector3.hh>
#include <plugins/road_gen/Line.hpp>
#include <plugins/road_gen/DashedLine.hpp>

class StraightSegment : public RoadSegment
{
public:
    StraightSegment(gz::math::Vector3<float> _position) : RoadSegment(_position) 
    {
        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(0.0, 0.0, 0.01),
                gz::math::Vector3<float>(10.0, 0.0, 0.01),
                0));

        lines.push_back(new Line(getId(), 
                gz::math::Vector3<float>(0.0, 4.0, 0.01),
                gz::math::Vector3<float>(10.0, 4.0, 0.01),
                0));

        lines.push_back(new DashedLine(getId(),
                gz::math::Vector3<float>(0.0, 2.0, 0.01),
                gz::math::Vector3<float>(10.0, 2.0, 0.01),
                0));
    }

    virtual std::vector<std::string> getModelsSdf()
    {
        std::vector<std::string> models;
        for (int i = 0; i < lines.size(); i++)
        {
            models.push_back(lines[i]->getSdf());
        }

        return models;
    }
};
