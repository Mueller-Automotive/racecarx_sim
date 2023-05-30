#pragma once

#include <vector>

#include <gz/math/Vector4.hh>
#include <plugins/road_gen/Line.hpp>

class StraightSegment : public RoadSegment
{
public:
    virtual StraightSegment(gz::math::Vector4<float> _position) : RoadSegment(_position) 
    {
        
    }

    virtual std::string getSdf()
    {
        std::string sdf = document_start;
        for (int i = 0; i < lines.size(); i++)
        {
            sdf += lines[i].getSdf();
        }

        sdf += document_end;
        return sdf;
    }
};
