#pragma once

#include <vector>

#include <gz/math/Vector3.hh>

#include <plugins/road_gen/Line.hpp>
#include <plugins/road_gen/Utils.hpp>

class RoadSegment
{
public:
    RoadSegment(gz::math::Vector3<float> _pos, gz::math::Vector3<float> _rot)
    {
        pos = _pos;
        rot = _rot;
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

protected:
    gz::math::Vector3<float> pos;
    gz::math::Vector3<float> rot;
    std::vector<Line*> lines;

    std::string document_start = "<?xml version='1.0'?><sdf version='1.7'>";
    std::string document_end = "</sdf>";
    unsigned int id = 0;

    std::string getId()
    {
        id++;
        return "road-element-" + std::to_string(id);
    }
};
