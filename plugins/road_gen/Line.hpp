#pragma once

#include <string>

#include <gz/math/Vector4.hh>
#include <plugins/road_gen/LineType.hpp>

#define FMT_HEADER_ONLY
#include <include/fmt/format.h>

class Line
{
public:
    std::string name;
    gz::math::Vector4<float> point1;
    gz::math::Vector4<float> point2;
    gz::math::Vector4<float> rot;
    LineType lineType;

    Line(std::string _name, gz::math::Vector4<float> _point1, gz::math::Vector4<float> _point2, gz::math::Vector4<float> _rot)
    {
        name = _name;
        point1 = _point1;
        point2 = _point2;
        rot = _rot;
    }

    virtual std::string getSdf()
    {
        std::string model_start = fmt::format("<?xml version='1.0'?><sdf version='1.7'><model name='{}'><static>true</static><self_collide>false</self_collide>", name);
        std::string model_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", point1[0], point1[1], point1[2], rot[0], rot[1], rot[2]);
        std::string model_end = "</model></sdf>";

        std::string plane_start = "<link name='plane1'> \
<visual name='visual'>\
<geometry>\
<plane>\
<normal>0 0 1</normal>\
<size>0.4 0.4</size>\
</plane>\
</geometry>\
<material>\
<ambient>0.7 0.7 0.7 1</ambient>\
<diffuse>0.7 0.7 0.7 1</diffuse>\
<specular>0 0 0 0</specular>\
<emissive>0 0 0 1</emissive>\
</material>\
</visual>";
        
        float x = 0;
        float y = 0;
        float z = 0;
        float r_x = 0;
        float r_y = 0;
        float r_z = 0;
        
        std::string plane_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", x, y, z, r_x, r_y, r_z);
        std::string plane_end = "</link>";

        std::string sdf = model_start + model_pose + plane_start + plane_pose + plane_end + model_end;
        return sdf;
    }
private:
    
};
