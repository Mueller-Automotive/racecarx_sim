#pragma once

#include <plugins/road_gen/Line.hpp>

class DashedLine : public Line
{
public:
    DashedLine(std::string _name, gz::math::Vector4<float> _point1, gz::math::Vector4<float> _point2, gz::math::Vector4<float> _pos, gz::math::Vector4<float> _rot)
    : Line(_name, _point1, _point2, _pos, _rot) {}

    virtual std::string getSdf()
    {
        float distance = point1.Distance(point2);

        std::string model_start = fmt::format("<?xml version='1.0'?><sdf version='1.7'><model name='{}'><static>true</static><self_collide>false</self_collide>", name);
        std::string model_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]);
        std::string model_end = "</model></sdf>";

        std::vector<std::string> links;
        for (int i = 0; i < (int)(distance / (dash_solid + dash_gap)); i++)
        {
            std::string plane_start = fmt::format("<link name='{}'> \
                                                    <visual name='visual'>\
                                                        <geometry>\
                                                        <plane>\
                                                        <normal>0 0 1</normal>\
                                                        <size>{} {}</size>\
                                                        </plane>\
                                                    </geometry>\
                                                    <material>\
                                                        <ambient>0.7 0.7 0.7 1</ambient>\
                                                        <diffuse>0.7 0.7 0.7 1</diffuse>\
                                                        <specular>0 0 0 0</specular>\
                                                        <emissive>0 0 0 1</emissive>\
                                                    </material>\
                                                    </visual>", "plane"+i, dash_solid, line_width);
            float x = (i * (dash_solid + dash_gap)) - dash_solid / 2;
            float y = 0;
            float z = 0;
            float r_x = 0;
            float r_y = 0;
            float r_z = 0;
            
            std::string plane_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", x, y, z, r_x, r_y, r_z);
            std::string plane_end = "</link>";

            links.push_back(plane_start + plane_pose + plane_end);

        }
        
        std::string sdf = model_start + model_pose;

        for (int i = 0; i < links.size(); i++)
        {
            sdf += links[i];
        }
        sdf += model_end;

        return sdf;
    }

    float dash_solid = 1.0;
    float dash_gap = 1.0;
private:

};
