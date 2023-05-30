#pragma once

#include <plugins/road_gen/Line.hpp>

class DashedLine : public Line
{
public:
    DashedLine(std::string _name, gz::math::Vector4<float> _point1, gz::math::Vector4<float> _point2, gz::math::Vector4<float> _pos, gz::math::Vector4<float> _rot)
    : Line(_name, _point1, _point2, _pos, _rot) {}

    DashedLine(std::string _name, gz::math::Vector4<float> _point1, gz::math::Vector4<float> _point2, gz::math::Vector4<float> _pos, gz::math::Vector4<float> _rot, float _angle)
    : Line(_name, _point1, _point2, _pos, _rot, _angle) {}

    virtual std::string getSdfStraight()
    {
        float distance = point1.Distance(point2);

        std::string model_start = fmt::format("<?xml version='1.0'?><sdf version='1.7'><model name='{}'><static>true</static><self_collide>false</self_collide>", name);
        std::string model_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]);
        std::string model_end = "</model></sdf>";

        std::vector<std::string> links;
        for (int i = 0; i < (int)(distance / (dash_solid + dash_gap)); i++)
        {
            std::string link_start = fmt::format("<link name='{}'> \
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
                                                    </visual>", "plane" + std::to_string(i), dash_solid, line_width);
            float x = (i * (dash_solid + dash_gap)) - dash_solid / 2;
            float y = 0;
            float z = 0;
            float r_x = 0;
            float r_y = 0;
            float r_z = 0;
            
            std::string link_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", x + point1[0], y + point1[1], z + point2[2], r_x, r_y, r_z);
            std::string link_end = "</link>";

            links.push_back(link_start + link_pose + link_end);

        }
        
        std::string sdf = model_start + model_pose;

        for (int i = 0; i < links.size(); i++)
        {
            sdf += links[i];
        }
        sdf += model_end;

        return sdf;
    }

    std::string getSdfCurved()
    {
        std::string model_start = fmt::format("<?xml version='1.0'?><sdf version='1.7'><model name='{}'><static>true</static><self_collide>false</self_collide>", name);
        std::string model_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]);
        std::string model_end = "</model></sdf>";

        int segments = 10;

        std::vector<gz::math::Vector4<float>> points = getBezierPoints(point1, point2, angle, segments);
        std::vector<std::string> links;

        // Iterate over points.size() - 1 because there's 1 more point than there are lines (a line consists of 2 points)
        for (int i = 0; i < points.size() - 1; i++)
        {
            gz::math::Vector4<float> center = (points[i+1] + points[i]) / 2;

            gz::math::Vector4<float> diff = points[i+1] - points[i];
            float rot_z = std::atan(diff[1] / diff[0]);

            std::cout << "Center " << i << " :" << center << std::endl;

            std::string link_start = fmt::format("<link name='{}'> \
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
                                                </visual>", "link"+std::to_string(i), points[i].Distance(points[i+1]) / 2, line_width);

            std::string link_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", center[0], center[1], center[2], 0, 0, rot_z);
            std::string link_end = "</link>";

            links.push_back(link_start + link_pose + link_end);
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
