#pragma once

#include <plugins/road_gen/Line.hpp>

class DashedLine : public Line
{
public:
    DashedLine(std::string _name, gz::math::Vector3<float> _point1, gz::math::Vector3<float> _point2)
    : Line(_name, _point1, _point2) {}

    DashedLine(std::string _name, gz::math::Vector3<float> _point1, gz::math::Vector3<float> _point2, float _angle)
    : Line(_name, _point1, _point2, _angle) {}

    virtual std::string getSdfStraight()
    {
        float distance = point1.Distance(point2);
        gz::math::Vector3<float> direction = (point2 - point1).Normalized();

        gz::math::Vector3<float> diff = point2 - point1;
        float rot_z = std::atan(diff[1] / diff[0]);

        std::string model_start = fmt::format("<model name='{}'><static>true</static><self_collide>false</self_collide>", name);
        std::string model_pose = "<pose>0 0 0 0 0 0</pose>";
        std::string model_end = "</model>";

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
            gz::math::Vector3<float> new_pos = point1 + (direction * ((i * (dash_solid + dash_gap)) + dash_solid / 2));
            
            std::string link_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", new_pos[0], new_pos[1], new_pos[2], 0, 0, rot_z);
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
        std::string model_start = fmt::format("<model name='{}'><static>true</static><self_collide>false</self_collide>", name);
        std::string model_pose = "<pose>0 0 0 0 0 0</pose>";
        std::string model_end = "</model>";

        int segments = 10;

        std::vector<gz::math::Vector3<float>> points = getBezierPoints(point1, point2, angle, segments);
        std::vector<std::string> links;

        // Iterate over points.size() - 1 because there's 1 more point than there are lines (a line consists of 2 points)
        for (int i = 0; i < points.size() - 1; i++)
        {
            gz::math::Vector3<float> center = (points[i+1] + points[i]) / 2;

            gz::math::Vector3<float> diff = points[i+1] - points[i];
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

    float dash_solid = 0.5;
    float dash_gap = 0.5;
private:

};
