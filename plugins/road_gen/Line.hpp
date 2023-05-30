#pragma once

#include <string>
#include <cmath>
#include <iostream>

#include <gz/math/Vector4.hh>

#define FMT_HEADER_ONLY
#include <include/fmt/format.h>

class Line
{
public:
    std::string name;
    gz::math::Vector4<float> point1;
    gz::math::Vector4<float> point2;
    gz::math::Vector4<float> pos;
    gz::math::Vector4<float> rot;
    float angle;

    float line_width = 0.1;

    Line(std::string _name, gz::math::Vector4<float> _point1, gz::math::Vector4<float> _point2, gz::math::Vector4<float> _pos, gz::math::Vector4<float> _rot, float _angle)
    {
        name = _name;
        point1 = _point1;
        point2 = _point2;
        pos = _pos;
        rot = _rot;
        angle = _angle;
    }

    Line(std::string _name, gz::math::Vector4<float> _point1, gz::math::Vector4<float> _point2, gz::math::Vector4<float> _pos, gz::math::Vector4<float> _rot)
    {
        name = _name;
        point1 = _point1;
        point2 = _point2;
        pos = _pos;
        rot = _rot;
        angle = 0;
    }

    virtual std::string getSdf()
    {
        if (angle == 0)
        {
            return getSdfStraight();
        }
        else
        {
            return getSdfCurved();
        }
    }

    virtual std::string getSdfStraight()
    {
        gz::math::Vector4<float> center = (point2 + point1) / 2;
        float distance = point1.Distance(point2);

        gz::math::Vector4<float> diff = point2 - point1;
        float rot_z = std::atan(diff[1] / diff[0]);

        std::string model_start = fmt::format("<?xml version='1.0'?><sdf version='1.7'><model name='{}'><static>true</static><self_collide>false</self_collide>", name);
        std::string model_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]);
        std::string model_end = "</model></sdf>";

        std::string link_start = fmt::format("<link name='link'> \
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
                                                </visual>", distance, line_width);
        
        std::string link_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", center[0], center[1], center[2], 0, 0, rot_z);
        std::string link_end = "</link>";

        std::string sdf = model_start + model_pose + link_start + link_pose + link_end + model_end;
        return sdf;
    }

    virtual std::string getSdfCurved()
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
            float rot = std::atan(diff[1] / diff[0]);

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
                                                </visual>", "link"+std::to_string(i), points[i].Distance(points[i+1]), line_width);

            std::string link_pose = fmt::format("<pose>{} {} {} {} {} {}</pose>", center[0], center[1], center[2], 0, 0, rot);
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

protected:
    // Create a discrete polyline between two points at a given angle
    // by creating a quadratic Bezier curve
    std::vector<gz::math::Vector4<float>> getBezierPoints(gz::math::Vector4<float> p1, gz::math::Vector4<float> p2, float angle, int segments)
    {
        std::vector<gz::math::Vector4<float>> points;

        // We need to create a third control point to "curve" our bezier curve
        // The third control point is the apex of an isosceles triangle, where p1
        // is one base point and p2 is the other

        float b = p1.Distance(p2); // base
        float a = b / (2 * std::sin(degreesToRadians( (180-std::abs(angle))/2 ))); // equal sides

        std::cout << "a: " << a << std::endl;   
        std::cout << "b: " << b << std::endl;
        
        // Calculate the height of the triangle
        float h = std::sqrt(std::pow(a, 2) - (std::pow(b, 2) / 4));

        // Multiply it by the sign of the rotation angle (to make the line curve left or right)
        h *= angle / std::abs(angle);

        std::cout << "h: " << h << std::endl;

        gz::math::Vector4<float> midpoint = (p1 + p2) / 2;
        gz::math::Vector4<float> u = (p2 - midpoint).Normalized();
        gz::math::Vector4<float> controlPoint;
        controlPoint[0] = -u[1];
        controlPoint[1] = u[0];

        controlPoint = midpoint + controlPoint*h;
        // Control point is now ready

        std::cout << "u: " << u << std::endl;
        std::cout << "midpoint: " << midpoint << std::endl;
        std::cout << "control point: " << controlPoint << std::endl;

        for (int i = 0; i < segments + 1; i++)
        {
            float perc = (float)i / segments;
            float xa = getPt(p1[0], controlPoint[0], perc);
            float ya = getPt(p1[1], controlPoint[1], perc);
            float xb = getPt(controlPoint[0], p2[0], perc);
            float yb = getPt(controlPoint[1], p2[1], perc);
            
            float x = getPt(xa, xb, perc);
            float y = getPt(ya, yb, perc);

            std::cout << "X" << i << " :" << x << std::endl;
            std::cout << "Y" << i << " :" << x << std::endl;
            std::cout << "Perc: " << perc << std::endl;

            points.push_back(gz::math::Vector4<float>(x, y, p1[2], 1.0));
        }

        return points;
    }

    float getPt(float n1, float n2, float perc)
    {
        float diff = n2 - n1;

        return n1 + (diff * perc);
    }

    float degreesToRadians(float degrees) {
        return degrees * M_PI / 180.0;
    }
};
