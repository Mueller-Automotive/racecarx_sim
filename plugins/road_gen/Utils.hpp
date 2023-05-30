
#pragma once

#include <string>
#include <iostream>

unsigned int road_id = 0;

std::string getId()
{
    road_id++;
    return "road-element-" + std::to_string(road_id);
}