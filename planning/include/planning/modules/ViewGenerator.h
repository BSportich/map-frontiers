#pragma once

//#include <String>

struct Frontier
{
    //position
    int x;
    int y;
    int z;

    //orientation
    int q_w ;
    int q_x;
    int q_y;
    int q_z;

};

class ViewGenerator
{
private:
    /* data */
public:
    ViewGenerator(/* args */);
    ~ViewGenerator();

    //std::string method_type;
    
};

ViewGenerator::ViewGenerator(/* args */)
{
}

ViewGenerator::~ViewGenerator()
{
}

#endif