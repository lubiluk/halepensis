#include "entity.hpp"
#include "entity_viewing.hpp"
#include "error.hpp"

#include <iostream>

int main(int argc, const char *argv[])
{
    try
    {
        entity obj1{"/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_0.pcd"};
        view_entity(obj1);
    }
    catch (error e)
    {
        std::cout << e.message << std::endl;
    }

    return 0;
}
