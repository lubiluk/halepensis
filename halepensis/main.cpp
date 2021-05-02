#include "types.hpp"
#include "utils.hpp"
#include "object.hpp"
#include "viewer.hpp"
#include "error.hpp"

#include <iostream>

int main(int argc, const char *argv[])
{
    try
    {
        auto obj = Object("/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_0.pcd");
        auto scene = Scene("/Users/lubiluk/Code/halepensis/data/cutting_board_on_hook.pcd");

        Viewer::view(obj);
//        Viewer::view({obj, scene, scene.findObject(obj)});
    }
    catch (Error e)
    {
        std::cout << e.message << std::endl;
    }

    return 0;
}
