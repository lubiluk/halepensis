#include "types.hpp"
#include "utils.hpp"
#include "object.hpp"
#include "viewer.hpp"
#include "error.hpp"

int main(int argc, char **argv)
{
    try
    {
        auto obj = Object("/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_0.pcd");
        auto scene = Scene("/Users/lubiluk/Code/halepensis/data/cutting_board_on_hook.pcd");

        Viewer::view(obj);
        Viewer::view(scene);
        Viewer::view(scene.findObject(obj));
    }
    catch (Error e)
    {
        std::cout << e.message << std::endl;
    }

    return 0;
}