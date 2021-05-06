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
        auto obj1 = Object({
            "/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_0.pcd",
            "/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_1.pcd",
            "/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_2.pcd",
            "/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_3.pcd",
            "/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_4.pcd",
            "/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_5.pcd",
            "/Users/lubiluk/Code/halepensis/data/cutting_board_scan/cutting_board_scan_6.pcd"
        });
        auto obj2 = Object({
//            "/Users/lubiluk/Code/halepensis/data/hook_scan/hook_scan_0.pcd",
            "/Users/lubiluk/Code/halepensis/data/hook_scan/hook_scan_1.pcd",
//            "/Users/lubiluk/Code/halepensis/data/hook_scan/hook_scan_2.pcd",
//            "/Users/lubiluk/Code/halepensis/data/hook_scan/hook_scan_3.pcd"
        });
        const auto source_scene = Scene("/Users/lubiluk/Code/halepensis/data/cutting_board.pcd");
        const auto target_scene = Scene("/Users/lubiluk/Code/halepensis/data/cutting_board_on_hook.pcd");
        
        obj1.findParts();

        Viewer::view(obj1.parts.front().cloud);
//        Viewer::view(obj2.getPointClouds(), true);
        
//        const auto source_obj1 = source_scene.findObject(obj1);
//        const auto source_obj2 = source_scene.findObject(obj2);
//        
//        Viewer::view({source_scene.getPointCloud(), source_obj1, source_obj2});
        
//        const auto target_obj1 = source_scene.findObject(obj1);
//        const auto target_obj2 = source_scene.findObject(obj2);
//
//        Viewer::view({target_scene.getPointCloud(), target_obj1, target_obj2});
    }
    catch (Error e)
    {
        std::cout << e.message << std::endl;
    }

    return 0;
}
