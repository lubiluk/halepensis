#include "io.hpp"
#include "filtering.hpp"
#include "normals.hpp"
#include "ransac.hpp"
#include "visualization.hpp"
#include "task_understanding.hpp"
#include "scene_visualization.hpp"
#include "graph_visualization.hpp"

#include <iostream>
#include <algorithm>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

/*  */
#include <set>
#include "understanding/task_rule.hpp"
#include <chrono>
#include "geometry.hpp"
/*  */

namespace fs = boost::filesystem;

int main(int argc, const char *argv[])
{
    using namespace std;
    using namespace boost::algorithm;
    
    auto dir_path = argv[1];
    fs::directory_iterator end_itr;
    
    vector<fs::path> paths;
    
    for ( fs::directory_iterator itr( dir_path ); itr != end_itr; ++itr ) {
        if (!fs::is_regular_file(*itr)) {
            continue;
        }
        
        if (ends_with(itr->path().leaf().string(), ".pcd")) {
            paths.push_back(*itr);
        }
    }
    
    sort(paths.begin(), paths.end());
    
    for (auto it = paths.begin(); it != paths.end(); ++it) {
        cout << it->string() << endl;
    }
    
    auto demo_count = paths.size() / 2;
    
    vector<task_understanding> understandings;
    
    
    for (int i = 0; i < demo_count; ++i) {
        auto cloud_before = load_cloud(paths[i * 2].string());
        if (!cloud_before) {
            std::cout << "Could not open file: " << argv[1 + i * 2] << std::endl;
            return -1;
        }
        
        auto cloud_after = load_cloud(paths[1 + i * 2].string());
        if (!cloud_after) {
            std::cout << "Could not open file: " << argv[2 + i * 2] << std::endl;
            return -1;
        }
        
//        view(cloud_before, cloud_after);
        
//        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        
        /* Remove far points */
        cloud_before = filter_field(cloud_before, "z", -1.0, -0.63);
        cloud_after = filter_field(cloud_after, "z", -1.0, -0.63);
//        view(cloud_before, cloud_after);
        
        cloud_before = remove_outliers(cloud_before);
        cloud_after = remove_outliers(cloud_after);
        cloud_before = downsample(cloud_before);
        cloud_after = downsample(cloud_after);

        compute_normals(cloud_before);
        compute_normals(cloud_after);
//        view(cloud_before, cloud_after);

        /* Remove table */
        auto indics = fit_plane(cloud_before, 0.015);
        cloud_before = extract_cloud(cloud_before, std::get<0>(indics.value()), true);
        indics = fit_plane(cloud_after);
        cloud_after = extract_cloud(cloud_after, std::get<0>(indics.value()), true);

//        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        
//        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        
        view(cloud_before, cloud_after);
        
        /* Task Reasoning Part */
        map<string, mat44> obj_transforms;
        
        if (i == 0) {
            mat44 transform_0;
            transform_0 <<
            1.0, 0.0, 0.0, 0.19954301416873932,
            0.0, 1.0, 0.0, 0.08365362882614136,
            0.0, 0.0, 1.0, 0.04248148202896118,
            0.0, 0.0, 0.0, 1.0;
            obj_transforms["object_1"] = transform_0;

            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_0"] = transform_1;
        } else if (i == 1) {
            mat44 transform_0;
            transform_0 <<
            0.9898858070373535, 0.0, 0.14186647534370422, -0.1073008701205253,
            0.0, 1.0, 0.0, 0.07755453884601593,
            -0.14186647534370422, 0.0, 0.9898858070373535, 0.028391443192958832,
            0.0, 0.0, 0.0, 1.0;
            obj_transforms["object_0"] = transform_0;

            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_1"] = transform_1;
        } else if (i == 2) {
            mat44 transform_0;
            transform_0 <<
            1.0, 0.0, 0.0, 0.20187997817993164,
            0.0, 1.0, 0.0, 0.07851298153400421,
            0.0, 0.0, 1.0, 0.07304024696350098,
            0.0, 0.0, 0.0, 1.0;
            obj_transforms["object_1"] = transform_0;

            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_0"] = transform_1;
        } else if (i == 3) {
            mat44 transform_0;
            transform_0 <<
            1.0, 0.0, 0.0, -0.18519198894500732,
            0.0, 1.0, 0.0, 0.08033152669668198,
            0.0, 0.0, 1.0, 0.018137631937861443,
            0.0, 0.0, 0.0, 1.0;
            obj_transforms["object_0"] = transform_0;

            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_1"] = transform_1;
        } else if (i == 4) {
            mat44 transform_0;
            transform_0 <<
            1.0, 0.0, 0.0, 0.2256317138671875,
            0.0, 1.0, 0.0, 0.10619379580020905,
            0.0, 0.0, 1.0, -0.019369959831237793,
            0.0, 0.0, 0.0, 1.0;
            obj_transforms["object_1"] = transform_0;

            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_0"] = transform_1;
        }
        else if (i == 5) {
           mat44 transform_0;
           transform_0 <<
            1.0, 0.0, 0.0, -0.26169833540916443,
            0.0, 1.0, 0.0, 0.08043941855430603,
            0.0, 0.0, 1.0, 0.05333158001303673,
            0.0, 0.0, 0.0, 1.0;
           obj_transforms["object_0"] = transform_0;

           mat44 transform_1;
           transform_1 <<
           1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
           obj_transforms["object_1"] = transform_1;
       }
        
        task_understanding task { cloud_before, cloud_after };
        task.detect_objects(obj_transforms);
//        view_clusters(task.before_scene.cloud, task.before_scene.object_clouds());
        task.detect_change();
        task.detect_features();
        task.describe_relations();

        view_scenes(task);
        // There is a bug that prevents us from showing graphs side by side...
//        view(task.before_scene.graph);
//        view(task.after_scene.graph);

        task.describe_task();
        view(task.task_description);
        
        understandings.push_back(task);
    }
    
    auto graph1 = understandings.front().task_description;
    save_to_graphviz(graph1, "/Users/lubiluk/Code/halepensis/graph_task_before.dot");
    auto graph2 = understandings.back().task_description;
    save_to_graphviz(graph2, "/Users/lubiluk/Code/halepensis/graph_task_after.dot");
    
    
    // Skill inference
    set<task_rule> before_rules = rules_from_graph(graph1, false);
    set<task_rule> after_rules = rules_from_graph(graph2, false);
    
    set<task_rule> skill_rules;
    set_intersection(after_rules.begin(), after_rules.end(),
                   before_rules.begin(), before_rules.end(),
                   inserter(skill_rules, skill_rules.begin()));
    
    cout << "Skill rules:" << endl;
    for (auto &r : skill_rules)  {
        cout << r.object1_id << " - " << r.feature1_id
        << " --" << r.relation_type << "--> "
        << r.object2_id << " - " << r.feature2_id << endl;
    }

    auto skill_graph = graph_from_rules(skill_rules);
    save_to_graphviz(skill_graph, "/Users/lubiluk/Code/halepensis/graph_skill.dot");
    
    return 0;
}
