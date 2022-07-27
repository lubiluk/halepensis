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

/*  */
#include <set>
#include "understanding/task_rule.hpp"
#include <chrono>
#include "geometry.hpp"
/*  */


int main(int argc, const char *argv[])
{
    using namespace std;
    
    auto demo_count = (argc - 1) / 2;
    
    vector<task_understanding> understandings;
    
    for (int i = 0; i < demo_count; ++i) {
        auto cloud_before = load_cloud(argv[1 + i * 2]);
        if (!cloud_before)
        {
            std::cout << "Could not open file: " << argv[1 + i * 2] << std::endl;
            return -1;
        }
        
        auto cloud_after = load_cloud(argv[2 + i * 2]);
        if (!cloud_after)
        {
            std::cout << "Could not open file: " << argv[2 + i * 2] << std::endl;
            return -1;
        }
        
//        view(cloud_before, cloud_after);
        
//        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        
        /* Remove far points */
        cloud_before = filter_field(cloud_before, "z", -1.0, 0.0);
        cloud_after = filter_field(cloud_after, "z", -1.0, 0.0);
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
            1, 0, 0, 0.165205,
            0, 1, 0, 0.062942,
            0, 0, 1, 0.036419,
            0, 0, 0, 1;
            obj_transforms["object_1"] = transform_0;
            
            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_0"] = transform_1;
        } else {
            mat44 transform_0;
            transform_0 <<
            0.9829, 0.0000, -0.1842, 0.0841,
            0.0000, 1.0000,  0.0000, 0.0650,
            0.1842, 0.0000,  0.9829, 0.0493,
            0.0000, 0.0000,  0.0000, 1.0000;
            obj_transforms["object_1"] = transform_0;
            
            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_0"] = transform_1;
        }
        
        task_understanding task { cloud_before, cloud_after };
        task.detect_objects(obj_transforms);
//        view_clusters(task.before_scene.cloud, task.before_scene.object_clouds());
        task.detect_change();
        task.detect_features();
        task.describe_relations();

        view_scenes(task);
        // There is a bug that prevents us from showing graphs side by side...
        view(task.before_scene.graph);
        view(task.after_scene.graph);

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
