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
        
        cloud_before = remove_outliers(cloud_before);
        cloud_after = remove_outliers(cloud_after);
        cloud_before = downsample(cloud_before);
        cloud_after = downsample(cloud_after);

        compute_normals(cloud_before);
        compute_normals(cloud_after);
//        view(cloud_before, cloud_after);

        /* Remove walls */
        auto indics = fit_plane(cloud_before);
        cloud_before = extract_cloud(cloud_before, std::get<0>(indics.value()), true);
        indics = fit_plane(cloud_after);
        cloud_after = extract_cloud(cloud_after, std::get<0>(indics.value()), true);

//        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        
//        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        
//        view(cloud_before, cloud_after);
        
        /* Task Reasoning Part */
        map<string, mat44> obj_transforms;
        
        if (i == 0) {
            mat44 transform_0;
            transform_0 <<
            0.11037899553775787, 0.9119392037391663, -0.39520034193992615, -0.2506454288959503,
            0.8748316168785095, -0.27785468101501465, -0.39682039618492126, 0.02264728769659996,
            -0.4716843366622925, -0.3019331395626068, -0.8284626007080078, -1.345994472503662,
            0.0, 0.0, 0.0, 1.0;
            obj_transforms["object_0"] = transform_0;

            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_1"] = transform_1;
        } else if (i == 1) {
            mat44 transform_0;
            transform_0 <<
            -0.2400357574224472, -0.8968278169631958, 0.37159478664398193, 0.048634301871061325,
            0.9409595727920532, -0.12082268297672272, 0.3162229359149933, 0.30346277356147766,
            -0.23870044946670532, 0.42556050419807434, 0.8728805184364319, -0.2948179543018341,
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
            -0.4027220904827118, -0.798197865486145, 0.4479900598526001, -0.05327226594090462,
            0.8848711848258972, -0.21431303024291992, 0.4136095643043518, 0.4347284734249115,
            -0.23413217067718506, 0.5629832148551941, 0.7926108837127686, -0.34297868609428406,
            0.0, 0.0, 0.0, 1.0;
            obj_transforms["object_0"] = transform_0;

            mat44 transform_1;
            transform_1 <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
            obj_transforms["object_1"] = transform_1;
        } else if (i == 3) {
            mat44 transform_0;
            transform_0 <<
            0.9960336089134216, -0.07417850196361542, -0.04913868010044098, -0.18010298907756805,
            0.06306114792823792, 0.9781120419502258, -0.19829308986663818, 0.18532726168632507,
            0.06277221441268921, 0.19440783560276031, 0.9789102077484131, -0.23559853434562683,
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
            0.9518234729766846, 0.2696799039840698, -0.1459617167711258, -0.11369828879833221,
            -0.2738206684589386, 0.9617416262626648, -0.008677425794303417, 0.04498139023780823,
            0.13803733885288239, 0.04822671413421631, 0.9892522096633911, -0.20330768823623657,
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

//        view_scenes(task);
        // There is a bug that prevents us from showing graphs side by side...
//        view(task.before_scene.graph);
//        view(task.after_scene.graph);
        
        if (i == 0) {
            task.after_scene.add_relation({"object_1", "peg_3", relation_type::inside, "object_0", "hole_1"});
        } else if (i == 1) {
            task.after_scene.add_relation({"object_1", "peg_1", relation_type::inside, "object_0", "hole_0"});
        } else if (i == 2) {
            task.after_scene.add_relation({"object_1", "peg_3", relation_type::inside, "object_0", "hole_1"});
        } else if (i == 3) {
            task.after_scene.add_relation({"object_1", "peg_1", relation_type::inside, "object_0", "hole_1"});
        } else if (i == 4) {
            task.after_scene.add_relation({"object_1", "peg_1", relation_type::inside, "object_0", "hole_0"});
        } else if (i == 5) {
            task.after_scene.add_relation({"object_1", "peg_0", relation_type::inside, "object_0", "hole_1"});
        }
        
        task.describe_task();
//        view(task.task_description);
        
        understandings.push_back(task);
    }
    
//    auto graph1 = understandings.front().task_description;
//    save_to_graphviz(graph1, "/Users/lubiluk/Code/halepensis/graph_task_before.dot");
//    auto graph2 = understandings.back().task_description;
//    save_to_graphviz(graph2, "/Users/lubiluk/Code/halepensis/graph_task_after.dot");
    
    /* Skill inference */
    
    set<task_rule> skill_rules;
    int i = 0;
    
    for (auto it = understandings.rbegin(); it != understandings.rend(); ++it) {
        auto u = *it;
        auto task_rules = rules_from_graph(u.task_description, false);
        
        if (skill_rules.empty()) {
            skill_rules = task_rules;
        }
        
        set<task_rule> updated_skill_rules;
        set_intersection(skill_rules.begin(), skill_rules.end(),
                         task_rules.begin(), task_rules.end(),
                       inserter(updated_skill_rules, updated_skill_rules.begin()));
        
        skill_rules = updated_skill_rules;
        
        cout << "Skill rules " << i << ": " << skill_rules.size() << endl;
        for (auto &r : skill_rules)  {
            cout << r.object1_id << " - " << r.feature1_id
            << " --" << r.relation_type << "--> "
            << r.object2_id << " - " << r.feature2_id << endl;
        }
        
        i++;
    }
    
    
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
