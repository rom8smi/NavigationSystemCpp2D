#ifndef SCENE_HPP
#define SCENE_HPP

#include "rts/math/float3.hpp"
#include <vector>

namespace NavigationSystemCode
{
    struct Scene
    {
        Float3 camera_position;
        Float3 camera_rotation;
        bool perspective_camera;
        float camera_size;
        float camera_fov;
        bool debug_launch_menu_visible;
        bool exit_to_launch_menu_visible;
        bool top_bar_resources_visible;
        bool lock_camera_motion;
        bool lock_units_selection;
        float max_search_distance;
        bool run_continuous_spawn;
        bool run_continuous_spawn_for_player_nation;
        bool show_terrain;
        bool show_trees;
        bool run_navigation_tests;
    };
}

#endif
