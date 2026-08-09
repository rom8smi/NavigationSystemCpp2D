#include "navigation_tests_scene_loader.hpp"
#include "rts/math/angle_utils.hpp"

namespace NavigationSystemCode
{
    void NavigationTestsSceneLoader::setup(Scene &scene)
    {
        scene.camera_position = Float3(0.0f, 1000.0f, 0.0f);
        scene.camera_rotation = Float3(AngleUtils::degrees_to_radians(-90.0f), AngleUtils::degrees_to_radians(0.0f), 0.0f);
        scene.perspective_camera = false;
        scene.camera_size = 100.0f;
        scene.camera_fov = 60.0f;

        scene.debug_launch_menu_visible = false;
        scene.exit_to_launch_menu_visible = false;
        scene.top_bar_resources_visible = false;
        scene.lock_camera_motion = true;
        scene.lock_units_selection = true;
        scene.max_search_distance = 70.0f;
        scene.run_continuous_spawn = false;
        scene.run_continuous_spawn_for_player_nation = false;

        scene.show_terrain = false;
        scene.show_trees = false;

        scene.run_navigation_tests = true;
    }
}
