#include "scene_system.hpp"
#include "rts/math/angle_utils.hpp"
#include "scene_indices.hpp"

namespace NavigationSystemCode
{
    void SceneSystem::load_navigation_tests_scene()
    {
        navigation_tests_scene_loader.setup(scene);
    }
}
