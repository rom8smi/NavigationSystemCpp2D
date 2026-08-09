#ifndef SCENE_SYSTEM_HPP
#define SCENE_SYSTEM_HPP

#include "scene.hpp"
#include "navigation_tests_scene_loader.hpp"

namespace NavigationSystemCode
{
    struct SceneSystem
    {
        Scene scene;
        NavigationTestsSceneLoader navigation_tests_scene_loader;

        void load_navigation_tests_scene();
    };
}

#endif
