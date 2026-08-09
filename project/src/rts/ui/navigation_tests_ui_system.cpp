#include "navigation_tests_ui_system.hpp"
#include "ui_button_press_mode.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"
#include "rts/math/rgba_presets.hpp"
#include "rts/navigation/navigation_tests/navigation_tests_scene_indices.hpp"
#include "rts/math/math_utils.hpp"

namespace NavigationSystemCode
{
    void NavigationTestsUISystem::create(
        GodotControlWrapper &godot_control,
        GodotScreenSystem &godot_screen_system)
    {
        set_button_and_scene_references();
        create_buttons(godot_control, godot_screen_system);

        last_window_width = godot_screen_system.window_width;
        last_window_height = godot_screen_system.window_height;
    }

    void NavigationTestsUISystem::set_button_and_scene_references()
    {
        button_names.push_back("BlockedDestination");
        button_names.push_back("BlockedDestinationWithBridge");
        button_names.push_back("CentralPassage");
        button_names.push_back("ChainedObstacles");
        button_names.push_back("Circles");
        button_names.push_back("RectangleSpawners");
        button_names.push_back("RectangleSpawnersWithObstacles");
        button_names.push_back("RandomDynamicObstacles");
        button_names.push_back("RandomObstacles");
        button_names.push_back("RandomRotatedObstacles");
        button_names.push_back("Spiral");

        int blocked_destination_index = NavigationTestsSceneIndices::BLOCKED_DESTINATION;
        int blocked_destination_with_bridge_index = NavigationTestsSceneIndices::BLOCKED_DESTINATION_WITH_BRIDGE;
        int central_passage_index = NavigationTestsSceneIndices::CENTRAL_PASSAGE;
        int chained_obstacles_index = NavigationTestsSceneIndices::CHAINED_OBSTACLES;
        int circles_index = NavigationTestsSceneIndices::CIRCLES;
        int rectangle_spawners_index = NavigationTestsSceneIndices::RECTANGLE_SPAWNERS;
        int rectangle_spawners_with_obstacles_index = NavigationTestsSceneIndices::RECTANGLE_SPAWNERS_WITH_OBSTACLES;
        int random_dynamic_obstacles_index = NavigationTestsSceneIndices::RANDOM_DYNAMIC_OBSTACLES;
        int random_obstacles_index = NavigationTestsSceneIndices::RANDOM_OBSTACLES;
        int random_rotated_obstacles_index = NavigationTestsSceneIndices::RANDOM_ROTATED_OBSTACLES;
        int spiral_index = NavigationTestsSceneIndices::SPIRAL;

        scene_indices.push_back(blocked_destination_index);
        scene_indices.push_back(blocked_destination_with_bridge_index);
        scene_indices.push_back(central_passage_index);
        scene_indices.push_back(chained_obstacles_index);
        scene_indices.push_back(circles_index);
        scene_indices.push_back(rectangle_spawners_index);
        scene_indices.push_back(rectangle_spawners_with_obstacles_index);
        scene_indices.push_back(random_dynamic_obstacles_index);
        scene_indices.push_back(random_obstacles_index);
        scene_indices.push_back(random_rotated_obstacles_index);
        scene_indices.push_back(spiral_index);

        number_of_buttons = scene_indices.size();
    }

    void NavigationTestsUISystem::create_buttons(
        GodotControlWrapper &godot_control,
        GodotScreenSystem &godot_screen_system)
    {
        button_colors.create(godot_control, number_of_buttons);
        button_texts.create(godot_control, number_of_buttons);
        buttons.create(godot_control, number_of_buttons);
        update_buttons_size(godot_screen_system);

        Rgba button_color = Rgba(0.8f, 0.8f, 0.8f, 1.0f);
        button_colors.set_color(button_color);

        for (int i = 0; i < button_names.size(); i++)
        {
            string name = "[center]" + button_names[i] + "[/center]";
            button_texts.set_text(name, i);
        }

        button_texts.set_use_text_bbcode(true);

        Rgba text_color = RgbaPresets::black();
        button_texts.set_text_color(text_color);
    }

    void NavigationTestsUISystem::process(
        NavigationTestsSystem &navigation_tests_system,
        GodotScreenSystem &godot_screen_system,
        NavigationSystem &navigation_system,
        GodotWorld &godot_world)
    {
        // GodotUtils::print(godot_screen_system.window_width);
        if (MathUtils::abs(last_window_width - godot_screen_system.window_width) > 0.01f ||
            MathUtils::abs(last_window_height - godot_screen_system.window_height) > 0.01f)
        {
            update_buttons_size(godot_screen_system);
            last_window_width = godot_screen_system.window_width;
            last_window_height = godot_screen_system.window_height;
        }

        int press_mode;
        int index;
        buttons.process(press_mode, index);

        if (index != -1 && press_mode == UIButtonPressMode::PRESSED_DOWN)
        {
            navigation_tests_system.set_scene_to_load(scene_indices[index], navigation_system, godot_world);
        }
    }

    void NavigationTestsUISystem::update_buttons_size(GodotScreenSystem &godot_screen_system)
    {
        float button_width = godot_screen_system.to_absolute_width(0.2f);
        float button_height = godot_screen_system.to_absolute_height(0.04f);
        float button_gap = godot_screen_system.to_absolute_height(0.005f);
        float button_height_with_gap = button_height + button_gap;

        Float2 begin = Float2(
            godot_screen_system.to_absolute_width(1.0f) - button_width,
            godot_screen_system.to_absolute_height(0.5f) - 0.5f * button_height_with_gap * number_of_buttons);
        Float2 size = Float2(button_width, button_height);
        Float2 gap = Float2(0.0f, button_height_with_gap);

        button_colors.set_positions(begin, size, gap);
        button_texts.set_positions(begin, size, gap);
        buttons.set_positions(begin, size, gap);

        button_texts.set_font_size(button_height * 1.5f / 3.0f);
    }

    void NavigationTestsUISystem::set_visible(bool visible, GodotScreenSystem &godot_screen_system)
    {
        button_colors.set_visible(visible);
        button_texts.set_visible(visible);
        buttons.set_visible(visible);

        if (visible)
        {
            update_buttons_size(godot_screen_system);
        }
    }
}
