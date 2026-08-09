#ifndef RGBA_HPP
#define RGBA_HPP

#include <string>

using namespace std;

namespace NavigationSystemCode
{
    struct Rgba
	{
        float r;
		float g;
		float b;
        float a;

        Rgba();
        Rgba(float p_r, float p_g, float p_b, float p_a);
        Rgba operator+(const Rgba &p_color) const;
        Rgba operator*(float p_scalar) const;
        string get_string();
    };
}

#endif
