#include "rgba.hpp"

namespace NavigationSystemCode
{
    Rgba::Rgba()
    {
        r = 0.0f;
        g = 0.0f;
        b = 0.0f;
        a = 0.0f;
    }

    Rgba::Rgba(float p_r, float p_g, float p_b, float p_a)
    {
        r = p_r;
        g = p_g;
        b = p_b;
        a = p_a;
    }

    Rgba Rgba::operator+(const Rgba &p_color) const
    {
        return Rgba(
            r + p_color.r,
            g + p_color.g,
            b + p_color.b,
            a + p_color.a);
    }

    Rgba Rgba::operator*(float p_scalar) const
    {
        return Rgba(
            r * p_scalar,
            g * p_scalar,
            b * p_scalar,
            a * p_scalar);
    }

    string Rgba::get_string()
    {
        return "(" + to_string(r) + ", " + to_string(g) + ", " + to_string(b) + ", " + to_string(a) + ")";
    }
}
