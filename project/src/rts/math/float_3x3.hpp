#ifndef FLOAT_3X3_HPP
#define FLOAT_3X3_HPP

#include "float3.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct Float3x3
    {
        float xx;
        float xy;
        float xz;

        float yx;
        float yy;
        float yz;

        float zx;
        float zy;
        float zz;

        Float3x3();
        Float3x3(
            float p_xx,
            float p_xy,
            float p_xz,

            float p_yx,
            float p_yy,
            float p_yz,

            float p_zx,
            float p_zy,
            float p_zz);

        static void update_looking_at_safe(Float3 &p_target, Float3 &p_up);
        static Float3x3 looking_at(Float3 &p_target, Float3 &p_up);
        void set_euler(const Float3 &p_euler);
        static Float3x3 rotated(const Float3 &p_euler, Float3x3 &original);
        void set_scale(const Float3 &p_euler);
        static Float3x3 from_scale(const Float3 &p_scale);
        void scale(const Float3 &p_scale);

        Float3x3 operator*(Float3x3 &p_matrix);

        float tdotx(Float3 &v);
        float tdoty(Float3 &v);
        float tdotz(Float3 &v);

        string get_string();
    };
}

#endif
