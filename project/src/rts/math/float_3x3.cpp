#include "float_3x3.hpp"
#include <cmath>

using namespace std;

namespace NavigationSystemCode
{
    Float3x3::Float3x3()
    {
        xx = 1.0f;
        xy = 0.0f;
        xz = 0.0f;

        yx = 0.0f;
        yy = 1.0f;
        yz = 0.0f;

        zx = 0.0f;
        zy = 0.0f;
        zz = 1.0f;
    }

    Float3x3::Float3x3(
        float p_xx,
        float p_xy,
        float p_xz,

        float p_yx,
        float p_yy,
        float p_yz,

        float p_zx,
        float p_zy,
        float p_zz)
    {
        xx = p_xx;
        xy = p_xy;
        xz = p_xz;

        yx = p_yx;
        yy = p_yy;
        yz = p_yz;

        zx = p_zx;
        zy = p_zy;
        zz = p_zz;
    }

    void Float3x3::update_looking_at_safe(Float3 &p_target, Float3 &p_up)
    {
        if ((p_target - p_up).length_squared() < 0.01f || (p_target + p_up).length_squared() < 0.01f)
        {
            p_target += Float3(0.0f, 0.0f, 0.01f);
            p_target.normalize();
        }
    }

    Float3x3 Float3x3::looking_at(Float3 &p_target, Float3 &p_up)
    {
        Float3 v_z = -p_target.normalized();
        Float3 v_x = p_up.cross(v_z);
        v_x.normalize();
        Float3 v_y = v_z.cross(v_x);

        return Float3x3(
            v_x.x,
            v_y.x,
            v_z.x,

            v_x.y,
            v_y.y,
            v_z.y,

            v_x.z,
            v_y.z,
            v_z.z);
    }

    void Float3x3::set_euler(const Float3 &p_euler)
    {
        float c = cos(p_euler.x);
        float s = sin(p_euler.x);
        Float3x3 xmat(1, 0, 0, 0, c, -s, 0, s, c);

        c = cos(p_euler.y);
        s = sin(p_euler.y);
        Float3x3 ymat(c, 0, s, 0, 1, 0, -s, 0, c);

        c = cos(p_euler.z);
        s = sin(p_euler.z);
        Float3x3 zmat(c, -s, 0, s, c, 0, 0, 0, 1);

        *this = ymat * xmat * zmat;
    }

    Float3x3 Float3x3::rotated(const Float3 &p_euler, Float3x3 &original)
    {
        Float3x3 rotated_basis;
        rotated_basis.set_euler(p_euler);
        return rotated_basis * original;
    }

    void Float3x3::set_scale(const Float3 &p_scale)
    {
        xx = p_scale.x;
        yy = p_scale.y;
        zz = p_scale.z;
    }

    Float3x3 Float3x3::from_scale(const Float3 &p_scale)
    {
        return Float3x3(p_scale.x, 0, 0, 0, p_scale.y, 0, 0, 0, p_scale.z);
    }

    void Float3x3::scale(const Float3 &p_scale)
    {
        xx *= p_scale.x;
        xy *= p_scale.x;
        xz *= p_scale.x;
        yx *= p_scale.y;
        yy *= p_scale.y;
        yz *= p_scale.y;
        zx *= p_scale.z;
        zy *= p_scale.z;
        zz *= p_scale.z;
    }

    Float3x3 Float3x3::operator*(Float3x3 &p_matrix)
    {
        Float3 rowx = Float3(xx, xy, xz);
        Float3 rowy = Float3(yx, yy, yz);
        Float3 rowz = Float3(zx, zy, zz);

        return Float3x3(
            p_matrix.tdotx(rowx), p_matrix.tdoty(rowx), p_matrix.tdotz(rowx),
            p_matrix.tdotx(rowy), p_matrix.tdoty(rowy), p_matrix.tdotz(rowy),
            p_matrix.tdotx(rowz), p_matrix.tdoty(rowz), p_matrix.tdotz(rowz));
    }

    // transposed dot products
    float Float3x3::tdotx(Float3 &v)
    {
        return xx * v.x + yx * v.y + zx * v.z;
    }
    float Float3x3::tdoty(Float3 &v)
    {
        return xy * v.x + yy * v.y + zy * v.z;
    }
    float Float3x3::tdotz(Float3 &v)
    {
        return xz * v.x + yz * v.y + zz * v.z;
    }

    string Float3x3::get_string()
    {
        Float3 colx = Float3(xx, yx, zx);
        Float3 coly = Float3(xy, yy, zy);
        Float3 colz = Float3(xz, yz, zz);

        return "[X: " + colx.get_string() + ", Y: " + coly.get_string() + ", Z: " + colz.get_string() + "]";
    }
}
