//
//  Camera.cpp
//  simulation
//
//  Created by skyer on 14-4-14.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Camera.h"
#include "main.h"


#define PI 3.141592654
Camera::Camera()
    : o(Vector3d(0, 40, -40)),dir(Vector3d(0, 0, 1)), up(Vector3d(0, 1, 1))
{
    up.normalize();
    dir = up.crossProduct(Vector3d(-1, 0, 0));
}

void Camera::Move(const double dis)
{
    o += dir * dis;
}

void Camera::Rotate(const int mode, const double ang)
{
    if (mode == 0)
    {
        Vector3d v(dir.crossProduct(up) * tan(ang / 180.0 * PI));
        dir += v;
        dir.normalize();
    }
    else
    if (mode == 1)
    {
        up += dir * tan(ang / 180.0 * PI);
        up.normalize();
        dir = up.crossProduct(dir.crossProduct(up));
    }
    else
    {
        Vector3d v(dir.crossProduct(up) * tan(ang / 180.0 * PI));
        up += v;
        up.normalize();
    }
}

void Camera::Render()
{
    Vector3d v(o + dir);
    gluLookAt(o[0], o[1], o[2], v[0], v[1], v[2], up[0], up[1], up[2]);
    lookat = Matrix4d::createLookAt(o, v, up);
}
