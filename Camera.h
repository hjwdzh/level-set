//
//  Camera.h
//  simulation
//
//  Created by skyer on 14-4-14.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#ifndef __simulation__Camera__
#define __simulation__Camera__

#include <iostream>
#include "vmath.h"

class Camera
{
public:
    Camera();
    void Move(const double dis);
    void Rotate(const int mode, const double ang);
    void Render();
    Matrix4d lookat;
private:
    Vector3d o, dir, up;
};
#endif /* defined(__simulation__Camera__) */
