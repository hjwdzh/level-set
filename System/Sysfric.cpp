#include "main.h"
#include "Sysfric.h"
#include "IMPLICIT_CUBE.h"
#include "IMPLICIT_SPHERE.h"
#include "Rigid_Geometry.h"
#include "PtJoint.h"
#include <stdio.h>

using namespace SimLib;

extern string res_path;

Sysfric::Sysfric()
{
    start_angle = 1;
    shoots = 0;
    supplemental = 0;
}

Sysfric::~Sysfric()
{}

void Sysfric::Initialize()
{
    ks = 100; kd = 10;
    Reset();
}

void Sysfric::RemoveBall() {
    m_objects.removeByName("ball");
}

void Sysfric::AddBall() {
#ifndef _WINDOWS_PLATFORM_
	string model_path = res_path + "/models/";
	string texture_path = res_path + "/texture/";
#else
	string model_path = res_path + "\\models\\";
	string texture_path = res_path + "\\texture\\";
#endif
    Rigid_Geometry* ball = new Rigid_Geometry("ball", (model_path + "sphere.obj").c_str(),Vector3d(0.0,0,-40),Vector3d(0,0,0),Vector3d(0.6,0.6,0.6),100,new IMPLICIT_SPHERE<float>(VECTOR<float,3>(),1));
    ball->setKr(1);
    ball->LoadTexture((texture_path + "ball.bmp").c_str());
    m_objects.addElement(ball);
}

void Sysfric::ShootBall() {
    double v = 8;
    start_angle = 0;
    m_objects.findByName("ball")[0]->v = Vector3d(-v * sin(angle / 180.0 * 3.1415926), 0, v * cos(angle / 180.0 * 3.1415926));
}

void Sysfric::Reset() {
    typedef VECTOR<float, 3> TV;
    clear();
    supplemental = 0;
    shoots += 1;
    bowlings.clear();
#ifndef _WINDOWS_PLATFORM_
	string model_path = res_path + "/models/";
	string texture_path = res_path + "/texture/";
#else
	string model_path = res_path + "\\models\\";
	string texture_path = res_path + "\\texture\\";
#endif
//    Rigid_Geometry* road1 = new Rigid_Geometry("road", (model_path + "cube.obj").c_str(),Vector3d(0,-3,-50),Vector3d(0,0,0),Vector3d(100,3,100),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
//    Rigid_Geometry* cube = new Rigid_Geometry("cube0", (model_path + "cube.obj").c_str(),Vector3d(0,-1,-40),Vector3d(0,0,0),Vector3d(100,1,100),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
    Rigid_Geometry* cube1 = new Rigid_Geometry("cube1", (model_path + "cube.obj").c_str(),Vector3d(-2,1,-40),Vector3d(0,0,0),Vector3d(1,1,1),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
    Rigid_Geometry* cube2 = new Rigid_Geometry("cube2", (model_path + "cube.obj").c_str(),Vector3d(0,1,-40),Vector3d(0,0,0),Vector3d(1,1,1.1),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
//    Rigid_Geometry* cube3 = new Rigid_Geometry("cube3", (model_path + "cube.obj").c_str(),Vector3d(0,0.5,-60),Vector3d(0,0,0),Vector3d(1,0.5,1),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
//    cube->setKr(0);
//    Rigid_Geometry* cube2 = new Rigid_Geometry("cube2", (model_path + "cube.obj").c_str(),Vector3d(5,5,-39.5),Vector3d(0,0,0),Vector3d(1,1,1),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
//    road1->setNailed();
//    road1->setKr(0.1);
//    road1->LoadTexture((texture_path + "wood.bmp").c_str(), 0.6);
//    cube->setNailed();
//    cube->LoadTexture((texture_path + "wood.bmp").c_str(), 0.6);
//    cube->kf = 0.1;
    cube1->LoadTexture((texture_path + "marble.bmp").c_str(), 0.6);
    cube1->setKr(0);
//    cube1->v = Vector3d(-5,0,0);
//    cube1->w = Vector3d(0,5,0);
    cube1->kf = 0.9;
    PtJoint* ptJoint = new PtJoint(Vector3d(-1,-1,-1),Vector3d(1,-1,-1));
//    PtJoint* ptJoint1 = new PtJoint(Vector3d(-1,-1,1),Vector3d(1,-1,1));
    ptJoint->parent = cube2;
    ptJoint->child = cube1;
//    ptJoint1->parent = cube2;
//    ptJoint1->child = cube1;
    cube2->LoadTexture((texture_path + "marble.bmp").c_str(), 0.6);
    cube2->kf = 0.9;
    cube2->setNailed();
//    cube3->LoadTexture((texture_path + "marble.bmp").c_str(), 0.6);
//    cube3->v = Vector3d(0,0,10);
//    cube3->kf = 0.1;
//    cube2->w = Vector3d(0,1,0);
//    m_objects.addElement(road1);
//    m_objects.addElement(cube);
    m_objects.addElement(cube1);
    m_objects.addElement(cube2);
//    m_objects.addElement(cube3);
    joints.push_back(ptJoint);
//    joints.push_back(ptJoint1);
    game_mode = BEGIN_SHOOT;
}

void Sysfric::Display() {
    this->SysDynPtc::Display();
    static int dir = 1;
    static double s0 = 0;
    double s = getTime();
    if (start_angle == 1) {
        angle += (int((s - s0) / 0.032)) * dir;
        if (angle > 30) {
            angle = 60 - angle;
            dir = -1;
        }
        if (angle < -30) {
            angle = -60 - angle;
            dir = 1;
        }
    }
    
    s0 = s;
    
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    // 保存当前投影矩阵
    // 保存当前模型视图矩阵
    
    glPushMatrix();
    glTranslated(-30, 30, 0);
    glRotated(angle, 0, 0, 1);
    glScaled(2,5,2);
    glColor3d(1,1,1);
    glBegin(GL_TRIANGLES);
    glVertex2d(-0.5,0);
    glVertex2d(0.5,0);
    glVertex2d(0, 0.5);
    glEnd();
    
    // 恢复之前保存的模型视图矩阵
    glPopMatrix();
    /*
     glMatrixMode(GL_PROJECTION);
     
     // 恢复之前保存的投影矩阵
     glPopMatrix();
     */    glEnable(GL_LIGHTING);
}

void Sysfric::collide_event(Geometric* s1,Geometric* s2) {
    if ((strcmp(s1->name.c_str(), "ball") == 0 && strcmp(s2->name.c_str(), "gate") == 0) ||
        (strcmp(s2->name.c_str(), "ball") == 0 && strcmp(s1->name.c_str(), "gate") == 0)) {
        RemoveBall();
        hit_time = getTime();
        game_mode = STOP;
    }
}

void Sysfric::setState(double* state, double t) {
    if (game_mode == STOP && getTime() - hit_time > 5.5) {
        for (int i = 0; i < bowlings.size(); ++i) {
            while (i < bowlings.size()) {
                if (bowlings[i]->x[1] > 1.5)
                    break;
                m_objects.removeByName(bowlings[i]->name.c_str());
                bowlings.erase(bowlings.begin() + i);
            }
        }
        m_objects.clearRemoveList();
        if (supplemental == 0) {
            game_mode = BEGIN_SHOOT;
            AddBall();
            supplemental = 1;
        } else {
            Reset();
        }
        start_angle = 1;
        time += t;
        return;
    }
    if (game_mode == STOP && getTime() - hit_time > 5) {
        time += t;
        return;
    }
    this->SysDynPtc::setState(state, t);
}

double* Sysfric::DerivEval(double* state, double t) {
    if (game_mode == STOP && getTime() - hit_time > 5) {
        return 0;
    }
    return this->SysDynPtc::DerivEval(state, t);
}
