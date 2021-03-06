#include "main.h"
#include "Sysfric.h"
#include "IMPLICIT_CUBE.h"
#include "IMPLICIT_SPHERE.h"
#include "Rigid_Geometry.h"
#include "PtJoint.h"
#include "RotJoint.h"
#include <stdio.h>

using namespace SimLib;

extern string res_path;
extern int g_demo;

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

void Sysfric::addCube(double x, double y) {
#ifndef _WINDOWS_PLATFORM_
	string model_path = res_path + "/models/";
	string texture_path = res_path + "/texture/";
#else
	string model_path = res_path + "\\models\\";
	string texture_path = res_path + "\\texture\\";
#endif
    char buf[100];
    static int t = 0;
    t += 1;
    sprintf(buf, "cube%d", t);
    Rigid_Geometry* cube2 = new Rigid_Geometry(buf, (model_path + "sphere.obj").c_str(),Vector3d(x, 20, y),Vector3d(0,0,0),Vector3d(1,1,1),100,new IMPLICIT_SPHERE<float>(VECTOR<float,3>(0, 0, 0),1));
    cube2->v = Vector3d((rand()+0.0)/RAND_MAX * 3 - 1.5, 0, (rand()+0.0)/RAND_MAX * 3 - 1.5);
    cube2->kf = 0.5;
    cube2->setKr(1);
    cube2->LoadTexture((texture_path + "wood.bmp").c_str());
    m_objects.addElement(cube2);
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
    Rigid_Geometry* cube1 = new Rigid_Geometry("cubee", (model_path + "cube.obj").c_str(),Vector3d(0,-10,0),Vector3d(0,0,0),Vector3d(50,10,50),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-50,-10,-50),TV(50,10,50))));
    cube1->setKr(0.6);
    cube1->kf = 0.3;
    cube1->LoadTexture((texture_path + "marble.bmp").c_str(), 1.0);
    cube1->setNailed();
    if (g_demo == 1) {
        double w = 0.95;
        Rigid_Geometry* frame = new Rigid_Geometry("frame", (model_path + "frame.obj").c_str(),Vector3d(0,0,0),Vector3d(0,0,0),Vector3d(1,1,1),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-5,-5,-5),TV(-5,-5,-5))));
        frame->LoadTexture((texture_path + "wood.bmp").c_str(), 1.0);
        frame->setNailed();
        m_objects.addElement(frame);
        
        for (int i = -2; i <= 2; ++i) {
            Rigid_Geometry* stick31 = new Rigid_Geometry("single_stick", (model_path + "single_stick.obj").c_str(),Vector3d(w * i,5.6,0),Vector3d(0,0,0),Vector3d(0.5,0.5,1),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-5,-5,-5),TV(-5,-5,-5))));
            stick31->LoadTexture((texture_path + "ball.bmp").c_str(), 1.0);
            m_objects.addElement(stick31);
            
            Rigid_Geometry* stick32 = new Rigid_Geometry("single_stick", (model_path + "single_stick.obj").c_str(),Vector3d(w * i,5.6,0),Vector3d(0,0,0),Vector3d(0.5,0.5,-1),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-5,-5,-5),TV(-5,-5,-5))));
            stick32->LoadTexture((texture_path + "ball.bmp").c_str(), 1.0);
            m_objects.addElement(stick32);
            

            Rigid_Geometry* ball = new Rigid_Geometry("ball", (model_path + "sphere.obj").c_str(),Vector3d(0.05 + i * w,3.88 - w/2,0),Vector3d(0,0,0),Vector3d(w/2,w/2,w/2),50,new IMPLICIT_SPHERE<float>(VECTOR<float,3>(),w/2));
            ball->setKr(1);
            ball->LoadTexture((texture_path + "iron.bmp").c_str());
            m_objects.addElement(ball);

            Vector3d pt = Vector3d(w * i, 5.6, 0.8);
            PtJoint* ptJoint = new PtJoint(pt - cube1->x, pt - stick31->x);
            ptJoint->parent = cube1;
            ptJoint->child = stick31;
            joints.push_back(ptJoint);
            
            pt = Vector3d(w * i, 5.6, -0.8);
            ptJoint = new PtJoint(pt - cube1->x, pt - stick32->x);
            ptJoint->parent = cube1;
            ptJoint->child = stick32;
            joints.push_back(ptJoint);

            pt = Vector3d(w * i, 3.88, -0.01);
            ptJoint = new PtJoint(pt - stick31->x, pt - ball->x);
            ptJoint->parent = stick31;
            ptJoint->child = ball;
            joints.push_back(ptJoint);
            pt = Vector3d(w * i, 3.88, 0.01);
            ptJoint = new PtJoint(pt - stick32->x, pt - ball->x);
            ptJoint->parent = stick32;
            ptJoint->child = ball;
            joints.push_back(ptJoint);
            pt = Vector3d(w * i, 4.5, 0);
            ptJoint = new PtJoint(pt - stick31->x, pt - ball->x);
            ptJoint->parent = stick31;
            ptJoint->child = ball;
            joints.push_back(ptJoint);
        }
        
    } else {

    double g_w = 12, l_w = 1, h = 10.5;
    Rigid_Geometry* cube2 = new Rigid_Geometry("W1", (model_path + "cube.obj").c_str(),Vector3d(-g_w, h / 3, 0),Vector3d(0,0,45),Vector3d(l_w,h,g_w * 1.5),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-l_w,-h,-g_w * 1.5),TV(l_w, h, g_w * 1.5))));
    cube2->setKr(0.6);
    cube2->kf = 0.3;
    cube2->LoadTexture((texture_path + "ball.bmp").c_str(), 1.0);
    cube2->setNailed();
    Rigid_Geometry* cube3 = new Rigid_Geometry("W2", (model_path + "cube.obj").c_str(),Vector3d(g_w, h / 3, 0),Vector3d(0,0,-45),Vector3d(l_w,h,g_w * 1.5),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-l_w,-h,-g_w * 1.5),TV(l_w, h, g_w * 1.5))));
    cube3->setKr(0.6);
    cube3->kf = 0.3;
    cube3->LoadTexture((texture_path + "ball.bmp").c_str(), 1.0);
    cube3->setNailed();
    Rigid_Geometry* cube4 = new Rigid_Geometry("W3", (model_path + "cube.obj").c_str(),Vector3d(0, h / 3, -g_w),Vector3d(-45,0,0),Vector3d(g_w * 1.5,h,l_w),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-g_w * 1.5,-h,-l_w),TV(g_w * 1.5, h, l_w))));
    cube4->setKr(0.6);
    cube4->kf = 0.3;
    cube4->LoadTexture((texture_path + "ball.bmp").c_str(), 1.0);
    cube4->setNailed();
    Rigid_Geometry* cube5 = new Rigid_Geometry("W4", (model_path + "cube.obj").c_str(),Vector3d(0, h / 3, g_w),Vector3d(45,0,0),Vector3d(g_w * 1.5,h,l_w),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-g_w * 1.5,-h,-l_w),TV(g_w * 1.5, h, l_w))));
    cube5->setKr(0.6);
    cube5->kf = 0.3;
    cube5->LoadTexture((texture_path + "ball.bmp").c_str(), 1.0);
    cube5->setNailed();
    m_objects.addElement(cube2);
    m_objects.addElement(cube3);
    m_objects.addElement(cube4);
    m_objects.addElement(cube5);
    }
    m_objects.addElement(cube1);

    /*    Rigid_Geometry* cube2 = new Rigid_Geometry("cube2", (model_path + "cube.obj").c_str(),Vector3d(0,1,-40),Vector3d(0,0,0),Vector3d(1.1,1,1.1),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
    cube1->LoadTexture((texture_path + "marble.bmp").c_str(), 0.6);
    cube1->setKr(0);
    cube1->kf = 0.9;
    cube2->LoadTexture((texture_path + "marble.bmp").c_str(), 0.6);
    cube2->kf = 0.9;
    cube2->setNailed();
    m_objects.addElement(cube1);
    m_objects.addElement(cube2);
    RotJoint* rotJoint = new RotJoint(Vector3d(-1.05,-1,-1),Vector3d(1.05,-1,-1),Vector3d(0,0,-1),-30,45);
    rotJoint->parent = cube2;
    rotJoint->child = cube1;
    rotJoint->initialize();
    rotJoint->kr = 0;
    rotJoint->kf = 0;
    rotJoint->kh = 100;
    joints.push_back(rotJoint);*/
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
