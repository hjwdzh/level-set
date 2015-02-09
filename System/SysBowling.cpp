#include "main.h"
#include "SysBowling.h"
#include "IMPLICIT_CUBE.h"
#include "IMPLICIT_SPHERE.h"
#include "Rigid_Geometry.h"
#include <stdio.h>

using namespace SimLib;

extern string res_path;

SysBowling::SysBowling()
{
    start_angle = 1;
    shoots = 0;
    supplemental = 0;
}

SysBowling::~SysBowling()
{}

void SysBowling::Initialize()
{
    ks = 100; kd = 10;
    Reset();
}

void SysBowling::RemoveBall() {
    m_objects.removeByName("ball");
}

void SysBowling::AddBall() {
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

void SysBowling::ShootBall() {
    double v = 8;
    start_angle = 0;
    m_objects.findByName("ball")[0]->v = Vector3d(-v * sin(0 / 180.0 * 3.1415926), 0, v * cos(0 / 180.0 * 3.1415926));
}

void SysBowling::Reset() {
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
    Rigid_Geometry* road1 = new Rigid_Geometry("road", (model_path + "cube.obj").c_str(),Vector3d(0,-3,-50),Vector3d(0,180,0),Vector3d(7,3,100),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
    Rigid_Geometry* road2 = new Rigid_Geometry("road", (model_path + "cube.obj").c_str(),Vector3d(-7.5,-1.7,-50),Vector3d(0,180,0),Vector3d(7,1,100),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
    Rigid_Geometry* road3 = new Rigid_Geometry("road", (model_path + "cube.obj").c_str(),Vector3d(7.5,-1.7,-50),Vector3d(0,180,0),Vector3d(7,1,100),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
    Rigid_Geometry* road4 = new Rigid_Geometry("road", (model_path + "cube.obj").c_str(),Vector3d(-15,-1,-50),Vector3d(0,180,0),Vector3d(7,1,100),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
    Rigid_Geometry* road5 = new Rigid_Geometry("road", (model_path + "cube.obj").c_str(),Vector3d(15,-1,-50),Vector3d(0,180,0),Vector3d(7,1,100),1,new IMPLICIT_CUBE<float>(RANGE<TV>(TV(-1,-1,-1),TV(1,1,1))));
    road1->setNailed();
    road1->setKr(0.1);
    road1->LoadTexture((texture_path + "wood.bmp").c_str(), 0.6);
    road2->setNailed();
    road2->setKr(0.1);
    road2->LoadTexture((texture_path + "black.bmp").c_str(), 0.6);
    road3->setNailed();
    road3->setKr(0.1);
    road3->LoadTexture((texture_path + "black.bmp").c_str(), 0.6);
    road4->setNailed();
    road4->setKr(0.1);
    road4->LoadTexture((texture_path + "wood.bmp").c_str(), 0.6);
    road5->setNailed();
    road5->setKr(0.1);
    road5->LoadTexture((texture_path + "wood.bmp").c_str(), 0.6);

    Rigid_Geometry* gate = new Rigid_Geometry("gate", (model_path + "gate.obj").c_str(),Vector3d(0.53,-1,0),Vector3d(0,180,0),Vector3d(10,10,10),10);
    gate->setNailed();
    gate->setKr(0);
    gate->LoadTexture((texture_path + "marble.bmp").c_str());

    typedef SimLib::VECTOR<float,3> TV;
    bowlings.resize(10);
    int t = 0;
    double scale = 5;
    for (int i = 1; i <= 4; ++i) {
        for (int j = 1; j <= 5 - i; ++j) {
            double h = scale * cos(3.14159265 / 12) * (1 - i / 4.0);
            double x = 0.5 * scale * (j - 1 - (4 - i) * 0.5);
            char buf[20];
            sprintf(buf, "bowling%d", t);
            bowlings[t] = new Rigid_Geometry(buf, (model_path + "bowling.obj").c_str(),Vector3d(x,-0.2,h),Vector3d(0,0,0),Vector3d(5,5,5),1);
            ++t;
        }
    }
    for (int i = 0; i < t; ++i) {
        m_objects.addElement(bowlings[i]);
    }
    
    m_objects.addElement(road1);
    m_objects.addElement(road2);
    m_objects.addElement(road3);
    m_objects.addElement(road4);
    m_objects.addElement(road5);
    m_objects.addElement(gate);
    AddBall();
    game_mode = BEGIN_SHOOT;
}

void SysBowling::Display() {
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

void SysBowling::collide_event(Geometric* s1,Geometric* s2) {
    if ((strcmp(s1->name.c_str(), "ball") == 0 && strcmp(s2->name.c_str(), "gate") == 0) ||
        (strcmp(s2->name.c_str(), "ball") == 0 && strcmp(s1->name.c_str(), "gate") == 0)) {
        RemoveBall();
        hit_time = getTime();
        game_mode = STOP;
    }
}

void SysBowling::setState(double* state, double t) {
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

double* SysBowling::DerivEval(double* state, double t) {
    if (game_mode == STOP && getTime() - hit_time > 5) {
        return 0;
    }
    return this->SysDynPtc::DerivEval(state, t);
}
