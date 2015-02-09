#ifndef __simulation__Sysfric__
#define __simulation__Sysfric__

#include "Springs.h"
#include "Geometrics.h"
#include "Plane.h"
#include "SysDynPtc.h"
#include "vmath.h"
#include <vector>

class Sysfric: public SysDynPtc
{
public:
    Sysfric();
    ~Sysfric();
    virtual void Initialize();
    void Reset();
    void RemoveBall();
    void AddBall();
    void ShootBall();
    void Display();
    void collide_event(Geometric*,Geometric*);
    void setState(double* state, double t);
    double* DerivEval(double* state, double t);
    double angle;
    double hit_time;
    int start_angle;
    std::vector<Rigid_Geometry*> bowlings;
    enum {BEGIN_SHOOT, SHOOTING, STOP} game_mode;
    
    int shoots, supplemental;
};

#endif /* defined(__simulation__SysDynPtc__) */
