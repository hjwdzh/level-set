#include "main.h"
#include "Solver.h"
#ifndef _WINDOWS_PLATFORM_
#include <sys/time.h>
#else
#include <time.h>
#endif

double g_mouseState = 0, g_simTime = 0;
int g_mouseX, g_mouseY;
Geometric* g_selectedObject;
extern int stopsign;
// 响应按键事件
void KeyboardFunc(unsigned char key, int x, int y)
{
    switch (key)
    {
		case 27:
			exit(0);
			break;
        case 'G':
            for (int i = 0; i < 1; ++i) {
                g_sys->addCube(((rand() + 0.0) / RAND_MAX - 0.5) * 20, ((rand() + 0.0) / RAND_MAX - 0.5) * 20);
            }
            break;
        case 'A':
            g_camera->Rotate(0, -5);
            break;
        case 'a':
            g_camera->Rotate(0, -1);
            break;
            
        case 'D':
            g_camera->Rotate(0, 5);
            break;
        case 'd':
            // 向右移动
            g_camera->Rotate(0, 1);
            break;
            
        case 'W':
            g_camera->Rotate(1, -5);
            break;
        case 'w':
            g_camera->Rotate(1, -1);
            break;

        case 'S':
            g_camera->Rotate(1, 5);
            break;
        case 's':
            g_camera->Rotate(1, 1);
            break;

        case 'Q':
            g_camera->Rotate(2, -5);
            break;
        case 'q':
            g_camera->Rotate(2, -1);
            break;

        case 'E':
            g_camera->Rotate(2, 5);
            break;
        case 'e':
            g_camera->Rotate(2, 1);
            break;
            
        case 'V':
            g_camera->Move(-10);
            break;
        case 'v':
            g_camera->Move(-0.1);
            break;
            
        case 'F':
            g_camera->Move(10);
            break;
        case 'f':
            g_camera->Move(0.1);
            break;
        case 'm':
        case 'M':
            stopsign = 1 - stopsign;
            break;
        case 'P':
            stopsign = 2;
            break;
        default:
            break;
    }
}

// 响应鼠标按下与释放事件
void MouseFunc(int button, int state, int x, int y)
{
    if (state == 0)
    {
        g_mouseX = x;
        g_mouseY = y;
        double mouseX = 2 * (double)x / g_WindowWidth - 1;
        double mouseY = 2 * (double)(g_WindowHeight - y) / g_WindowHeight - 1;
        g_selectedObject = g_sys->mouseSelect(mouseX * g_right, mouseY * g_top);
        if (button == GLUT_RIGHT_BUTTON)
        {
            g_selectedObject->setNailed();
        }
        else
        {
            g_mouseState = 1;
        }
    }
    else
    {
        if (g_selectedObject)
        {
            g_selectedObject->setUserForce(Vector3d());
            g_selectedObject->setSelected(false);
            g_selectedObject = 0;
        }
        g_mouseState = 0;
        g_mouseState = 0;
    }
}

// 响应鼠标移动事件
void MotionFunc(int x, int y)
{
    g_mouseX = x;
    g_mouseY = y;
}

void Animate(int id)
{
#ifndef _WINDOWS_PLATFORM_
	timeval t1, t2;
	gettimeofday(&t1, 0);
#else
	DWORD t1, t2;
	t1 = GetTickCount();
#endif
    if (stopsign == 1) {
        glutTimerFunc(33, Animate, 1);
        return;
    }
    if (g_mouseState && g_selectedObject)
    {
        double mouseX, mouseY;
        Vector4d v(g_selectedObject->x[0],g_selectedObject->x[1],g_selectedObject->x[2],1);
        v = g_camera->lookat * v;
        v = v / v[3];
        mouseX = (2 * (double)g_mouseX / g_WindowWidth - 1) * g_right / (-g_d) * v[2];
        mouseY = (2 * (double)(g_WindowHeight - g_mouseY) / g_WindowHeight - 1) * g_top / (-g_d) * v[2];
        double hitX = v[0];
        double hitY = v[1];
        v = Vector4d(mouseX - hitX, mouseY - hitY, 0, 0);
        v = g_camera->lookat.inverse() * v;
        g_selectedObject->setUserForce(Vector3d(v[0], v[1], v[2]) * 5);
    }
    for (int i = 0; i < 1; ++i)
        Solver::NRBS(*g_sys, 0.05);
#ifndef _WINDOWS_PLATFORM_
    gettimeofday(&t2, 0);
    g_simTime = (t2.tv_usec - t1.tv_usec) * 1e-6 + (t2.tv_sec - t1.tv_sec);
#else
	t2 = GetTickCount();
	g_simTime = (t2 - t1) * 1e-3;
#endif
    if (stopsign == 2)
        stopsign = 1;
    glutTimerFunc(33, Animate, 1);
}