//
//  main.h
//  simulation
//
//  Created by skyer on 14-4-13.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#define _WINDOWS_PLATFORM_

#ifndef __simulation__main__
#define __simulation__main__

#include <iostream>
using namespace std;
#ifndef _WINDOWS_PLATFORM_
#if __linux || __unix
// for glGenerateMipmap
// see: http://stackoverflow.com/a/16630508/732016
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#else /* not Linux/Windows; probably Mac */
#include <OpenGL/gl.h>
#include <OpengL/glu.h>
#include <GLUT/glut.h>
#endif /* __linux || __unix */
#else /* on _WINDOWS_PLATFORM_ */
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )
#include <Windows.h>
#include "gl.h"
#include "glu.h"
#include "glut.h"
#endif

#include "SysBowling.h"
#include "Camera.h"
#include <string>
using namespace std;

extern SysBowling* g_sys;
extern Camera* g_camera;
extern int g_WindowWidth;
extern int g_WindowHeight;
extern double g_top, g_right, g_d;
void Initialize();
void Uninitialize();

void DrawTextHHL(const char* text, float x, float y);
void DrawFunc();
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void Animate(int id);

#endif /* defined(__simulation__main__) */
