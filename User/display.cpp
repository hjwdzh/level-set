#include "main.h"
#include <stdio.h>
#include <sys/time.h>


extern double g_simTime;
double g_colTime;

// 描绘函数
void DrawFunc()
{
    glViewport(0, 0, g_WindowWidth, g_WindowHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double fov = 35.0, zNear = 1, aspectRatio = (double)g_WindowWidth / g_WindowHeight, zFar = 1000;
    GLdouble rFov = fov * 3.14159265 / 180.0;
    glFrustum( -zNear * tan( rFov / 2.0 ) * aspectRatio,
              zNear * tan( rFov / 2.0 ) * aspectRatio,
              -zNear * tan( rFov / 2.0 ),
              zNear * tan( rFov / 2.0 ),
              zNear, zFar );
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // 清屏
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    GLfloat mat_specular []={1,1,1, 1.0};
//    GLfloat mat_shininess []={50.0};
    GLfloat mat_diffuse []={1,1,1,1};
    GLfloat mat_ambient []={0.5,0.5,0.5, 1.0};
    GLfloat light_position []={-10.0, 10.0, 10, 0};
    float white_light []={1.0, 1.0, 1.0, 1.0};
    GLfloat lmodel_ambient []={1, 1, 1, 1.0};
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
//    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_CULL_FACE);
    glPushMatrix();
    g_camera->Render();
    g_sys->Display();
    glPopMatrix();
    
    float x = 20, y = 20;
    char buf[1000];
    sprintf(buf, "SimFPS: %f Object: %d ColFPS: %f", 1 / g_simTime, g_sys->m_objects.vp.size(), g_colTime / g_simTime);
    DrawTextHHL(buf, x, y);
    // 交换显示缓冲区
    glutSwapBuffers() ;
}

// 在屏幕上显示文本，x 和 y 为屏幕坐标
void DrawTextHHL(const char* text, float x, float y)
{
    const char* c = text;
    // 检查OpenGL状态
    int isDepthOpen = 0;
    int isStencilOpen = 0;
    int isLightOpen = 0;
    int isFogOpen = 0;
    
    if(glIsEnabled(GL_DEPTH_TEST))
    {
        isDepthOpen = 1;
        glDisable(GL_DEPTH_TEST);
    }
    if(glIsEnabled(GL_STENCIL_TEST))
    {
        isStencilOpen = 1;
        glDisable(GL_STENCIL_TEST);
    }
    if(glIsEnabled(GL_LIGHTING))
    {
        isLightOpen = 1;
        glDisable(GL_LIGHTING);
    }
    if(glIsEnabled(GL_FOG))
    {
        isFogOpen = 1;
        glDisable(GL_FOG);
    }
    
    
    
    // 设置字体颜色
    glColor3f(1.0, 1.0, 0.0);
    
    /*
     * 设置正投影
     */
    
    glMatrixMode(GL_PROJECTION);
    
    // 保存当前投影矩阵
    glPushMatrix();
    
    glLoadIdentity();
    gluOrtho2D( 0, g_WindowWidth, 0, g_WindowHeight );
    
    // 反转Y轴（朝下为正方向）(与窗口坐标一致)
    glScalef(1, -1, 1);
    // 将原点移动到屏幕左上方(与窗口坐标一致)
    glTranslatef(0, -g_WindowHeight, 0);
    glMatrixMode(GL_MODELVIEW);
    
    // 保存当前模型视图矩阵
    glPushMatrix();
    glLoadIdentity();
    
    glDisable(GL_LIGHTING);
    // 设置文字位置
    glRasterPos2f( x, y );
    
    // 依次描绘所有字符(使用显示列表)
    for( ; *c != '\0'; c++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
    
    // 恢复之前保存的模型视图矩阵
    glEnable(GL_LIGHTING);
    glPopMatrix();
    
    glMatrixMode(GL_PROJECTION);
    
    // 恢复之前保存的投影矩阵
    glPopMatrix();
    
    // 返回模型视图矩阵状态
    glMatrixMode(GL_MODELVIEW);
    
    // 恢复OpenGL状态
    if(isDepthOpen)
    {
        glEnable(GL_DEPTH_TEST);
    }
    if(isStencilOpen)
    {
        glEnable(GL_STENCIL_TEST);
    }
    if(isLightOpen)
    {
        glEnable(GL_LIGHTING);
    }
    if(isFogOpen)
    {
        glEnable(GL_FOG);
    }
}
