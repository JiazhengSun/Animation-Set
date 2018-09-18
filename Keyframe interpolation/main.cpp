#include <string>
#include <iostream>
#include <stdlib.h>

#if defined(__APPLE__) && defined(__MACH__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "spline.h"

using namespace std;

// global variables
Spline2d mySpline;
unsigned int window_width = 800, window_height = 600;

// opengl functions
void myGlutResize(int w, int h);

void myGlutDisplay(void);

void myGlutIdle();

void myGlutKeyboard(unsigned char key, int x, int y);

void myGlutMouse(int button, int state, int x, int y);

void dragAndDrop(int x, int y);

void ShowUIText();

int PointIndex;
bool firstLastDeboor = false;
bool firstLastConnect = false;
// main function
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitWindowSize(window_width, window_height);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
    glutCreateWindow("Spline Interpolation");
    glutDisplayFunc(myGlutDisplay);
    glutReshapeFunc(myGlutResize);
    glutKeyboardFunc(myGlutKeyboard);
    glutMouseFunc(myGlutMouse);
    glutMotionFunc(dragAndDrop);
    // anti aliasing
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glutMainLoop();

    return 0;
}

void dragAndDrop(int x, int y){
  if (PointIndex != -1) {
    mySpline.UpdatePointCoor(PointIndex, x, window_height - y);
  }
  glutPostRedisplay();
}

void myGlutResize(int w, int h)
{
    window_width = w;
    window_height = h;
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glutPostRedisplay();
}

void myGlutDisplay(void) {
    glClearColor(1.f , 1.f, 1.f ,1.0f);
    ::glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);    // opengl matrix for camera
    glLoadIdentity();
    gluOrtho2D(0, window_width, 0, window_height);
    glMatrixMode(GL_MODELVIEW);     // opengl matrix for object
    glLoadIdentity();

    glColor3d(0, 0, 0);
    // render all the control points
    glPointSize(10);
    vector<Eigen::Vector2d> controlPoints = mySpline.GetControlPoints();
    glBegin(GL_POINTS);
    for (int p = 0; p < controlPoints.size(); p++) {
        glVertex2d(controlPoints[p](0), controlPoints[p](1));
    }
    glEnd();

    // render the interpolated spline
    glLineWidth(2);
    vector<Eigen::Vector2d> interpolatedPoints = mySpline.GetInterpolatedSpline();
    glBegin(GL_LINE_STRIP);
    for (int p = 0; p < interpolatedPoints.size(); p++) {
        glVertex2d(interpolatedPoints[p](0), interpolatedPoints[p](1));
    }
    glEnd();

    // render the text
    ShowUIText();

    glutSwapBuffers();
}

void myGlutKeyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27:    // esc
            exit(0);
            break;
        case 'C':
            mySpline.RemoveAll();
            break;
        case 'l':
            mySpline.SetSplineType(Spline2d::LINEAR);
            break;
        case 'b':
            mySpline.SetSplineType(Spline2d::BEZIER);
            break;
        case 'd':
            mySpline.SetSplineType(Spline2d::DEBOOR);
            break;
        case 'c':
            mySpline.SetSplineType(Spline2d::CATMULL);
            break;
        case '3':
            firstLastDeboor = !firstLastDeboor;
            break;
        case '4':
            firstLastConnect = !firstLastConnect;
            break;
        case '5':
            mySpline.SetSplineType(Spline2d::CTWO);
        default:
            break;
    }

    glutPostRedisplay();
}

void myGlutMouse(int button, int state, int x, int y) {
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
      int temp = mySpline.CheckPoint(x, window_height - y);
        if(temp == -1) {
            mySpline.AddPoint(Eigen::Vector2d(x, window_height - y));   // append control point to spline
        }
        PointIndex = temp;

    } else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
        //mySpline.RemoveLastPoint(); // remove last added point
        mySpline.RemoveCertainPoint(x, window_height - y);
    }

    glutPostRedisplay();
}

void RenderBitmapString(float x, float y, void *font,char *string)
{
    char *c;
    ::glRasterPos2f(x, y);
    for (c=string; *c != '\0'; c++) {
        ::glutBitmapCharacter(font, *c);
    }
    ::glRasterPos2f(x+1, y);
    for (c=string; *c != '\0'; c++) {
        ::glutBitmapCharacter(font, *c);
    }
}

void ShowUIText()
{
    int* pFont=(int*)GLUT_BITMAP_8_BY_13;

    GLint viewport[4];
    ::glGetIntegerv(GL_VIEWPORT,viewport);
    const int win_w = viewport[2];
    const int win_h = viewport[3];

    glPushMatrix();
    glLoadIdentity();
    glScalef(1, -1, 1);
    glTranslatef(0, -win_h, 0);

    char s_tmp[256];
    int interval = 14;
    glColor3d(0.0, 0.0, 0.0);
    strcpy(s_tmp,"Left Click: add point");
    RenderBitmapString(10, 20, pFont, s_tmp);

    strcpy(s_tmp,"Right Click: remove last point");
    RenderBitmapString(10, 20 + interval, pFont, s_tmp);

    strcpy(s_tmp,"\'C\': Remove all points");
    RenderBitmapString(10, 20 + interval * 2, pFont, s_tmp);

    strcpy(s_tmp,"\'l\': Linear");
    RenderBitmapString(10, 20 + interval * 3, pFont, s_tmp);

    strcpy(s_tmp,"\'b\': Bezier");
    RenderBitmapString(10, 20 + interval * 4, pFont, s_tmp);

    strcpy(s_tmp,"\'d\': DeBoor");
    RenderBitmapString(10, 20 + interval * 5, pFont, s_tmp);

    strcpy(s_tmp,"\'3\': DeBoor include first and last point");
    RenderBitmapString(10, 20 + interval * 6, pFont, s_tmp);

    strcpy(s_tmp,"\'c\': Catmull-Rom");
    RenderBitmapString(10, 20 + interval * 7, pFont, s_tmp);

    strcpy(s_tmp,"\'4\': Connect first and last control point");
    RenderBitmapString(10, 20 + interval * 8, pFont, s_tmp);

    strcpy(s_tmp,"\'5\': C2 interpolation");
    RenderBitmapString(10, 20 + interval * 9, pFont, s_tmp);

    glPopMatrix();
}
