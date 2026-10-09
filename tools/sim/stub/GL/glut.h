#ifndef STUB_GLUT_H
#define STUB_GLUT_H
#include <GL/gl.h>
#define GLUT_KEY_F1 1
#define GLUT_KEY_F2 2
#define GLUT_KEY_F3 3
#define GLUT_KEY_F4 4
#define GLUT_KEY_F5 5
#define GLUT_KEY_F6 6
#define GLUT_KEY_F7 7
#define GLUT_KEY_F8 8
#define GLUT_KEY_F9 9
#define GLUT_KEY_F10 10
#define GLUT_KEY_F11 11
#define GLUT_KEY_F12 12
#define GLUT_KEY_LEFT 100
#define GLUT_KEY_UP 101
#define GLUT_KEY_RIGHT 102
#define GLUT_KEY_DOWN 103
#define GLUT_KEY_PAGE_UP 104
#define GLUT_KEY_PAGE_DOWN 105
#define GLUT_KEY_HOME 106
#define GLUT_KEY_END 107
#define GLUT_KEY_INSERT 108
#define GLUT_ELAPSED_TIME 700
#define GLUT_WINDOW_WIDTH 102
#define GLUT_WINDOW_HEIGHT 103
#ifdef __cplusplus
extern "C" {
#endif
void glutPostRedisplay(void); void glutDisplayFunc(void (*)(void)); void glutSwapBuffers(void);
int glutGet(unsigned int); void glutMainLoop(void); void glutIdleFunc(void (*)(void));
void glutTimerFunc(unsigned int, void (*)(int), int);
#ifdef __cplusplus
}
#endif
#endif
