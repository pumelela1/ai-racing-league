#ifndef STUB_GL_H
#define STUB_GL_H
typedef unsigned int GLuint; typedef int GLint; typedef float GLfloat; typedef unsigned int GLenum;
typedef unsigned char GLubyte; typedef int GLsizei; typedef void GLvoid; typedef double GLdouble;
typedef unsigned char GLboolean; typedef unsigned int GLbitfield; typedef float GLclampf;
#define GL_RGB 0x1907
#define GL_RGBA 0x1908
#define GL_UNSIGNED_BYTE 0x1401
#define GL_PACK_ROW_LENGTH 0x0D02
#define GL_PACK_ALIGNMENT 0x0D05
#define GL_FRONT 0x0404
#define GL_BACK 0x0405
#ifdef __cplusplus
extern "C" {
#endif
void glReadPixels(GLint,GLint,GLsizei,GLsizei,GLenum,GLenum,GLvoid*);
void glPixelStorei(GLenum,GLint);
void glReadBuffer(GLenum);
#ifdef __cplusplus
}
#endif
#endif
