#ifndef LOAD_OPENGL_H
#define LOAD_OPENGL_H

#if WIN32
#include <cstdlib> // To disable glut::exit() function
#include <GL/glut.h>
#elif defined(__linux)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <Glut/glut.h>
#else
#error "Load OpenGL Error: What's your operating system?"
#endif

#endif // #ifndef LOAD_OPENGL_H

