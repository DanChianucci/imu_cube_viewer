//============================================================================
// Name        : Topagraphy.h
// Author      : Dan Chianucci
// Description : Header File declares all Variables and functions contained
//				 in Topography.cpp
// 
//============================================================================

#ifndef TOPAGRAPHY_H_
#define TOPAGRAPHY_H_


//Include Serial functionality
#include "Serial/BufferedAsyncSerial.h"

//Include Various Boost functionality
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>


#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>



//Include Standard Headers
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>






//******************************************************************************
//								Display Flags
//******************************************************************************
bool displayDebug = true;				///Whether or not to display debug info
bool displayQuat = true;				///Whether or not to display Cube
bool homing = false;					///Whether or not to Home the Quaternion


//******************************************************************************
//								Glut Variables
//******************************************************************************
int window;								///Window ID
GLuint gl_rgb_tex;						///Texture of 2D Cam
int width;								///Window Width
int height;								///Window Heighth


//******************************************************************************
//								Interaction Variables
//******************************************************************************
int mx = -1;							///Previous Mouse X coords
int my = -1; 							///Previous Mouse Y Coords
int rotangles[2] = { 0 }; 				///Panning angles
float zoom = 66; 						///zoom factor



//******************************************************************************
//								Quaternion Variables
//******************************************************************************
float quat[4];							///Cube Rotations In Quaternion Form
float hQ[4];							///Cube Home Position
float euler[3];							///Cube Rotations Converted to Euler


//******************************************************************************
//								Data Variables
//******************************************************************************
float acc[4],gyr[4],mag[4];


//******************************************************************************
//								Serial Port Variables
//******************************************************************************
BufferedAsyncSerial myPort;				///The Serial port Object
const char * port = "COM8";				///IMU Sensor Always Connects to COM8
const int baud = 9600;					///Serial Port Baud Rate

int commError;							///Serial Comm Error ID
char commMsg[50];						///Serial Comm Error Message

long total = 0;							///Total number of Packets Sent
long missed = 0;   						///Total number of Packet Read Errors





//******************************************************************************
//								FPS Variables
//******************************************************************************
int frameCount;							///The Total number of Frames
int previousTime;						///The last time the FPS was processed
float fps = 0;							///Current FPS

//******************************************************************************
//								Functions
//******************************************************************************
int main(int argc, char **argv);

void initSerialComms();
void InitGL(int Width, int Height);


void mouseMoved(int x, int y);
void mousePress(int button, int state, int x, int y);
void keyPressed(unsigned char key, int x, int y);
void ResizeGLScene(int w, int h);

void UpdateData();
void getQuaternionData();

int readQuat();
void calcEulerFrom(float * q);
void quatConj(float * result, float * q);
void quatProd(float* prod, float * a, float * b);
void normalizeQuat();

void LoadVertexMatrix();
void LoadRGBMatrix();

void DrawGLScene();
void print_bitmap_string(char* s, float x, float y);
void drawQuaternionCube();
void drawFPS();
void drawDebug();

union floatData{
	char arr[4];
	float fl;
};



#endif /* TOPAGRAPHY_H_ */




