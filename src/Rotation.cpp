//============================================================================
// Name        : Topagraphy.cpp
// Author      : Dan Chianucci
// Description : OpenGL App which collects data from a Kinecgt Sensor as well
//				 as an IMU and displays relevant data onscreen.
//============================================================================
#include "Rotation.h"
#include <iostream>
#include <windows.h>

using namespace std;

int main(int argc, char **argv)
{
	initSerialComms();

	width = 640;
	height = 480;
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow("Orientation");

	glutDisplayFunc(&DrawGLScene);
	glutIdleFunc(&UpdateData);
	glutReshapeFunc(&ResizeGLScene);
	glutKeyboardFunc(&keyPressed);
	glutMotionFunc(&mouseMoved);
	glutMouseFunc(&mousePress);
	InitGL(width, height);

	glutMainLoop();

	return 0;
}

void initSerialComms()
{
	cout<<"Opening Serial Comms"<<endl;
	//Unless there is an error, assume its connected

	try
	{
		myPort.open(port, baud);
		Sleep(1000);
	}
	catch (...){}

	if (!myPort.isOpen())
	{
		sprintf(commMsg, "Failed %s @ %d bps", port, baud);
		commError = 1;
		cerr<<commMsg<<endl;
		return;
	}
	else
	{
		commError = 0;
		sprintf(commMsg,"IMU connected");
		cout<<commMsg<<endl;
		return;
	}
}

void InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glEnable(GL_DEPTH_TEST);
	glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	ResizeGLScene(Width, Height);
}

void mouseMoved(int x, int y)
{
	if (mx >= 0 && my >= 0)
	{
		rotangles[0] += y - my;
		rotangles[1] += x - mx;
	}
	mx = x;
	my = y;
}

void mousePress(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		mx = x;
		my = y;
	}
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		mx = -1;
		my = -1;
	}
}

void keyPressed(unsigned char key, int x, int y)
{
	if (key == 27) //esc
	{
		glutDestroyWindow(window);
		exit(0);
	}

	if (key == 'd')
		displayDebug = !displayDebug;
	if (key == 'q')
		displayQuat = !displayQuat;
	if (key == 'h')
	{
		quatConj(hQ, quat);
		homing = true;
	}
	if (key == 'n')
		homing = false;
}

void ResizeGLScene(int w, int h)
{
	GLdouble size;
	GLdouble aspect;

	width = w;
	height = h;
	/* Use the whole window. */
	glViewport(0, 0, w, h);

	/* We are going to do some 2-D orthographic drawing. */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	size = (GLdouble) ((w >= h) ? w : h) / 2.0;

	if (w <= h)
	{
		aspect = (GLdouble) h / (GLdouble) w;
		glOrtho(-size, size, -size * aspect, size * aspect, -100000.0,
				100000.0);
	}
	else
	{
		aspect = (GLdouble) w / (GLdouble) h;
		glOrtho(-size * aspect, size * aspect, -size, size, -100000.0,
				100000.0);
	}

	/* Make the world and window coordinates coincide so that 1.0 in */
	/* model space equals one pixel in window space.                 */
	glScaled(aspect, aspect, 1.0);

	/* Now determine where to draw things. */
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void UpdateData()
{
	getQuaternionData();
	glutPostRedisplay();
}

void getQuaternionData()
{
	if (!commError && displayQuat)
	{
		total++;
		if (readQuat() > 0)
		{
			normalizeQuat();
			if (homing)
			{
				float res[4];
				quatProd(res, hQ, quat);
				calcEulerFrom(res);
			}
			else
			{
				calcEulerFrom(quat);
			}
		}
	}
}

int readQuat()
{
	myPort.read();

	myPort.write("d", 1);
	myPort.flush();
	Sleep(150);

	if(myPort.available()>=52)
	{
		char flData[4];
		cout<<"Q: ";
		for(int i = 0; i<4;i++)
		{
			myPort.read(flData,4);
			memcpy(&quat[i],flData,4);
			cout<<quat[i]<<" ";
		}
		cout<<endl;

		cout<<"A: ";
		for(int i = 0; i<3;i++)
		{
			myPort.read(flData,4);
			memcpy(&acc[i],flData,4);
			cout<<acc[i]<<" ";
		}
		cout<<endl;
		cout<<"G: ";
		for(int i = 0; i<3;i++)
		{
			myPort.read(flData,4);
			memcpy(&gyr[i],flData,4);
			cout<<gyr[i]<<" ";
		}
		cout<<endl;
		cout<<"M: ";
		for(int i = 0; i<3;i++)
		{
			myPort.read(flData,4);
			memcpy(&mag[i],flData,4);
			cout<<mag[i]<<" ";
		}

		cout<<endl;
		cout<<"Avail: "<<myPort.available()<<endl<<endl;

		return 1;
	}

	else
	{
		missed++;
		//cerr<<"Missed: "<<myPort.available()<<endl;
		return -1;
	}
}

void calcEulerFrom(float * q)
{
	euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3],
			2 * q[0] * q[0] + 2 * q[1] * q[1] - 1) * 180 / 3.14159625; //psi

	euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / 3.14159625; //theta

	euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1],
			2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180 / 3.14159625; //phi
}

void quatConj(float * result, float * q)
{
	result[0] = q[0];
	result[1] = -q[1];
	result[2] = -q[2];
	result[3] = -q[3];
}

void quatProd(float* prod, float * a, float * b)
{
	prod[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	prod[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	prod[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	prod[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void normalizeQuat()
{
	float mag = sqrt(
			pow(quat[0], 2) + pow(quat[1], 2) + pow(quat[2], 2)
					+ pow(quat[3], 2));
	quat[0] /= mag;
	quat[1] /= mag;
	quat[2] /= mag;
	quat[3] /= mag;
}

void LoadVertexMatrix()
{
	float fx = 594.21f;
	float fy = 591.04f;
	float a = -0.0030711f;
	float b = 3.3309495f;
	float cx = 339.5f;
	float cy = 242.7f;
	GLfloat mat[16] =
	{ 1 / fx, 0, 0, 0, 0, -1 / fy, 0, 0, 0, 0, 0, a, -cx / fx, cy / fy, -1, b };
	glMultMatrixf(mat);
}

void LoadRGBMatrix()
{
	float mat[16] =
	{ 5.34866271e+02, 3.89654806e+00, 0.00000000e+00, 1.74704200e-02,
			-4.70724694e+00, -5.28843603e+02, 0.00000000e+00, -1.22753400e-02,
			-3.19670762e+02, -2.60999685e+02, 0.00000000e+00, -9.99772000e-01,
			-6.98445586e+00, 3.31139785e+00, 0.00000000e+00, 1.09167360e-02 };
	glMultMatrixf(mat);
}

void DrawGLScene()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawQuaternionCube();

	if (displayDebug)
		drawDebug();

	glFlush();
	glutSwapBuffers();
}

void print_bitmap_string(char* s, float x, float y)
{
	glRasterPos2f(x, y);
	if (s && strlen(s))
	{
		while (*s)
		{
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *s);
			s++;
		}
	}
}

void drawQuaternionCube()
{
	if (!commError && displayQuat)
	{

		glLoadIdentity();
		glPushMatrix();

		glTranslatef(0, 0, -3);
		glScalef(10, 10, 10);

		glRotatef(-euler[2], 0, 0, 1);
		glRotatef(-euler[1], 1, 0, 0);
		glRotatef(-euler[0], 0, 1, 0);

		//Y+
		glBegin(GL_POLYGON);
		glColor3f(0, 1, 0);
		glVertex3f(-30, -5, 20);
		glVertex3f(30, -5, 20);
		glVertex3f(30, 5, 20);
		glVertex3f(-30, 5, 20);
		glEnd();

		//Y-
		glBegin(GL_POLYGON);
		glColor3f(0, 0, 1);
		glVertex3f(-30, -5, -20);
		glVertex3f(30, -5, -20);
		glVertex3f(30, 5, -20);
		glVertex3f(-30, 5, -20);
		glEnd();

		//X-
		glBegin(GL_POLYGON);
		glColor3f(1, 0, 0);
		glVertex3f(-30, -5, -20);
		glVertex3f(-30, -5, 20);
		glVertex3f(-30, 5, 20);
		glVertex3f(-30, 5, -20);
		glEnd();

		//X+
		glBegin(GL_POLYGON);
		glColor3f(1, 1, 0);
		glVertex3f(30, -5, -20);
		glVertex3f(30, -5, 20);
		glVertex3f(30, 5, 20);
		glVertex3f(30, 5, -20);
		glEnd();

		//Z-
		glBegin(GL_POLYGON);
		glColor3f(1, 0, 1);
		glVertex3f(-30, -5, -20);
		glVertex3f(30, -5, -20);
		glVertex3f(30, -5, 20);
		glVertex3f(-30, -5, 20);
		glEnd();

		//Z+
		glBegin(GL_POLYGON);
		glColor3f(0, 1, 1);
		glVertex3f(-30, 5, -20);
		glVertex3f(30, 5, -20);
		glVertex3f(30, 5, 20);
		glVertex3f(-30, 5, 20);
		glEnd();

		glPopMatrix();

		glColor3f(1, 1, 1);

		char buf[60];

		sprintf(buf,"A< %6.2f , %6.2f , %6.2f >",acc[0],acc[1],acc[2]);
		print_bitmap_string(buf,-(width / 2)+9,-(height / 2) + 8 * 15);

		sprintf(buf,"G< %6.2f , %6.2f , %6.2f >",gyr[0],gyr[1],gyr[2]);
		print_bitmap_string(buf,-(width / 2)+9,-(height / 2) + 7 * 15);

		sprintf(buf,"M< %6.2f , %6.2f , %6.2f >",mag[0],mag[1],mag[2]);
		print_bitmap_string(buf,-(width / 2)+9,-(height / 2) + 6 * 15);


		sprintf(buf, "E< %6.2f , %6.2f , %6.2f >", euler[0], euler[1], euler[2]);
		print_bitmap_string(buf, -(width / 2)+9, -(height / 2) + 4 * 15);

		sprintf(buf, "Q< %4.2f , %4.2f , %4.2f , %4.2f >", quat[0], quat[1],quat[2], quat[3]);
		print_bitmap_string(buf, -(width / 2)+9, -(height / 2) + 3 * 15);




	}

}

void drawFPS()
{
	//On the first call this will be invalid, but it resets itself so its ok

	//  Increase frame count
	frameCount++;

	//  Get the number of milliseconds since glutInit called
	//  (or first call to glutGet(GLUT ELAPSED TIME)).
	int currentTime = glutGet(GLUT_ELAPSED_TIME );

	//  Calculate time passed
	int timeInterval = currentTime - previousTime;

	if (timeInterval > 1000)
	{
		//  calculate the number of frames per second
		fps = frameCount / (timeInterval / 1000.0f);
		//  Set time
		previousTime = currentTime;
		//  Reset frame count
		frameCount = 0;
	}

	char msg[20];
	sprintf(msg, "FPS: %4.2f", fps);
	glColor3f(0, 1, 1);
	print_bitmap_string(msg, -(width / 2 - 9), height / 2 - 3 * 15);
}

void drawDebug()
{
	//--------------------------------------------------------------------------
	//Serial Message
	//--------------------------------------------------------------------------
	if (commError)
	{
		glColor3f(1, 0, 0);
		print_bitmap_string(commMsg, -(width / 2 - 9), height / 2 - 15);
	}

	else
	{
		glColor3f(0, 1, 0);
		print_bitmap_string(commMsg, -(width / 2 - 9), height / 2 - 15);
	}

	//--------------------------------------------------------------------------
	//FPS
	//--------------------------------------------------------------------------

	drawFPS();

	//--------------------------------------------------------------------------
	//Missed Serial Packets info
	//--------------------------------------------------------------------------
	glColor3f(0, 1, 1);
	char msg[50];
	sprintf(msg, "Missed: %4d  Total: %4d   Ratio: %4.2f", missed, total,
			(float) missed / total);
	print_bitmap_string(msg, -(width / 2 - 9), height / 2 - 4 * 15);


	//--------------------------------------------------------------------------
	//Coord Rotations
	//--------------------------------------------------------------------------
	sprintf(msg, "RotX: %d     RotY: %d", rotangles[0], rotangles[1]);
	print_bitmap_string(msg, -(width / 2 - 9), height / 2 - 5 * 15);

	//--------------------------------------------------------------------------
	//Quaternion Homing Info
	//--------------------------------------------------------------------------
	char msg2[50];
	if (homing)
		sprintf(msg2, "< %3.2f , %3.2f , %3.2f , %3.2f >", hQ[0], hQ[1], hQ[2],
				hQ[3]);
	else
		sprintf(msg2, "OFF");
	sprintf(msg, "Q Home: %s", msg2);
	print_bitmap_string(msg, -(width / 2 - 9), height / 2 - 6 * 15);

	//--------------------------------------------------------------------------
	//Key Map Directions
	//--------------------------------------------------------------------------
	char* msg3[] ={"[ESC]   -  Close the Application",
			       "[d]     -  Toggle Debug Display",
			       "[q]     -  Toggle Quaternion Display",
			       "[h]     -  Home The Quaternion",
			       "[n]     -  Cancel Quaternion Homing",
					};
	glColor3f(1, 1, 1);
	for(int i =0; i<5; i++)
	{
		print_bitmap_string(msg3[i], -(width / 2 - 9), height/2 - (15+i)*15);
	}
}







