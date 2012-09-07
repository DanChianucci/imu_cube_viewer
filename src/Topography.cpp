#define _WIN32_WINNT 0x0501
#define WINVER 0x0501

#include "libfreenect.h"
#include "libfreenect_sync.h"

#include "Serial/BufferedAsyncSerial.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

#define PERIOD 50

#if defined(__APPLE__)
#	include <GLUT/glut.h>
#	include <OpenGL/gl.h>
#	include <OpenGL/glu.h>
#else
#	include <GL/glut.h>
#	include <GL/gl.h>
#	include <GL/glu.h>
#endif

bool displayDebug = true;
bool displayKinect = true;
bool displayQuat = true;
bool homing = false;
bool color = true;

int window;
GLuint gl_rgb_tex;
int mx = -1, my = -1; // Prevous mouse coordinates
int rotangles[2] =
{ 0 }; // Panning angles
float zoom = 66; // zoom factor
int width, height;

float euler[3];
float quat[4];
float hQ[4];

BufferedAsyncSerial myPort;
int commError;
char commMsg[50];

const char * port = "COM8";
int baud = 19200;

bool noKinect;
short *depth = 0;
char *rgb = 0;
uint32_t ts;

long missed = 0;
long total = 0;

int frameCount;
int previousTime;
float fps = 0;

void quatConj(float * result, float * q);

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
		freenect_sync_stop();
		glutDestroyWindow(window);
		exit(0);
	}

	if (key == 'w') //Zoom in
		zoom *= 1.1f;
	if (key == 's') //Zoom out
		zoom /= 1.1f;

	if (key == ' ') //Home in the Kinect
	{
		rotangles[0] = 0;
		rotangles[1] = 0;
	}

	if (key == 'd')
		displayDebug = !displayDebug;
	if (key == 'k')
		displayKinect = !displayKinect;
	if (key == 'q')
		displayQuat = !displayQuat;
	if (key == 'h')
	{
		quatConj(hQ, quat);
		homing = true;
	}
	if (key == 'n')
		homing = false;
	if (key == 'c')
		color = !color;
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

void calcEulerFrom(float * q)
{
	euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3],
			2 * q[0] * q[0] + 2 * q[1] * q[1] - 1) * 180 / 3.14159625; //psi

	euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180 / 3.14159625; //theta

	euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1],
			2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180 / 3.14159625; //phi
}

int readQuat()
{
	myPort.write("g", 1);
	myPort.flush();

	boost::this_thread::sleep(boost::posix_time::milliseconds(25));

	std::string data = myPort.readStringUntil(",.\r\n");

	std::vector<std::string> Qs;

	boost::split(Qs, data, boost::is_any_of(","), boost::token_compress_on);

	if (Qs.size() >= 4)
	{
		float tmp[4];
		for (int i = 0; i <= 3; i++)
		{
			try
			{
				tmp[i] = boost::lexical_cast<float>(Qs[i]);
			} catch (...)
			{
				missed++;
				printf("Bad Lexical Cast");
				printf("%d < %s , %s , %s , %s >\n", i, Qs[0].c_str(),
						Qs[1].c_str(), Qs[2].c_str(), Qs[3].c_str());
				return -2;
			}
		}
		for (int i = 0; i <= 3; i++)
		{
			quat[i] = tmp[i];
		}
		return 1;
	}
	else
	{
		missed++;
		return -1;
	}
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

void getKinectData()
{
	if (!noKinect && displayKinect)
	{
		if (freenect_sync_get_depth((void**) &depth, &ts, 0,FREENECT_DEPTH_11BIT) < 0)
			noKinect = true;

		if (freenect_sync_get_video((void**) &rgb, &ts, 0, FREENECT_VIDEO_RGB)
				< 0)
			noKinect = true;

	}
}

void UpdateData()
{
	getKinectData();
	getQuaternionData();
	glutPostRedisplay();
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

		if (!noKinect && displayKinect)
		{
			glTranslatef(-(width / 2 - 100), -(height / 2 - 100), -3);
			glScalef(2, 2, 2);
		}
		else
		{
			glTranslatef(0, 0, -3);
			glScalef(10, 10, 10);
		}

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

		char buf[60];
		sprintf(buf, "Q< %4.2f , %4.2f , %4.2f , %4.2f >", quat[0], quat[1],
				quat[2], quat[3]);
		glColor3f(1, 1, 1);
		print_bitmap_string(buf, -(width / 2 - 9), -(height / 2 - 1 * 15));

		sprintf(buf, "E< %6.2f , %6.2f , %6.2f >", euler[0], euler[1],
				euler[2]);
		print_bitmap_string(buf, -(width / 2 - 9), -(height / 2 - 2 * 15));
	}

}

void drawKinectCloud()
{
	if (!noKinect && displayKinect)
	{
		static unsigned int indices[480][640];
		static short xyz[480][640][3];
		int i, j;
		for (i = 0; i < 480; i++)
		{
			for (j = 0; j < 640; j++)
			{
				xyz[i][j][0] = j;
				xyz[i][j][1] = i;
				xyz[i][j][2] = depth[i * 640 + j];
				indices[i][j] = i * 640 + j;
			}
		}

		glColor3f(1, 1, 1);
		glClear(GL_DEPTH_BUFFER_BIT);

		glLoadIdentity();
		glPushMatrix();

		glScalef(zoom, zoom, 1);
		glTranslatef(0, 0, -3.5);
		glRotatef(rotangles[0], 1, 0, 0);
		glRotatef(rotangles[1], 0, 1, 0);
		glTranslatef(0, 0, 1.5);

		LoadVertexMatrix();

		// Set the projection from the XYZ to the texture image
		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glScalef(1 / 640.0f, 1 / 480.0f, 1);
		LoadRGBMatrix();
		LoadVertexMatrix();
		glMatrixMode(GL_MODELVIEW);

		glPointSize(.6);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_SHORT, 0, xyz);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(3, GL_SHORT, 0, xyz);

		if (color)
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
			glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB,
					GL_UNSIGNED_BYTE, rgb);
		}
		glPointSize(.6f);
		glDrawElements(GL_POINTS, 640 * 480, GL_UNSIGNED_INT, indices);
		glPopMatrix();
		glDisable(GL_TEXTURE_2D);
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
	//Kinect Message
	//--------------------------------------------------------------------------
	if (noKinect)
	{
		glColor3f(1, 0, 0);
		print_bitmap_string("No Kinect Found", -(width / 2 - 9),
				height / 2 - 2 * 15);
	}

	else
	{

		glColor3f(0, 1, 0);
		print_bitmap_string("Kinect Sensor Found", -(width / 2 - 9),
				height / 2 - 2 * 15);
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
				   "[SPACE] -  Reset View Rotation",
				   "[w]     -  Zoom In",
			       "[s]     -  Zoom Out",
			       "[d]     -  Toggle Debug Display",
			       "[k]     -  Toggle Kinect Display",
			       "[q]     -  Toggle Quaternion Display",
			       "[h]     -  Home The Quaternion",
			       "[n]     -  Cancel Quaternion Homing",
			       "[c]     -  Toggle Color Display"
					};
	glColor3f(1, 1, 1);
	for(int i =0; i<10; i++)
	{
		print_bitmap_string(msg3[i], -(width / 2 - 9), height/2 - (15+i)*15);
	}
}

void DrawGLScene()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawKinectCloud();
	drawQuaternionCube();

	if (displayDebug)
		drawDebug();

	glFlush();
	glutSwapBuffers();
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

void initSerialComms()
{
	printf("Opening Serial Comms\n");
	commError = 0;
	sprintf(commMsg, "IMU connected");

	try
	{
		myPort.open(port, baud);
	} catch (...)
	{
	}

	if (!myPort.isOpen())
	{
		sprintf(commMsg, "Failed %s @ %d bps", port, baud);
		commError = 1;
		printf(commMsg);
		return;
	}

	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	printf("\tSending WHOAMI\n");
	myPort.write("????.", 5);
	myPort.flush();

	boost::this_thread::sleep(boost::posix_time::seconds(1));

	std::string response = myPort.readStringUntil("\r\n");

	printf("\tResponse: %s\n", response.c_str());

	int numBytes = strlen(response.c_str());

	if (numBytes > 0)
	{
		if (response.compare("I am an AHRSuIMU") == 0)
		{
			sprintf(commMsg, "IMU Connected");
			commError = 0;
		}

		else
		{
			sprintf(commMsg, "Unexpected IMU Response");
			commError = 2;
		}
	}

	else
	{
		sprintf(commMsg, "No IMU Response");
		commError = 3;
	}

	printf("%s\n\n",commMsg);
	return;
}

void initKinectComms()
{
	printf("Initialising Kinect\n");
	noKinect = false;


	if (freenect_sync_get_depth((void**) &depth, &ts, 0, FREENECT_DEPTH_11BIT)< 0)
		noKinect = true;
	printf("\tDepth Stream Started\n");

	if (freenect_sync_get_video((void**) &rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
		noKinect = true;
	printf("\tVideo Stream Started\n");

	if(!noKinect)
	{
		printf("Kinect Succesfully Connected\n");
		freenect_sync_set_led(LED_BLINK_GREEN ,0);
		freenect_sync_set_tilt_degs(30,0);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		freenect_sync_set_tilt_degs(0,0);
	}
	else
		printf("Failed to Communicate with Kinect");
}

int main(int argc, char **argv)
{
	initSerialComms();
	initKinectComms();

	width = 640;
	height = 480;
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow("LibFreenect");

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

