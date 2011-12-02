#include "Graphics.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif defined _WIN32
#include "glutForWindows.h"
#else
#include <GL/glut.h>
#endif

Graphics *m_graphics = NULL;

Graphics::Graphics(const char fname[], const int nrLinks, const double linkLength) 
{
    ManipSimulator* m_sim = new ManipSimulator(fname);
    m_planner                = new ManipPlanner(m_sim);

    m_planner->m_manipSimulator->theta_limits.resize(nrLinks);
    for(int i = 0; i < nrLinks; ++i)
    {
	m_planner->m_manipSimulator->AddLink(linkLength);
        m_planner->m_manipSimulator->theta_limits[i] = -(2*M_PI)/nrLinks + ((nrLinks-i+0.0)/nrLinks*(12))/180*M_PI; //backoff varies from 12 to 0
    }
    m_planner->m_manipSimulator->FK();

    m_selectedCircle = -1;
    m_editRadius     = false;
    m_run = false;
    
}

Graphics::~Graphics(void)
{
	if(m_planner->m_manipSimulator)
		delete m_planner->m_manipSimulator;
    if(m_planner)
	delete m_planner;
}

void Graphics::MainLoop(void)
{	
    m_graphics = this;

//create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(900, 450);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Planner");	   	


//register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutMotionFunc(CallbackEventOnMouseMotion);
    glutIdleFunc(NULL);
    glutTimerFunc(15, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void)
{
    if(m_run && !m_planner->m_manipSimulator->HasRobotReachedGoal())
    {
	m_planner->ConfigurationMove(m_dtheta, m_dx, m_dy);
	m_planner->m_manipSimulator->base_x += m_dx;
	m_planner->m_manipSimulator->base_y += m_dy;
        m_planner->m_manipSimulator->AddToLinkTheta(m_dtheta);
	m_planner->m_manipSimulator->FK();
    }
} 

void Graphics::HandleEventOnMouseMotion(const double mousePosX, const double mousePosY)
{
    if(m_selectedCircle >= 0)
    {
	if(m_editRadius)
	{
	    const double cx = m_planner->m_manipSimulator->m_circles[3 * m_selectedCircle];
	    const double cy = m_planner->m_manipSimulator->m_circles[3 * m_selectedCircle + 1];
	    
	    m_planner->m_manipSimulator->m_circles[3 * m_selectedCircle + 2] = sqrt((cx - mousePosX) * (cx - mousePosX) +
								   (cy - mousePosY) * (cy - mousePosY));
	}
	else
	{
	    m_planner->m_manipSimulator->m_circles[3 * m_selectedCircle] = mousePosX;
	    m_planner->m_manipSimulator->m_circles[3 * m_selectedCircle + 1] = mousePosY;
	}
	
    }
    
}

void Graphics::HandleEventOnMouseBtnDown(const int whichBtn, const double mousePosX, const double mousePosY)
{
	// we don't want this to do anything for now
   /* 
    m_selectedCircle = -1;
    for(int i = 0; i < m_planner->m_manipSimulator->m_circles.size() && m_selectedCircle == -1; i += 3)
    {
	const double cx = m_planner->m_manipSimulator->m_circles[i];
	const double cy = m_planner->m_manipSimulator->m_circles[i + 1];
	const double r  = m_planner->m_manipSimulator->m_circles[i + 2];
	const double d  = sqrt((mousePosX - cx) * (mousePosX - cx) + (mousePosY - cy) * (mousePosY - cy));
	
	if(d <= r)
	    m_selectedCircle = i / 3;
    }
    
    if(m_selectedCircle == -1)
    {
	m_planner->m_manipSimulator->m_circles.push_back(mousePosX);
	m_planner->m_manipSimulator->m_circles.push_back(mousePosY);
	m_planner->m_manipSimulator->m_circles.push_back(1.0);
    }*/    
}

void Graphics::HandleEventOnKeyPress(const int key)
{
    switch(key)
    {
    case 27: //escape key
	exit(0);
	
    case 'r':
	m_editRadius = !m_editRadius;
	break;
	
	
    case 'p':
	m_run = !m_run;
	break;
    }
   
}


void Graphics::HandleEventOnDisplay(void)
{
//draw robot
    glColor3f(1, 0, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    
    const int n = m_planner->m_manipSimulator->GetNrLinks();
    
    glBegin(GL_LINE_STRIP);
    glVertex2d(m_planner->m_manipSimulator->GetLinkStartX(0), m_planner->m_manipSimulator->GetLinkStartY(0));	
    for(int j = 0; j < n; ++j)
	glVertex2d(m_planner->m_manipSimulator->GetLinkEndX(j), m_planner->m_manipSimulator->GetLinkEndY(j));
    glEnd();
    
    for(int j = 0; j < n; ++j)
	DrawCircle2D(m_planner->m_manipSimulator->GetLinkStartX(j), m_planner->m_manipSimulator->GetLinkStartY(j), 0.15);
   
    
//draw goal and obstacles

    glColor3f(0, 1, 0);
    DrawCircle2D(m_planner->m_manipSimulator->GetGoalCenterX(), m_planner->m_manipSimulator->GetGoalCenterY(), m_planner->m_manipSimulator->GetGoalRadius());
    glColor3f(0, 0, 1);
    for(int i = 0; i < m_planner->m_manipSimulator->GetNrObstacles(); ++i)
	DrawCircle2D(m_planner->m_manipSimulator->GetObstacleCenterX(i), 
		     m_planner->m_manipSimulator->GetObstacleCenterY(i), 
		     m_planner->m_manipSimulator->GetObstacleRadius(i));
    
    
    //draw the electrode tip
    glColor3f(1, 1, 0);
    DrawCircle2D(m_planner->GetElectrodeTip().m_x, 
                 m_planner->GetElectrodeTip().m_y, 
                 3*m_planner->m_manipSimulator->GetObstacleRadius(100));
    
    //display the currently sensed OCT points
    glColor3f(1,0,0);
    for(int j=0; j<m_planner->sensedPoints.size(); j++)
    {
        int i = m_planner->sensedPoints[j];
        DrawCircle2D(m_planner->m_manipSimulator->GetObstacleCenterX(i), 
                     m_planner->m_manipSimulator->GetObstacleCenterY(i), 
                     2*m_planner->m_manipSimulator->GetObstacleRadius(i));
    }

    
}


void Graphics::DrawCircle2D(const double cx, const double cy, const double r)
{
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
	glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
    glEnd();	
}


void Graphics::CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);	
	
	glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-12, 12, -6, 6, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();	    
	
	m_graphics->HandleEventOnDisplay();
	
	glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics &&  state == GLUT_DOWN)
    {
	double mouseX, mouseY;
	MousePosition(x, y, &mouseX, &mouseY);
	m_graphics->HandleEventOnMouseBtnDown(button, mouseX , mouseY);
	glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnMouseMotion(int x, int y)
{
    double mouseX, mouseY;
    MousePosition(x, y, &mouseX, &mouseY);
    m_graphics->HandleEventOnMouseMotion(mouseX , mouseY);
    glutPostRedisplay();
}


void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnTimer();
	glutTimerFunc(15, CallbackEventOnTimer, id);
	glutPostRedisplay();	    
    }
}



void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
	m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::MousePosition(const int x, const int y, double *posX, double *posY)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posZ;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, posX, posY, &posZ);
}

int main(int argc, char **argv)
{
    if(argc < 4)
    {
	printf("missing arguments\n");		
	printf("  Planner <obstacle file> <nrLinks> <linkLength> \n");
	return 0;		
    }

    Graphics graphics(argv[1], atoi(argv[2]), atof(argv[3]));
    
    graphics.MainLoop();
    
    return 0;    
}
