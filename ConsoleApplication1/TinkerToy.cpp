// TinkerToy.cpp : Defines the entry point for the console application.
//
#include <iostream>
using namespace std;
#include "Gravity.h"
#include <algorithm>
#include <list>
#include <Windows.h>

#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "imageio.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>

#if defined(_WIN32)
#  include <glut.h>
#elif defined(__APPLE__)
#  include <GLUT/glut.h>
#endif
// #include <glut.h>
/* macros */

/* external definitions (from solver) */
extern void simulation_step( std::vector<Particle*> pVector, float dt );

/* global variables */

static int N;
static float dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

// static Particle *pList;
static std::vector<Particle*> pVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

std::list<Gravity*> gravity;
std::list<SpringForce*> springs;
std::list<RodConstraint*> rods;
std::list<CircularWireConstraint*> circles;

long long level_elapsed_time = 0;
long long level_start_time = 0;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	pVector.clear();
}

static void clear_data ( void )
{
	int i, size = pVector.size();

	for(i=0; i<size; i++){
		pVector[i]->reset();
	}
}

static void init_system(void)
{
	const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	level_start_time = timeGetTime();

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

	//pVector.push_back(new Particle(Vec2f(0.0, 0.0) + Vec2f(dist, 0.0)));
	pVector.push_back(new Particle(center + offset));
	pVector.push_back(new Particle(center + 2 * offset));
	pVector.push_back(new Particle(center + 3 * offset));

	// You shoud replace these with a vector generalized forces and one of
	// constraints...

	for(int i = 0; i < 3; i = i + 1)
	{
		/*
		if (i == 0)
		{
			gravity.push_back(new Gravity(pVector[i], Vec2f(0.0, 0.0)));
		}
		else
		{
			gravity.push_back(new Gravity(pVector[i], Vec2f(0.0, -0.0981)));
		}
		*/
	}

	//springs.push_back(new SpringForce(pVector[0], pVector[1], 0.5, 0.8, 0.5));
	//springs.push_back(new SpringForce(pVector[1], pVector[2], 0.5, 0.8, 0.5));

	//rods.push_back(new RodConstraint(pVector[0], pVector[1], 0.5));
	//rods.push_back(new RodConstraint(pVector[1], pVector[2], 0.5));

	circles.push_back(new CircularWireConstraint(pVector[0], 0.0, dist));
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	//cout << "time: " << (level_elapsed_time - level_start_time) << endl;
	level_elapsed_time = timeGetTime();
	while(!(((level_elapsed_time - level_start_time) % 33) == 0))
	{
		level_elapsed_time = timeGetTime();
		Sleep(1);
	}
	
	if (((level_elapsed_time - level_start_time) % 1000) <= 33)
	{
		cout << "frames per second: " << frame_number << endl;
		frame_number = 0;
	}
	frame_number++;

	glutSwapBuffers ();
}

static void draw_particles ( void )
{
	int size = pVector.size();

	for(int i=0; i< size; i++)
	{
		pVector[i]->draw();
	}
}

static void draw_forces ( void )
{
	// change this to iteration over full 
	for_each(springs.begin(), springs.end(), [](SpringForce* s)
	{
		s->draw();
	});

	for_each(gravity.begin(), gravity.end(), [](Gravity* g)
	{
		g->draw();
	});

}

static void draw_constraints ( void )
{
	// change this to iteration over full set
	for_each(rods.begin(), rods.end(), [](RodConstraint* r)
	{
		r->draw();
	});

	for_each(circles.begin(), circles.end(), [](CircularWireConstraint* c)
	{
		c->draw();
	});
}

/*
----------------------------------------------------------------------
relates mouse movements to tinker toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	int i, j;
	// int size, flag;
	int hi, hj;
	// float x, y;
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0]
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((mx /(float)win_x)*N);
	j = (int)(((win_y-my)/(float)win_y)*N);

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {
		cout << "x pos : " << i << " y pos: " << j << endl;
	}

	if ( mouse_down[2] ) {
	}

	hi = (int)((       hmx /(float)win_x)*N);
	hj = (int)(((win_y-hmy)/(float)win_y)*N);

	if( mouse_release[0] ) {
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case 'c':
	case 'C':
		clear_data ();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data ();
		exit ( 0 );
		break;

	case ' ':
		dsim = !dsim;
		break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	if (dsim)
	{
		for_each(rods.begin(), rods.end(), [](RodConstraint* r)
		{
			r->apply();
		});

		for_each(springs.begin(), springs.end(), [](SpringForce* s)
		{
			s->apply();
		});

		for_each(gravity.begin(), gravity.end(), [](Gravity* g)
		{
			g->apply();
		});

		for_each(circles.begin(), circles.end(), [](CircularWireConstraint* c)
		{
			c->apply();
		});

		simulation_step(pVector, dt);
	}
	else        {get_from_UI();remap_GUI();}

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_constraints();
	draw_particles();

	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Tinkertoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		dt = 0.1f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;

	init_system();

	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}
