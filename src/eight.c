#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ypspur.h>

void iamat(void);

int main( void )
{
	double x, y, theta;

	if ( Spur_init() < 0 )
	{
		fprintf(stderr, "ERROR : cannot open spur.\n");
		return -1;
	}

	Spur_set_pos_GL( 0, 0, 0);
	Spur_set_pos_LC( 0, 0, 0);

	// ちょっと加速
	Spur_set_vel( 0.2 );
	Spur_set_accel( 1.0 );
	Spur_set_angvel( M_PI  );
	Spur_set_angaccel( M_PI );

	Spur_circle_GL( 0, 0.5, 0.5 );
	while( !Spur_near_pos_GL( 0, 1.0, 0.01 ) ) {
		iamat();
		usleep( 10000 );
	}

	Spur_circle_GL( 0, 0.5, 0.5 );
	while( !Spur_near_pos_GL( 0, 0, 0.01 ) ) {
		iamat();
		usleep( 10000 );
	}

	Spur_circle_GL( 0, -0.5, -0.5 );
	while( !Spur_near_pos_GL( 0, -1.0, 0.01 ) ) {
		iamat();
		usleep( 10000 );
	}

	Spur_circle_GL( 0, -0.5, -0.5 );
	while( !Spur_near_pos_GL( 0, 0, 0.01 ) ) {
		iamat();
		usleep( 10000 );
	}
	
	Spur_stop(  );
	usleep( 40000 );
	Spur_free(  );
	printf( "Hit Ctrl-C to exit.\n" );
	while( 1 )
	{
		Spur_get_pos_GL( &x, &y, &theta );
		printf( "%f %f %f\n", x, y, theta * 180.0 / M_PI );
		usleep( 1000000 );
	}
	
	return 0;
}

void iamat(void)
{
	double real_x, real_y, real_th;
	Spur_get_pos_GL( &real_x, &real_y, &real_th );
	printf("(%f, %f)\n", real_x, real_y);
}
