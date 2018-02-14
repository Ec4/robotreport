#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>
#include <ypspur.h>
#include <scip2awd.h>

int escape;

void ctrlc( int notused )
{
	// MDコマンドを発行したままプログラムを終了すると、
	// 次回起動時に少し余分に時間がかかる
	escape = 1;
	signal( SIGINT, NULL );
}

int main( int argc, char *argv[] )
{
	S2Port *port;    // ポート
	S2Sdd_t buf;     // データ取得用ダブルバッファ
	S2Scan_t *scan;  // データ読み出し用構造体
	S2Param_t param; // センサのパラメータ構造体

	int ret;

	if( argc != 2 ){
		fprintf( stderr, "USAGE: %s device\n", argv[0] );
		return 0;
	}

	if ( Spur_init() < 0 )
	{
		fprintf(stderr, "ERROR : cannot open spur.\n");

		return -1;
	}

	Spur_set_pos_GL( 0, 0, 0 );
	Spur_set_pos_LC( 0, 0, 0 );

	Spur_set_vel( 0.2 );
	Spur_set_accel( 1.0 );
	Spur_set_angvel( M_PI );
	Spur_set_angaccel( M_PI );

	//Spur_stop_line_GL( 5.0, 0.0, M_PI / 2 );
	Spur_stop_line_GL( 5.0, 0.0, 0.0 );

	// ポートを開く
	port = Scip2_Open( argv[1], B0 );
	if( port == 0 ){
		fprintf( stderr, "ERROR: Failed to open device.\n" );
		return 0;
	}
	printf( "Port opened\n" );

	// 初期化
	escape = 0;
	signal( SIGINT, ctrlc );
	S2Sdd_Init( &buf );
	printf( "Buffer initialized\n" );

	// URGのパラメータ取得
	Scip2CMD_PP( port, &param );

	// URG-04LXの全方向のデータを取得開始
	Scip2CMD_StartMS( port, param.step_min, param.step_max,
			1, 0, 0, &buf, SCIP2_ENC_3BYTE );

	while( !escape ){
		ret = S2Sdd_Begin( &buf, &scan );
		if( ret > 0 ){
			// 新しいデータがあった時の処理をここで行う
			printf( "Front distance: %lu mm\n", 
					scan->data[ param.step_front - param.step_min ] );
			if ( scan->data[ param.step_front - param.step_min ] < 1000.0 ) 
			{
				Spur_set_vel( 0.0 );
				Spur_set_accel( -2.5 );
				escape = 1;
			}
			// S2Sdd_BeginとS2Sdd_Endの間でのみ、構造体scanの中身にアクセス可能
			S2Sdd_End( &buf );
		}
		else if( ret == -1 ){
			// 致命的なエラー時(URGのケーブルが外れたときなど)
			fprintf( stderr, "ERROR: Fatal error occurred.\n" );
			break;
		}
		else{
			// 新しいデータはまだ無い
			usleep( 10000 );
		}
	}
	printf( "\nStopping\n" );

	ret = Scip2CMD_StopMS( port, &buf );
	if( ret == 0 ){
		fprintf( stderr, "ERROR: StopMS failed.\n" );
		return 0;
	}

	printf( "Stopped\n" );
	S2Sdd_Dest( &buf );
	printf( "Buffer destructed\n" );
	Scip2_Close( port );
	printf( "Port closed\n" );

	return 1;
}
