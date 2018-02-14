/*
 * ロボット-センサ座標系変換サンプルプログラム
 *   Original: 2011/04/19 Taku Shikina
 *   Modified: 2012/12/10 Atsushi Watanabe
 */

/*--------------*/
// ライブラリ
/*--------------*/
#include <stdio.h>                         // printfなどの入出力関数
#include <stdlib.h>                        // mallocとfreeやexitなど
#include <math.h>                          // 数学関数
#include <signal.h>                        // シグナル

#include <ypspur.h>                        // spur用ライブラリ
#include <scip2awd.h>                      // AWDのURGライブラリ

int g_escape;

/*=====================================================*/
// 関数名　　：ctrlc
// 機能概要　：シグナルハンドラの設定．Ctrl+Cでプログラムを終了させる
/*=====================================================*/
void ctrlC( int aStatus )
{
    signal( SIGINT, NULL );
    g_escape = 1;                                // 終了予約
}

void setctrlC(  )
{
    signal( SIGINT, ctrlC );
}

/**************************/
// メインプログラム
/**************************/
int main( int aArgc, char **aArgv )
{
    int ret;
    int i;

    S2Port *urgPort;                            // デバイスファイル名
    S2Sdd_t urgBuf;                             // データを確保するバッファ
    S2Scan_t *urgData;                          // バッファへのポインタ
    S2Param_t urgParam;                         // URGのパラメータを確保

    if( aArgc < 2 )
    {
        fprintf( stderr, "USAGE: %s /dev/ttyACM0\n", aArgv[0] );
        return 0;
    }
    g_escape = 0;
    setctrlC(  );                               // シグナルハンドラの設定

    /**********************spur関連****************************/
    ret = Spur_init(  );                        // 初期化
    if( ret <= 0 )
    {
        fprintf( stderr, "ERROR: Failed to open yp-spur.\n" );
        return 0;
    }
    Spur_set_vel( 0.15 );                       // 最大速度0.15m/sに設定
    Spur_set_accel( 1.0 );                      // 加速度1.0m/sに設定
    Spur_set_angvel( 0.25 );                    // 最大角速度0.25rad/sに設定
    Spur_set_angaccel( 2.0 );                   // 各加速度2.0rad/ssに設定
    Spur_set_pos_GL( 0, 0, 0 );                 // スタート地点を原点にGL座標を設定

    /*********************************************************/

    /**********************URG関連****************************/
    /* ポートオープン */
    urgPort = Scip2_Open( aArgv[1], B0 );       // デバイス名,ボーレート設定

    if( urgPort == 0 )
    {
        fprintf( stderr, "ERROR: Failed to open device. %s\n", aArgv[1] );
        return 0;
    }
    fprintf( stderr, "Port opened\n" );

    /* バッファの初期化 */
    S2Sdd_Init( &urgBuf );
    fprintf( stderr, "Buffer initialized\n" );

    /* URGパラメータの読み出し */
    Scip2CMD_PP( urgPort, &urgParam );

    /* 垂れ流しモードの開始 */
    Scip2CMD_StartMS( urgPort, urgParam.step_min, urgParam.step_max, 1, 0, 0, 
                      &urgBuf, SCIP2_ENC_3BYTE );

    /* 定数の計算 */
    double resolution = 2.0 * M_PI / urgParam.step_resolution;

    /********************************************************/

    /*************ロボットの座標を考慮したURGデータの使用***************/

    Spur_free( );
    
    while( !g_escape )
    {

        /* 測位データの取り出し */
        ret = S2Sdd_Begin( &urgBuf, &urgData );

        if( ret > 0 )
        {
            double d, theta;                    // (mm), (rad) URGの生データ（極座標系）
            double x_sensor, y_sensor;          // (m) URGのデータ（センサ座標系）
            double x_fs, y_fs;                  // (m) FS座標系のURGのデータ（ロボット座標系）
            double x_gl, y_gl;                  // (m) GL座標系のURGのデータ（世界座標系）
            double pos_x_gl, pos_y_gl, pos_theta_gl; // (m) ,(rad) GL座標系のロボットの位置

            // GL座標系のロボットの位置（世界座標系）

            // ロボットの現在位置を取得
            Spur_get_pos_GL( &pos_x_gl, &pos_y_gl, &pos_theta_gl );
	    //printf( "X: %f\tY: %f\tth: %f\n", pos_x_gl, pos_y_gl, pos_theta_gl );

            // センサデータをGL座標系に張り付けて出力
            for ( i = 0; i < urgData->size; i++ )
            {
                // 極座標系のデータの取得
                d = urgData->data[i];
                theta = ( double )( urgParam.step_min + i - urgParam.step_front )
                           * resolution;

                if( d < urgParam.dist_min || d > urgParam.dist_max )
                    continue;

                // 極座標系からセンサ座標系への変換
                x_sensor = d * cos( theta ) * 0.001;    // mm -> m
                y_sensor = d * sin( theta ) * 0.001;    // mm -> m

                // センサ座標系からFS座標系への変換
                x_fs = x_sensor;
                y_fs = y_sensor;

                // FS座標系からGL座標系への変換
                x_gl = pos_x_gl + x_fs * cos( pos_theta_gl ) - y_fs * sin( pos_theta_gl );
                y_gl = pos_y_gl + y_fs * cos( pos_theta_gl ) + x_fs * sin( pos_theta_gl );

                // 出力 
		printf( "%f\t%f\n", x_gl, y_gl );
            }

            S2Sdd_End( &urgBuf );            // アンロック(読み込み終了)

        }
        else if( ret < 0 )
        {                                        // 戻り値が負だとエラー

            fprintf( stderr, "ERROR: Fatal error occurred.\n" );
            break;
        }
        else
        {
            usleep( 10000 );                     // 測域データに新しいデータがない
        }

    }

    /***********************************************/

    /************終了処理***************/

    // YP-Spurの停止 
    Spur_stop(  );

    // URGの終了
    ret = Scip2CMD_StopMS( urgPort, &urgBuf );    // URGの測域停止
    if( ret == 0 )
    {
        fprintf( stderr, "ERROR: StopMS failed.\n" );
        return 0;
    }
    fprintf( stderr, "Stopped\n" );

    S2Sdd_Dest( &urgBuf );                        // バッファの解放
    fprintf( stderr, "Buffer destructed\n" );

    Scip2_Close( urgPort );                           // ポートを閉じる
    fprintf( stderr, "Port closed\n" );

    return 0;
}
