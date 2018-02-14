#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>
#include <ypspur.h>
#include <scip2awd.h>

#define GOAL_DISTANCE	500
#define GOAL_DIAGONAL	GOAL_DISTANCE*2
#define GOAL_ACCURACY	10.0
#define GOAL_MIN	GOAL_DIAGONAL-GOAL_ACCURACY
#define GOAL_MAX	GOAL_DIAGONAL+GOAL_ACCURACY
#define CHECKTIME	10

int escape;
float prediag_data[CHECKTIME];

void ctrlc(int notused)
{
	escape = 1;
	signal(SIGINT, NULL);
}

int main(int argc, char *argv[])
{
	S2Port *port;    // ポート
	S2Sdd_t buf;     // データ取得用ダブルバッファ
	S2Scan_t *scan;  // データ読み出し用構造体
	S2Param_t param; // センサのパラメータ構造体

	int cycle, ret;

	if (argc != 2) {
		fprintf(stderr, "USAGE: %s device\n", argv[0]);
		return 0;
	}

	if (Spur_init() < 0)
	{
		fprintf(stderr, "ERROR : cannot open spur.\n");

		return -1;
	}

	Spur_set_pos_GL(0, 0, 0);
	Spur_set_pos_LC(0, 0, 0);

	Spur_set_vel(0.1);
	Spur_set_accel(0.5);
	Spur_set_angvel(M_PI/4);
	Spur_set_angaccel(M_PI);

	// ポートを開く
	port = Scip2_Open(argv[1], B0);
	if (port == 0){
		fprintf(stderr, "ERROR: Failed to open device.\n");
		return 0;
	}
	printf("Port opened\n");

	// 初期化
	cycle = 1;
	escape = 0;
	prediag_data[0] = 500.0;
	signal(SIGINT, ctrlc);
	S2Sdd_Init(&buf);
	printf("Buffer initialized\n");

	// URGのパラメータ取得
	Scip2CMD_PP(port, &param);

	// URG-04LXの全方向のデータを取得開始
	Scip2CMD_StartMS(port, param.step_min, param.step_max,
			1, 0, 0, &buf, SCIP2_ENC_3BYTE);

	while (!escape) {
		ret = S2Sdd_Begin(&buf, &scan);
		if (ret > 0){
			int nowall = 0;
			unsigned long diag_data;

			diag_data = scan->data[param.step_front - param.step_min
						+ param.step_resolution/12];

			// 定期的に壁の横にいるか検出する
			if (cycle == CHECKTIME-1) {
				while (cycle != 1) {
					// 壁の横にいないなら左前方に壁が見えるまでその場で回転する
					if (prediag_data[cycle] < 20.0 || 2000.0 < prediag_data[cycle])
						nowall++;
					cycle--;
					printf("%d(%ld) : %d\n", cycle, diag_data, nowall);
				}
				if (CHECKTIME/4 < nowall) {
					printf("Adjust\n");
					Spur_spin_FS(M_PI/16);
					sleep(1);
				} else {
					prediag_data[0] = prediag_data[CHECKTIME-1];
					cycle = 1;
				}
				nowall = 0;
			}

			printf ("distance to wall is %ld\n", diag_data);

			Spur_orient_FS(0);

			if (GOAL_MIN <= diag_data && diag_data <= GOAL_MAX)
				Spur_orient_FS((prediag_data[cycle-1] < diag_data) ?
						M_PI/256 : -M_PI/256);
			else
				Spur_orient_FS((GOAL_MAX < diag_data) ?
					       	M_PI/128 : -M_PI/128);

			prediag_data[cycle] = diag_data;
			cycle++;

			S2Sdd_End(&buf);
		}
		else if (ret == -1){
			// 致命的なエラー時(URGのケーブルが外れたときなど)
			fprintf(stderr, "ERROR: Fatal error occurred.\n");
			break;
		}
		else{
			// 新しいデータはまだ無い
			usleep(10000);
		}
	}

	Spur_set_vel(0.0);
	Spur_set_accel(-1.0);
	printf("\nStopping\n");

	ret = Scip2CMD_StopMS(port, &buf);
	if (ret == 0){
		fprintf(stderr, "ERROR: StopMS failed.\n");
		return 0;
	}

	printf("Stopped\n");
	S2Sdd_Dest(&buf);
	printf("Buffer destructed\n");
	Scip2_Close(port);
	printf("Port closed\n");

	return 1;
}
