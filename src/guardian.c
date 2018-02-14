#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>
#include <string.h>
#include <ypspur.h>
#include <scip2awd.h>

// Spurに投げる命令の単位
#define POLE_R		0.1
#define DISTANCE	0.5
#define LOSTAREA	2

#define MAXOBJ		8
#define OBJBUF		512

enum robostate {
	Pole_setup,
	Patrol,
	Alien_found,
	Alien_chase,
	Alien_lost
};

typedef struct {
	int use;
	double x, y;
} OBJECT;

enum robostate state = Pole_setup;
OBJECT obj[MAXOBJ][OBJBUF];
double human_x, human_y, human_t;
int prev_use[MAXOBJ];
int escape;

void ctrlc(int notused)
{
	escape = 1;
	signal(SIGINT, NULL);
}

void gl_to_lc(double glx, double gly, double *lcx, double *lcy)
{
	double pos_x_gl, pos_y_gl, pos_theta_gl;
	double pos_x_lc, pos_y_lc, pos_theta_lc;

	Spur_get_pos_GL(&pos_x_gl, &pos_y_gl, &pos_theta_gl);
	Spur_get_pos_LC(&pos_x_lc, &pos_y_lc, &pos_theta_lc);

	*lcx = glx + pos_x_lc - pos_x_gl;
	*lcy = gly + pos_y_lc - pos_y_gl;

	return;
}

// 物体を覆う円の半径を求めるのに使いたいが、
// 物体の中心は円でないと求められない
int within_range(OBJECT p, OBJECT q, double range)
{
	//printf("%fm /%fm", hypot((p.x-q.x), (p.y-q.y)), range);
	return (hypot((p.x-q.x), (p.y-q.y)) <= range) ? 1 : 0;
}

/* 参考: http://www.iot-kyoto.com/satoh/2016/01/29/tangent-003 */
void p3_to_circle(OBJECT *o, double *x, double *y, double *r)
{
	int p1, p2, p3;
	double a, b, c, d;
	double x1, y1, x2, y2, x3, y3;

	p1 = o[0].use;
	p2 = p1*2/3;
	p3 = p1/3;

	x1 = o[p1].x;
	y1 = o[p1].y;
	x2 = o[p2].x;
	y2 = o[p2].y;
	x3 = o[p3].x;
	y3 = o[p3].y;

	a = x2 - x1;
	b = y2 - y1;
	c = x3 - x1;
	d = y3 - y1;

	*x = x1 + (d*(a*a + b*b) - b*(c*c + d*d)) / (a*d - b*c) / 2 ;

	*y = b ? (a*(x1+x2-(*x)-(*x)) + b*(y1+y2)) / b / 2
		: (c*(x1+x3-(*x)-(*x)) + d*(y1+y3)) / d / 2;

	*r = (sqrt(((*x)-x1) * ((*x)-x1) + ((*y)-y1) * ((*y)-y1)) +
			sqrt(((*x)-x2) * ((*x)-x2) + ((*y)-y2) * ((*y)-y2)) +
			sqrt(((*x)-x3) * ((*x)-x3) + ((*y)-y3) * ((*y)-y3))) / 3;

	return;
}

void discard_obj(int objnum)
{
	int i;
	for (i = 0; i < OBJBUF; i++)
		obj[objnum][i].use = obj[objnum][i].x = obj[objnum][i].y = 0;
	return;
}

int mostuse() {
	int i, m = 0;

	for (i = 0; i < MAXOBJ; i++)
		m = (m > obj[i][0].use) ? m : obj[i][0].use;

	for (i = 0; i < MAXOBJ; i++)
		if (m == obj[i][0].use)
			return i;

	return -1;
}

// 区分け
void pigeonhole(OBJECT *tmp, int tmpcnt)
{
	int i, objnum, tail_n, first, last, val;

	// prev_use取得
	for (objnum = 0; 0 <= objnum && objnum < MAXOBJ; objnum++)
		prev_use[objnum] = obj[objnum][0].use;

	// obj[A][0].useが1のAが存在すればそこから1cm未満か検索
	// 次がuse0もしくは[0]ならそこをエンドポイントに
	while (tmpcnt > 0) {
		while (!tmp[tmpcnt].use)	tmpcnt--;
		first = (tmpcnt < 0) ? 0 : tmpcnt;
		while (tmp[tmpcnt].use)		tmpcnt--;
		last = (tmpcnt < 0) ? 0 : tmpcnt+1;
#ifdef PRN
		printf("first:%d last:%d\n", first, last);
#endif
		val = first-last+1;

		if (val == 1) {
#ifdef PRN
			printf("NOISE CLEAR\n");
#endif
			continue;	// ノイズ除去
		}

		// ここからtmpがどのobj[A]に近いかを判別する
		for (objnum = 0; 0 <= objnum && objnum < MAXOBJ; objnum++) {
#ifdef PRN
			printf("NEAR SEARCHING: objnum:%d\n", objnum);
#endif
			tail_n = obj[objnum][0].use;

			// 空のオブジェクトは検査しない
			if (tail_n == 0) {
#ifdef PRN
				printf("EMPTY OBJECT\n");
#endif
				continue;
			}

			for (i = first; i != last; i--) {
				if (!within_range (tmp[i], obj[objnum][tail_n], 0.05)) {
#ifdef PRN
					printf("NO MATCH(OUTRANGE):\n"
						"tmp(%f,%f)\tobj(%f,%f)\n"
						, tmp[i].x, tmp[i].y, obj[objnum][tail_n].x, obj[objnum][tail_n].y);
#endif
					continue;	// no match
				}

				// OBJBUF-tail_n-1とfirst-last+1を比較して
				// memcpy(memory.hが必要？)
				if (OBJBUF-tail_n-1 > first-last+1) {	// 空き有
#ifdef PRN
					printf("EMPTY\n");
#endif
					memcpy(&obj[objnum][tail_n+1], &tmp[last], sizeof(OBJECT)*val);
					obj[objnum][0].use += val;
#ifdef OBJCAP
					printf("----------\nCAPTURE\n");
					printf("tail_n: %d\n", tail_n);
					for (i = 1; i <= obj[objnum][0].use; i++)
						printf("%d:\t%f\t%f\n" , i, obj[objnum][i].x, obj[objnum][i].y);
					printf("----------\n");
#endif
				} else {
#ifdef PRN
					printf("FULL\n");		// 空き無
#endif
					memcpy(&obj[objnum][1], &tmp[last], sizeof(OBJECT)*val);
					obj[objnum][0].use = val;
				}
				objnum = -2;	// break後メイン処理ループも抜けるため
				break;
			}
		}

		// tmpが既に登録されているobj[A]にヒットした
		if (objnum != MAXOBJ)	continue;

		// tmpがどのobj[A]にも近くなかった
#ifdef PRN
		printf("NO NEAR OBJECT val: %d\n", val);
#endif
		if (val < 5) {
#ifdef PRN
			printf("NOISE CLEAR\n");
#endif
			continue;	// ノイズ除去
		}

		// 必要かもしれない
		//for (objnum = 0; objnum < MAXOBJ; objnum++)
		for (objnum = 0; 0 <= objnum && objnum < MAXOBJ; objnum++) {
#ifdef PRN
			printf("objnum: %d : use %d\n", objnum, obj[objnum][0].use);
#endif
			tail_n = obj[objnum][0].use;
			if (tail_n == 0) {
#ifdef PRN
				printf("NEWOBJ CREATED objnum: %d\n", objnum);
#endif
				memcpy(&obj[objnum][1], &tmp[last], sizeof(OBJECT)*val);
				obj[objnum][0].use = val;
				objnum = -1;
				break;
			}
		}

		if (objnum == MAXOBJ) {	// 空きがなかった
			for (i = 1; i < MAXOBJ; i++)
				discard_obj(i);

			break;
		}
	}

	for (objnum = 0; objnum < MAXOBJ; objnum++)
		if (obj[objnum][0].use == prev_use[objnum])
			discard_obj(objnum);

	// tmpは初期化しなくても上書きされる

	return;
}

// PIを法としたthetaの絶対値が0.01未満であれば半周
int half_circle(double theta)
{
	return (fmod(fabs(theta), M_PI) < 0.01) ? 1 : 0;
}

int main(int argc, char *argv[])
{
	S2Port *port;	 // デバイスポート
	S2Sdd_t buf;	 // データ取得用ダブルバッファ
	S2Scan_t *scan;  // データ読み出し用構造体
	S2Param_t param; // センサのパラメータ構造体

	int i, j, ret, human = -1;
	double pos_x_gl, pos_y_gl, pos_theta_gl; // (m) ,(rad) GL座標系のロボットの位置

	if (argc != 2) {
		fprintf(stderr, "USAGE: %s device\n", argv[0]);
		return 0;
	}

	// Spur初期化
	if (Spur_init() < 0) {
		fprintf(stderr, "ERROR : cannot open spur.\n");
		return -1;
	}

	Spur_set_pos_GL(0, 0, 0);
	Spur_set_pos_LC(0, 0, 0);	// 最初だけ
	Spur_set_vel(0.1);
	Spur_set_accel(0.5);
	Spur_set_angvel(M_PI/2);
	Spur_set_angaccel(M_PI/8);

	// Scip初期化
	port = Scip2_Open(argv[1], B0);
	if (port == 0) {
		fprintf(stderr, "ERROR: Failed to open device.\n");
		return 0;
	}
	printf("Port opened\n");
	signal(SIGINT, ctrlc);
	S2Sdd_Init(&buf);
	printf("Buffer initialized\n");

	// 初期化
	escape = 0;

	// オブジェクト初期化
	for (i = 0; i < MAXOBJ; i++)
		for (j = 0; j < OBJBUF; j++)
			obj[i][j].use = obj[i][j].x = obj[i][j].y = 0;

	// URGのパラメータ取得
	Scip2CMD_PP(port, &param);

	// URG-04LXの全方向のデータを取得開始
	Scip2CMD_StartMS(port, param.step_min, param.step_max,
			1, 0, 0, &buf, SCIP2_ENC_3BYTE);

	while (!escape) {
		ret = S2Sdd_Begin(&buf, &scan);
		if (ret > 0) {
			OBJECT tmp[scan->size];
			double pole_x, pole_y, pole_r;
			int obj_m = 0;

			double d, theta;	// (mm), (rad) URGの生データ（極座標系）
			double x_urg, y_urg;	// (m) URGのデータ（センサ座標系）
			double x_fs, y_fs;	// (m) FS座標系のURGのデータ（ロボット座標系）
			double x_gl, y_gl;	// (m) GL座標系のURGのデータ（世界座標系）
			double x_lc, y_lc;	// (m) LC座標系のURGのデータ（ポール座標系）

			// ロボットの現在位置を取得
			Spur_get_pos_GL(&pos_x_gl, &pos_y_gl, &pos_theta_gl);

			// センサデータをGL座標系に張り付け
			for (i = 0; i < scan->size; i++)
			{
				// 極座標系のデータの取得
				d = scan->data[i];
				theta = (double)(param.step_min + i - param.step_front)
					* 2.0 * M_PI / param.step_resolution;

				//// ものに当たりそうになったらバックする機能
				//printf("DDDD %f\n", d);
				//if (scan->size/2-5 < i && i <scan->size/2+5 && 0 < d && d < 10) {
				//	Spur_stop();
				//	Spur_vel(-0.2, pos_theta_gl);
				//	usleep(100000);
				//}

				// printf("DEBUG用 theta: %f\n", theta);
				// Pole_setupではthetaの一部だけ使うようにする
				// thetaがある範囲にいるときcontinue
				if ((state == Pole_setup && theta < 0)	// 左しか見ない
					|| d < param.dist_min || d > 2000) {	// maxだと5.6mも！！
					tmp[i].use = 0;
#ifdef RAWTEST
					printf("----\t----\n");
#endif
					continue;
				}

				// 極座標系からセンサ座標系への変換
				x_urg = d * cos(theta) * 0.001;
				y_urg = d * sin(theta) * 0.001;

				// センサ座標系からFS座標系への変換
				x_fs = x_urg;
				y_fs = y_urg;

				// FS座標系からGL座標系への変換
				x_gl = pos_x_gl + x_fs * cos(pos_theta_gl)
						- y_fs * sin(pos_theta_gl);
				y_gl = pos_y_gl + y_fs * cos(pos_theta_gl)
						+ x_fs * sin(pos_theta_gl);

				// GL座標系からLC座標系への変換
				gl_to_lc(x_gl, y_gl, &x_lc, &y_lc);
				tmp[i].x = x_lc;
				tmp[i].y = y_lc;
				tmp[i].use = 1;
#ifdef RAWTEST
				printf("%f\t%f\n" , x_lc, y_lc);
#endif
			}
#ifdef TMPTEST
			printf("TMP START\n");
			for (i = 0; i < scan->size; i++) {
				if (tmp[i].use)
					printf("0A\t%f\t%f\n" , tmp[i].x, tmp[i].y);
				else
					printf("----\t----\n");
			}
			printf("TMP END\n");
#endif
			// センサが反応したGL座標をつなげ、配列として実装されたオブジェクトにする
			// obj[0][]はポールの集合(全て隣り合っている)
			// obj[1][]以降を隣り合った点の集合とする

			// 次のサイクルでその周辺になければ消す(useを0に)
			// obj[0][]とかは[0][0]に100.0以外の何かが入っているとき
			// 1回で複数検出されないものは明らかにノイズなので、tmp[]は毎回リセットする
			// tmp[]の類似のやつを探してあれば全部追加する、なければ削除する
			// 消されるのはobj[0][],[1][]だけ。
			// obj[1][]はAlien->Patrolになるときリセットする。

			// 仮定：同じ物体はtmp配列の連続したところに保存される

			pigeonhole(tmp, scan->size);

			double lcx, lcy, lct;
			if (state == Pole_setup) {
				Spur_free();

				obj_m = mostuse();
				if (obj[obj_m][0].use < 500) {	// 500までたまってない
					S2Sdd_End(&buf);
					continue;
				}

#ifdef PRN
				printf("--------\nMAYBE A POLE obj_m: %d\n", obj_m);
				for (i = 1; i < OBJBUF; i++)
					if (obj[obj_m][i].use)
						printf("%3d:\t%f\t%f\n" , i, obj[obj_m][i].x, obj[obj_m][i].y);
				printf("--------\n");
				for (i = 1; i < MAXOBJ; i++)
					printf("obj[%d]:\t%d\n", i, obj[i][obj_m].use);
				printf("--------\n");
#endif

				// 最も反応しているobjの円の方程式を出す
				p3_to_circle(obj[obj_m], &pole_x, &pole_y, &pole_r);

				if (pole_r < POLE_R) {
#ifdef PRN
					printf("----------\n\nGET A POLE!!!\n");
					printf("%f\t%f\t%f\n" , pole_x, pole_y, pole_r);
					printf("----------\n");
#endif
					//最終的にはLCをセットして完了
					Spur_set_pos_LC(-(pole_x-pos_x_gl), -(pole_y-pos_y_gl)
							, pos_theta_gl);
					state = Patrol;

					// 全部消す
					for (i = 0; i < MAXOBJ; i++)
						discard_obj(i);
				}
			} else if (state == Patrol) {
#ifdef STATE
				printf("STATE IS --- PATROL ---\n");
#endif
				// ***周回***
				Spur_circle_LC(0, 0, DISTANCE);

				// human 判定
				for (i = 0; i < MAXOBJ; i++) {
					// 100以上たまっていなければ無視
					if (obj[i][0].use < 100) continue;

					//gl_to_lc(obj[i][300].x, obj[i][300].y, &lcx, &lcy);
					//double alien_dist = pow(lcx, 2.0) + pow(lcy, 2.0);

					// 原点からroot2m以上離れている
#ifdef PRN
					printf("GIWAKU%f\t%f\t%f\n",obj[i][100].x,obj[i][100].y,pow(obj[i][100].x, 2.0) + pow(obj[i][100].y, 2.0));
#endif
					if (1 <= pow(obj[i][100].x, 2.0) + pow(obj[i][100].y, 2.0)) {
#ifdef PRN
						//printf("alien (%f, %f)\n", lcx, lcy);
						//printf("CATCH MODE i:%d\n", i);
#endif
						human = i;
						human_x = obj[human][100].x;
						human_y = obj[human][100].y;
						human_t = atan2(human_y, human_x);

						state = Alien_found;
						for (i = 0; i < MAXOBJ; i++)
							discard_obj(i);
						break;
					}
				}

				// 周回軌道は1周ごとに修正する
				// obj[0][]を利用する
				// 3点取ってきて中心を計算、半径は初期情報として与えられる。
				// (境界判定を簡単化するため3/4周ごとでも良い)
				// 半周ごとにthetaがマイナスになることを利用する
				// 半周ごとにobjはリセットした方が綺麗に出るかも？
				// (曲がる時のズレが大きいようなので)
				// NULLで問題ないようなら47行目も修正
				Spur_get_pos_LC(&lcx, &lcy, &lct);

				// 半周したならば
				if (half_circle(lct)) {
#ifdef PRN
					printf("-----------------------\n\n\n\n\n\nADJUST");
					printf("\n\n\n\n\n\n---------------------\n");
#endif
					obj_m = mostuse();

					int u = obj[obj_m][0].use;
					double d = pow(obj[obj_m][u].x, 2.0) + pow(obj[obj_m][u].y, 2.0);

					// 原点からの距離的にmostuseがポールではなければ抜ける
					if (0.5 <= d)	break;

					p3_to_circle(obj[obj_m], &pole_x, &pole_y, &pole_r);
					Spur_set_pos_LC(-pole_x, -pole_y , lct);

					// 全部消す
					for (i = 0; i < MAXOBJ; i++)
						discard_obj(i);
				}
			} else if (state == Alien_found) {
#ifdef STATE
				printf("STATE IS --- FOUND ---\n");
#endif
				Spur_stop_line_LC(human_x/2, human_y/2, human_t);
				usleep(10000);

//単位の確認
				if (Spur_near_pos_LC(human_x/2, human_y/2, 1))	// 単位の確認
					state = Alien_chase;

				// 不審者のいる場所 （中心点） を見つけたら
				// そことポール(直径12cm)の間に十分な距離があるかを判定し、
				// あれば割り込んでいく。なければ近づいて警告
				//obj_m = mostuse();
				//OBJECT o = obj[obj_m][obj[obj_m][0].use];
				//if (pow(o.x, 2.0) + pow(o.y, 2.0) > 0.5 && obj[obj_m][0].use) {
				//	human = -1;
				//	state = Alien_lost;
				//}
				/*
				} else if (Spur_near_pos_LC(0, 0, LOSTAREA) == 1) {
					// ロボットがLOSTAREAを超えたらロスト、ポールの周回に戻る
#ifdef PRN
					printf("HUMAN!!! human:%d\n", human);
#endif
					Spur_line_LC(human_x, human_y, human_t);

					   //printf("\n--------\n");
					   //for (i = 1; i < OBJBUF; i++)
					   //printf("obj[human][%d]:\t(%f,\t%f)\n", i, obj[human][i].x, obj[human][i].y);
					   //printf("\n--------\n");


					//Spur_stop_line_GL(human_x/2, human_y/2, 0);
				} else {
					human = 0;
					state = Alien_lost;
				}
			*/
			} else if (state == Alien_chase) {
#ifdef STATE
				printf("STATE IS --- CHASE ---\n");
#endif
				// 追いかけるならこの部分を記述する
				obj_m = mostuse();
				OBJECT o = obj[obj_m][obj[obj_m][0].use];

				Spur_stop_line_LC(o.x*3/5, o.y*3/5, atan2(o.y, o.x));
				usleep(10000);

				// 見失ったor追い出した
				if (Spur_near_pos_LC(0, 0, LOSTAREA)) {
					human = -1;

					// 全部消す
					for (i = 0; i < MAXOBJ; i++)
						discard_obj(i);
					state = Alien_lost;
				}
				for (i = 0; i < MAXOBJ; i++) {
					// 100以上たまっていなければ無視
					if (obj[i][0].use < 100) continue;

					// 原点からroot2m以上離れている
					printf("KIETAKA%f\t%f\t%f\n",obj[i][100].x,obj[i][100].y,pow(obj[i][100].x, 2.0) + pow(obj[i][100].y, 2.0));
					// まだ不審者がいる
					if (1 < pow(obj[i][100].x, 2.0) + pow(obj[i][100].y, 2.0)) {
						break;
					} else if (i == MAXOBJ-1) {
						human = -1;

						// 全部消す
						for (i = 0; i < MAXOBJ; i++)
							discard_obj(i);
						state = Alien_lost;
					}
				}

				//if ((pow(o.x, 2.0) + pow(o.y, 2.0) > 0.5 && obj[obj_m][0].use < 1)
				//	|| Spur_near_pos_LC(0, 0, LOSTAREA)) {
				//	human = -1;

				//	// 全部消す
				//	for (i = 0; i < MAXOBJ; i++)
				//		discard_obj(i);
				//	state = Alien_lost;
				//}
			} else if (state == Alien_lost) {
#ifdef STATE
				printf("STATE IS --- LOST ---\n");
#endif

				// 普通に周回軌道に戻るだけでうまくいきそう
				// つまりcircle_LCするだけにしてしまう
				//
				// -> うまくいかなかった
				//
				Spur_circle_LC(0, 0, DISTANCE);
				usleep(10000);
				state = Patrol;

				// 近くに着いた
				//
				//if (Spur_near_pos_LC(0, 0, 0.5)) {
				//	state = Patrol;
				//} else {
				//	// 全部消す
				//	for (i = 0; i < MAXOBJ; i++)
				//		discard_obj(i);
				//}

				// obj_m = mostuse();
				// if (!obj[obj_m][500].use) {
				// 	S2Sdd_End(&buf);
				// 	continue;
				// }

				// // [0].useが多いやつを送るように変更する
				// p3_to_circle(obj[obj_m], &pole_x, &pole_y, &pole_r);

				// // ポールが見つかったら
				// if (pole_r < POLE_R) {
				// 全部消す

				// 最終的にはLCをセットして完了

				// Spur_set_pos_LC(-(pole_x-pos_x_gl), -(pole_y-pos_y_gl), pos_theta_gl);
				//
				// 周回軌道に戻すだけならnear_pos判定に意味はない
				// if (Spur_near_pos_LC(0, 0, 10) == 1)
				// 	state = Patrol;
				// }
			}

			// S2Sdd_BeginとS2Sdd_Endの間でのみ、構造体scanの中身にアクセス可能
			S2Sdd_End(&buf);
		}
		else if(ret == -1) {
			// 致命的なエラー時(URGのケーブルが外れたときなど)
			fprintf(stderr, "ERROR: Fatal error occurred.\n");
			break;
		}
		else {
			// 新しいデータはまだ無い
			usleep(10000);
		}
	}

	Spur_set_vel(0.0);
	Spur_set_accel(-1.0);
	printf("\nStopping\n");

	ret = Scip2CMD_StopMS(port, &buf);
	if (ret == 0) {
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
