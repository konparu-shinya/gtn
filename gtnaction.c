/********************************************************************************
 * gtn for ラズパイ のAction実行部
 *  パルスモーターコントローラー:L6470
 *                                SPI0 CS0:GPIO17(pin11) CS1:GPIO27(pin13) CS2:GPIO22(pin15) CS3:GPIO23(pin16)
 *      R 9の設定:StepMode
 *      R10の設定:b15-b8:KVAL_HOLD b7-b0:KVAL_RUN/KVAL_ACC/KVAL_DEC
 *  DIO
 *                                GPIO2(pin3) GPIO3(pin5) GPIO4(pin7) GPIO5(pin29) GPIO6(pin31) GPIO13(pin33)
 *  A/D
 *                                USB-RS232C
 *  gcc -o gtnaction gtnaction.c -lwiringPi -lpthread
 ********************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <termios.h>
#include <pthread.h>
#include <net/if.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/shm.h>
#include <linux/serial.h>
#include "wiringPi.h"
#include "wiringPiSPI.h"

// フォトンカウントのゲート補正
#define GATE_COUNT(a) ((long)((double)a*((double)shm->gate_time/10.0)))

// ヒーター温度
#define TARGET_TEMP		40.0
//#define	HEX2TEMP(a)	((double)(a)*0.0263-29.061)
#define	HEX2TEMP(a)	((double)(a)*0.0256-28.084)
//#define	TEMP2HEX(a)	(unsigned short)(((double)(a)+29.061)/0.0263)
#define	TEMP2HEX(a)	(unsigned short)(((double)(a)+28.084)/0.0256)
//static double	temp=0.0;
static int	temp=TEMP2HEX(0);

// MAX1000との接続SPI０
#define MAX_SPI_CHANNEL 0
struct _shm {
	pthread_mutex_t mutex;	// ミューテックス
	long	count;			// フォトンカウント値
	long	gate_time;		// ゲートタイムmsec
} static *shm;

// ラズパイのSPI1はモード3指定ができないのでGPIOでSPI制御する
#define L6470_SPI_L6470 1
#define	GPIO19	19		// SPI1 MISO
#define	GPIO20	20		// SPI1 MOSI
#define	GPIO21	21		// SPI1 SCLK
// L6470のSPI CS
#define	GPIO17	17
#define	GPIO27	27
#define	GPIO22	22
#define	GPIO23	23
const unsigned char L6470_CH[] = {GPIO17, GPIO27, GPIO22, GPIO23, 0};
// IN/OUT
#define	GPIO2	2
#define	GPIO3	3
#define	GPIO4	4
#define	GPIO5	5
#define	GPIO6	6
#define	GPIO13	13
const unsigned char GPIO_CH[] = {GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO13, 0};
#define	GPIO18	18

#define QUEUELIMIT 5
#define	CONSOLE_MAX	20
#define	PPM_MAX	4

#define ACK		0x06
#define NAK		0x15
#define STX		0x02
#define ETX		0x03
#define ENQ		0x05
#define EOT		0x04
#define TIMEOUT	3000

static const int rts=TIOCM_RTS;
// for RS232C
//static const char cnt_head[]={0x0, 0x20, 0x40, 0x60, 0x80, 0xa0, 0xc0, 0xe0};
// for SPI
static const char cnt_head[]={0x0, 0x40, 0x80, 0xc0, 0x0, 0x40, 0x80, 0xc0};

static char ack_msg_data[] = {STX, 0x04,0x01,0x00,0x11, ETX};
                                                                //1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2
static char msg_msg_data[] = {STX, 0x29,0x00,0x00,0xC9,0x99,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3, ETX};

// 動作管理テーブル
struct _seq_tbl {
	int run;					// 0:停止 1:実行中
	int count;					// 動作中の繰り返し回数
	int current;				// 動作中のカレント行番号
	int max_line;				// actionテーブルの最大行数
	int run_times;				// コンソールから受け取った繰り返し回数
	int ret_line;				// 1:Step実行(動作終了時に次のlineを送信する)
	int reg_flag;				// レジスタ読み込み値の保存
	int	busy;					// 1:Wait中
	int	run_line;				// 実行中のline番号
	struct timespec wai_start;	// Wait開始
} static seq_tbl[CONSOLE_MAX]={
		{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0}
	};

// フォトンカウント管理テーブル
struct _cnt_tbl {
	int	busy;							// 1:カウント取込み中
	int mode;							// 0:10ms生データ出力 1:1secごとのファイル出力 2:10msecフィルタデータ出力
	int n;								// 取り込んだデータ数
	int	sec;							// カウント取り込み秒数表示
	int	times;							// カウント取り込み秒数
	int owner;							// 指示コンソールNo.
	int min;							// カウントMin値
	int max;							// カウントMax値
	time_t t;							// カウント取込み開始
	struct timespec tim_start;			// カウント取込み開始
	long tim_start_fpga;				// FPGAのカウント取込み開始時刻
#define	CNT_SZ	100
	long buf[2000*CNT_SZ];				// 2000秒のデータバッファ
	struct timespec tm[2000*CNT_SZ];	// 取込み時刻
	long tm_fpga[2000*CNT_SZ];			// FPGAのカウント時刻
	int	mkflag[2000];
} static cnt_tbl={0, 0, 0};

// フォトンカウント取り込みバッファテーブル
struct _cnt_dev_tbl {
	struct timespec exec_tim;			// 実行時刻
	time_t enable_tim;					// リセット後の取り込み開始時刻
	int	rd;								// 読み込みポインタ
	int	wr;								// 書き込みポインタ
	int	n;								// 受信数
#define	CNT_DEV_SZ	200
	long buf[CNT_DEV_SZ];				// 2秒分のデータバッファ
	struct timespec	tm[CNT_DEV_SZ];		// 取込み時刻
	long tm_fpga[CNT_DEV_SZ];			// FPGAのカウント時刻
} static cnt_dev_tbl={{0,0}, 0, 0, 0, 0};

// 動作シーケンス格納テーブル
struct _action_tbl {
	int line;
	int slvno;
	int mno;
	int act;
	unsigned long move_pulse;
	int start_pulse;
	int max_pulse;
	int st_slope;
	int ed_slope;
	int ratio;
	int dummy1;
	int dummy2;
	int dummy3;
} static action_tbl[CONSOLE_MAX][1000];

// パルスモーター管理テーブル
struct _ppm_ctrl {
	int driving;			// 0:not use 1:init home out 2:init home in 3:init home add 4:init busy 5:standby 0x10:低速駆動中
	long move_pulse;		// 低速ドライブ時の移動パルス数
	int	ratio;
	struct _init_pulse {
		int	init;
		int	home_out;
		int	home_in;
		int	home_add;
	} init_pulse;
	struct _init_dir {
		int	home_out;
		int	home_in;
	} init_dir;
	struct _dir {
		int home;
		int step;
	} dir;
	struct _speed {
		int	start;
		int	max;
		int	acc;
		int	dec;
	} speed;
	int r9;
	int r10;
	int r11;
} static ppm_ctrl[PPM_MAX]={0,0,0,0,0,0,0,0,0,0};


// イベント
static int event[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static struct timespec tim_start;

static void count_dev_rcv(void);


// 現在時刻取得
static struct timespec tim_get_now(void)
{
	struct timespec tim_now;
//	clock_gettime(CLOCK_MONOTONIC_RAW, &tim_now);
	clock_gettime(CLOCK_MONOTONIC, &tim_now);
	return tim_now;
}

// tmにaddを加算する
static struct timespec tim_add(struct timespec tm, long addmsec)
{
	tm.tv_nsec += ((addmsec%1000)*1000000);
	if (tm.tv_nsec>=1000000000) {
		tm.tv_sec += 1;
		tm.tv_nsec -= 1000000000;
	}
	tm.tv_sec += (addmsec/1000);
	return tm;
}

// 引数 t1:経過時刻 t2:起点時刻 msec:観測時間(msec)
// 戻値 1:TimeUP
static int tim_timeup(struct timespec t1, struct timespec t2, long msec)
{
	struct timespec tm = tim_add(t2, msec);
	return (t1.tv_sec>tm.tv_sec || (t1.tv_sec>=tm.tv_sec && t1.tv_nsec>=tm.tv_nsec))?1:0;
}

// 引数 t1:経過時刻 t2:起点時刻
static struct timespec tim_diff(struct timespec t1, struct timespec t2)
{
	struct timespec ret={0,0};
	if (tim_timeup(t1, t2, 0)==0) {
		return ret;
	}
	if (t1.tv_sec>0 && t1.tv_nsec<t2.tv_nsec) {
		t1.tv_nsec += 1000000000;
		t1.tv_sec  -= 1;
	}
	ret.tv_sec = t1.tv_sec-t2.tv_sec;
	ret.tv_nsec = t1.tv_nsec-t2.tv_nsec;
	return ret;
}


// CRC計算
static unsigned char calc_crc(char *sbuf, int size)
{
	int i;
	unsigned char crc = 0x84;

	for (i=1; i<(size-2); i++) {
      crc = ((((crc>>1)|(crc<<7)))&0xff)^sbuf[i];
	}
	return crc;
}

// NT1200(アーキテクト)フォーマットでデータ送信
static void nt_send(int sock, char *sbuf, int size)
{
	sbuf[size-2] = calc_crc(sbuf, size);
	send(sock, sbuf, size, 0);
}

// NT1200(アーキテクト)フォーマットでデータ受信
// 戻値:0:SockClose >0:STX,ETXを除いた受信長 <0:Error
static int nt_recv(int sock, char c, char *p)
{
	struct timespec tim_last=tim_get_now();
	int len, length, i, count=0;
	// STX
	if (c!=STX) return -1;
	p[count++] = c;

	// length
	while (1) {
		len = recv(sock, &c, 1, 0);
		if (len==0) return 0;
		if (len>0) break;
		if (tim_timeup(tim_get_now(), tim_last, TIMEOUT)) return -1;
	}
	length = (int)c;
	p[count++] = c;

	tim_last=tim_get_now();
	for (i=1; i<length; i++) {
		while (1) {
			len = recv(sock, &c, 1, 0);
			if (len==0) return 0;
			if (len>0) break;
			if (tim_timeup(tim_get_now(), tim_last, TIMEOUT)) return -1;
		}
		p[count++] = c;
	}

	// ETX
	tim_last=tim_get_now();
	while (1) {
		len = recv(sock, &c, 1, 0);
		if (len==0) return 0;
		if (len>0) break;
		if (tim_timeup(tim_get_now(), tim_last, TIMEOUT)) return -1;
	}
	if (c!=ETX) return -1;
	p[count++] = c;

	return count;
}

// メッセージ送信
static int message(int sock, int my_thread_no, int disp_no, int line_no, char *str)
{
	msg_msg_data[6] = (char)my_thread_no;
	msg_msg_data[7] = (char)disp_no;
	msg_msg_data[8] = (char)line_no;
	strncpy(&msg_msg_data[9], str, 32);

	nt_send(sock, msg_msg_data, sizeof(msg_msg_data));
	return 0;
}

static int wiringPiSPIDataRW2(int ch, int cs, unsigned char *data, int len)
{
	int i;
	unsigned char c, org=*data;
	*data=0;
	for (i=0; i<8; i++) {
		// SPI1 MOSI
		digitalWrite(GPIO20, (org>>(7-i))&0x01);
		// SPI1 SCLK
		digitalWrite(GPIO21, 0);
		delay(1);
		// SPI1 SCLK
		digitalWrite(GPIO21, 1);
		// SPI1 MISO
		c = digitalRead(GPIO19)&0x01;
		(*data) += (c<<(7-i));
		delay(1);

		// フォトンカウント取り込み
		count_dev_rcv();
	}
}

// 温度設定
static void config_temp(double target)
{
		unsigned char data[4]={0x29,0x40,0x80,0xc0};
		data[2] += (unsigned char)(((unsigned short)TEMP2HEX(target))>>6); 
		data[3] += (unsigned char)(TEMP2HEX(target)&0x3f);
		pthread_mutex_lock(&shm->mutex);
		wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, 4);
		pthread_mutex_unlock(&shm->mutex);
}

// L6470に1byte送信
static unsigned char L6470_write(unsigned char ch, unsigned char data)
{
	unsigned char org=data;
	digitalWrite(L6470_CH[ch], 0);
	wiringPiSPIDataRW2(L6470_SPI_L6470, ch, &data, 1);
	digitalWrite(L6470_CH[ch], 1);
	delay(1);
	return data;
}

// L6470にコマンド送信送信
static long L6470_cmd_write(unsigned char ch, unsigned char no, char bytes, long data)
{
	unsigned char c;
	int i;
	long ret=0L;
	L6470_write(ch, no);
	for (i=bytes-1; i>=0; i--) {
		c = L6470_write(ch, (unsigned char)(data>>(8*i)));
		ret += ((long)c << (8*i));
	}
	return ret;
}

// L6470 パラメータ書き込み
static long L6470_param_write(unsigned char ch, unsigned char no, long data)
{
/* パラメータ毎のbit数定義 */
const int bit_tbl[] = {22, 9, 22, 20, 12, 12, 10, 13, 8, 8, 8, 8, 14, 8, 8, 8, 4, 5, 4, 7, 10, 8, 8, 16, 16};
	int bytes = (bit_tbl[(no&0x1f)-1]+7)/8;
	return L6470_cmd_write(ch, no, bytes, data);
}

// L6470 パラメータ読み出し
static long L6470_param_read(unsigned char ch, unsigned char no)
{
	return L6470_param_write(ch, 0x20|no, 0L);
}

// L6470 駆動確認
static int L6470_busy(unsigned char ch)
{
	return ((L6470_param_read(ch, 0x19)&0x02) == 0x00)?1:0;
}

// L6470 ABS位置をunsigned型で得る
static unsigned long L6470_abs_pos(unsigned char ch)
{
	unsigned long abs_pos=L6470_param_read(ch, 0x01);
	if (abs_pos&0x200000) {
		return (~(abs_pos|0xffc00000))+1;
	}
	return abs_pos;
}

// L6470 初期設定
static int L6470_init(unsigned char ch)
{
	// NOP x4 -> Reset
	L6470_write(ch, 0);
	L6470_write(ch, 0);
	L6470_write(ch, 0);
	L6470_write(ch, 0);
	L6470_write(ch, 0xc0);	// Reset
	L6470_write(ch, 0xc0);	// Reset
	L6470_write(ch, 0xc0);	// Reset
	L6470_write(ch, 0xA8);	// HardHiZ

	// MIN_SPEED設定。
	L6470_param_write(ch, 0x08, 0x15);
	if (L6470_param_read(ch, 0x08)==0) {
		return -1;
	}
	// MAX_SPEED設定。
	L6470_param_write(ch, 0x07, 20);
	// KVAL_HOLD設定。
	L6470_param_write(ch, 0x09, (ppm_ctrl[ch].r10>>8) & 0xff);
	// KVAL_RUN設定。
	L6470_param_write(ch, 0x0A, ppm_ctrl[ch].r10 & 0xff);
	// KVAL_ACC設定。
	L6470_param_write(ch, 0x0B, ppm_ctrl[ch].r10 & 0xff);
	// KVAL_DEC設定。
	L6470_param_write(ch, 0x0C, ppm_ctrl[ch].r10 & 0xff);
	// OCD_TH設定。
	L6470_param_write(ch, 0x13, 0x0f);
	// STALL_TH設定。
	L6470_param_write(ch, 0x14, 0x7f);
	// STEP MODE
	L6470_param_write(ch, 0x16, ppm_ctrl[ch].r9);
	// SW割込み解除
	L6470_param_write(ch, 0x18, L6470_param_read(ch, 0x18)|0x10);
	// SoftStop
	L6470_write(ch, 0xB0);
	return 0;
}

// L6470 スピード変更
static void L6470_change_spd(unsigned char ch, int start_pulse, int max_pulse, int st_slope, int ed_slope)
{
	struct _ppm_ctrl *pctrl = &ppm_ctrl[ch];
	if (pctrl->speed.start!=start_pulse) {
		pctrl->speed.start=start_pulse;
		// MIN_SPEED設定。
		L6470_param_write(ch, 0x08, (long)((double)start_pulse*(250*pow(10,-9))/(pow(2,-24))));
	}
	// start_pulseより小さい場合はstart_pulseの値に矯正する
	if (max_pulse<start_pulse) {
		max_pulse=start_pulse;
	}
	if (pctrl->speed.max!=max_pulse) {
		long l = (long)((double)max_pulse*(250*pow(10,-9))/(pow(2,-18)));
		// MAXは分解能が荒いので設定値より小さくなる場合がある
		long add = (((double)l*pow(2,-18)/(250*pow(10,-9)))<max_pulse) ? 1L:0L;
		pctrl->speed.max=max_pulse;
		// MAX_SPEED設定。
		L6470_param_write(ch, 0x07, l+add);
	}
	if (pctrl->speed.acc!=st_slope) {
		pctrl->speed.acc=st_slope;
		// ACC設定。
		L6470_param_write(ch, 0x05, (long)((double)st_slope*(pow(250*pow(10,-9),2))/(pow(2,-40))));
	}
	if (pctrl->speed.dec!=ed_slope) {
		pctrl->speed.dec=ed_slope;
		// DEC設定。
		L6470_param_write(ch, 0x06, (long)((double)ed_slope*(pow(250*pow(10,-9),2))/(pow(2,-40))));
	}
}

// パルスモーターの初期化動作
// 戻値: 0:初期化完了 1:初期化中 -1:Home Out Err -2:Home In Err -3:モーター未応答
static int ppm_init(int ch)
{
	struct _ppm_ctrl *pctrl = &ppm_ctrl[ch];
	int sts_port     = L6470_param_read(ch, 0x19);
	int busy_flag    = ((sts_port&0x02) == 0x00)?1:0;
	int home_in_flag = ((sts_port&0x04) == 0x00)?1:0;

	if (sts_port==0) return -3;
	if (busy_flag) return 1;

	switch (pctrl->driving) {
	case 0:		// 0:not use
	case 5:		// 5:standby
		pctrl->speed.start=pctrl->init_pulse.init;
		pctrl->speed.max  =pctrl->init_pulse.init;
		pctrl->speed.acc  =1;
		pctrl->speed.dec  =1;
		// 初期化動作のMIN_SPEED設定。
		L6470_param_write(ch, 0x08, (long)((double)pctrl->speed.start*(250*pow(10,-9))/(pow(2,-24))));
		// 初期化動作のMAX_SPEED設定。
		L6470_param_write(ch, 0x07, (long)((double)pctrl->speed.max*(250*pow(10,-9))/(pow(2,-18))));
		// ACC設定。
		L6470_param_write(ch, 0x05, (long)((double)pctrl->speed.acc*(pow(250*pow(10,-9),2))/(pow(2,-40))));
		// DEC設定。
		L6470_param_write(ch, 0x06, (long)((double)pctrl->speed.dec*(pow(250*pow(10,-9),2))/(pow(2,-40))));

		// home out
		if (home_in_flag) {
			pctrl->driving=1;
		}
		// home in
		else{
			pctrl->driving=2;
		}
		break;
	case 1:		// 1:init home out
		{
			unsigned char cmd = (pctrl->init_dir.home_out)?0x40:0x41;
			L6470_cmd_write(ch, cmd, 3, pctrl->init_pulse.home_out);//Move
			pctrl->driving=2;
		}
		break;
	case 2:		// 2:init home in
		{
			unsigned char cmd = (pctrl->init_dir.home_in)?0x92:0x93;
			/* HOMEチェック */
			if (home_in_flag) {
				pctrl->driving=0;
				return -1;
			}
			L6470_write(ch, cmd);//ReleseSW
			pctrl->driving=3;
		}
		break;
	case 3:		// 3:init home add
		{
			unsigned char cmd = (pctrl->init_dir.home_in)?0x40:0x41;
			/* HOMEチェック */
			if (!home_in_flag) {
				pctrl->driving=0;
				return -2;
			}
			L6470_cmd_write(ch, cmd, 3, pctrl->init_pulse.home_add);//Move
			pctrl->driving=4;
			L6470_write(ch, 0xD8);// ResetPos
		}
		break;
	case 4:		// 4:init home add busy
		pctrl->driving=5;
		break;
	default:
		break;
	}
	return (pctrl->driving==5) ? 0:1;
}

// フォトンカウント取り込み 10msec周期に取り込む
static void count_dev_rcv(void)
{
	struct timespec tim_now=tim_get_now();
	int over_led=0;
	// 10msec毎にフォトンカウント取り込み
	if (tim_timeup(tim_now, cnt_dev_tbl.exec_tim, 0)) {
		unsigned char data1[4]={0x14,0x40,0x80,0xc0};// フォトンカウント(FPGAは10msec周期でPMTのフォトンをカウントしSPIで送る)
		unsigned char data2[4]={0x15,0x40,0x80,0xc0};// FPGAのカウント時刻(FPGAはフォトンをカウントする度にこの値を1増やす 範囲は0-3ffffh)
		pthread_mutex_lock(&shm->mutex);
		wiringPiSPIDataRW(MAX_SPI_CHANNEL, data1, 4);
		wiringPiSPIDataRW(MAX_SPI_CHANNEL, data2, 4);
		pthread_mutex_unlock(&shm->mutex);

		// FPGAの取込み開始時刻として記録
		if (cnt_tbl.tim_start_fpga<0) {
			cnt_tbl.tim_start_fpga = ((data2[1]&0x3f)<<12)+((data2[2]&0x3f)<<6)+(data2[3]&0x3f);
		}
		// エラー発生
		if (data1[0]&0x20) {
			unsigned char data3[4]={0x1f,0x40,0x80,0xc0};
			unsigned char data4[4]={0x3f,0x40,0x80,0xc1};
			pthread_mutex_lock(&shm->mutex);
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, data3, 4);// error flag
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, data4, 4);// reset
			pthread_mutex_unlock(&shm->mutex);
			// 過大光判定
			over_led = (data3[3]&0x02)?1:0;
		}
//printf("%s %d %d %02X %02X %8d %8d %10d %10d %10d %10d\n", __FILE__, __LINE__, over_led, data1[0]&0x02, data1[0]&0x04, tim_now.tv_sec, cnt_dev_tbl.exec_tim.tv_sec, tim_now.tv_nsec, cnt_dev_tbl.exec_tim.tv_nsec, ((data1[1]&0x3f)<<12)+((data1[2]&0x3f)<<6)+(data1[3]&0x3f), ((data2[1]&0x3f)<<12)+((data2[2]&0x3f)<<6)+(data2[3]&0x3f));
		// 過大光の場合
		if (over_led) {
// 過大光検知とカウント取り込みタイミングは連動していないためコメントにする
//			data1[1] = 0x7f;
//			data1[2] = 0xbf;
//			data1[3] = 0xff;
		}
		// キャリー
		else if (data1[0]&0x08) {
			data1[1] = 0x7f;
			data1[2] = 0xbf;
			data1[3] = 0xfe;
		}
		// オーバーフロー
		else if (data1[0]&0x04) {
//printf("%s %d %8d %8d %10d %10d %6d %02X %02X %02X %02X\n", __FILE__, __LINE__, tim_bk.tv_sec, tim_now.tv_sec, tim_bk.tv_nsec, tim_now.tv_nsec, cnt_dev_tbl.n, data[0], data[1], data[2], data[3]);
//			data1[1] = 0x7f;
//			data1[2] = 0xbf;
//			data1[3] = 0xfd;
		}

	// エラー発生時は取り込まない場合
	//	if ((data1[0]&0x2C)==0) {
//printf("%s %d %d\n", __FILE__, __LINE__, cnt_dev_tbl.n);
		// (過大光の場合はカウントの最上位ビットを1にする)
			cnt_dev_tbl.buf[cnt_dev_tbl.wr]=((data1[1]&0x3f)<<12)+((data1[2]&0x3f)<<6)+(data1[3]&0x3f) + ((over_led)?0x80000000:0);
			cnt_dev_tbl.tm_fpga[cnt_dev_tbl.wr]=((data2[1]&0x3f)<<12)+((data2[2]&0x3f)<<6)+(data2[3]&0x3f);
			cnt_dev_tbl.tm[cnt_dev_tbl.wr]=tim_now;

			cnt_dev_tbl.wr = (cnt_dev_tbl.wr+1)%CNT_DEV_SZ;

			cnt_dev_tbl.n  += 1;
			if (cnt_dev_tbl.n>CNT_DEV_SZ) {
				cnt_dev_tbl.rd = (cnt_dev_tbl.rd+1)%CNT_DEV_SZ;
				cnt_dev_tbl.n  = CNT_DEV_SZ;
			}
	//	}
		// 次回10msec後
		cnt_dev_tbl.exec_tim=tim_add(cnt_dev_tbl.exec_tim, 10);
	}
}

// フォトンカウント取り込みバッファテーブルのクリア
// 戻値:取込み開始時刻
static struct timespec count_dev_reset(void)
{
	struct timespec ret;

	// リセット
	cnt_dev_tbl.rd = 0;
	cnt_dev_tbl.wr = 0;
	cnt_dev_tbl.n  = 0;
	cnt_dev_tbl.exec_tim=tim_add(tim_get_now(), 10);
	cnt_dev_tbl.enable_tim=cnt_dev_tbl.exec_tim.tv_sec+1;

	ret.tv_sec=cnt_dev_tbl.enable_tim;
	ret.tv_nsec=0;
	return ret;
}

// フォトンカウント取り込みバッファ取り込み数
static int count_dev_n(void)
{
	return cnt_dev_tbl.n;
}

// フォトンカウント取り込みバッファ読み出し
static int count_dev_read(long *ptr, struct timespec *tm, long *tm_fpga, int len)
{
	int n=0;
	while (len>0 && cnt_dev_tbl.n>0) {
		*ptr++ = cnt_dev_tbl.buf[cnt_dev_tbl.rd];
		if (tm!=NULL) {
			*tm++ = cnt_dev_tbl.tm[cnt_dev_tbl.rd];
		}
		if (tm_fpga!=NULL) {
			*tm_fpga++ = cnt_dev_tbl.tm_fpga[cnt_dev_tbl.rd];
		}
//printf("%s %d %4d %02X %d\n", __FILE__, __LINE__, len, cnt_dev_tbl.buf[cnt_dev_tbl.rd]&0x3f, cnt_dev_tbl.rd);
		cnt_dev_tbl.rd = (cnt_dev_tbl.rd+1)%CNT_DEV_SZ;
		cnt_dev_tbl.n--;
		n++;
		len--;
	}
	return n;
}

// buf内のカウント値の平均を求める
static long get_1st_data(long buf[], int n, int *psts)
{
	unsigned long total=0L;
	int i, m;
	*psts=0;
//printf("%s %d %d\n", __FILE__, __LINE__, n);
	for (i=0, m=0; i<n; i++) {
//printf("%s %d %4d %4d %10x\n", __FILE__, __LINE__, i, n, buf[i]);
		//過大光確認
		if (buf[i]&0x80000000) {
			*psts=1;
		}
		total += (buf[i]&0x7fffffff);
		m++;
	}
	return GATE_COUNT(total/m);
}

// 0秒からのFPGA時刻に変換する
static long fpga_time(long tm)
{
	long ret=tm-cnt_tbl.tim_start_fpga;
	// 最大時刻は18bit(0x3ffff)
	if (cnt_tbl.tim_start_fpga>tm) {
		ret=0x3ffff-cnt_tbl.tim_start_fpga+tm+1;
	}
	return ret;
}

// カウント値をファイルに保存する
static int count_save(char *str)
{
const double f11=0.000473, f12=-0.9391, f21=0.000483, f22=1.938145;
	unsigned long dat_bk1=0L, dat_bk2=0L;
	double e1=0.0, e2=0.0;
	int ret=-1;
	int i, j;
	FILE *fp;
	strftime(str, 32, "/tmp/%y%m%d%H%M%S.csv", localtime(&cnt_tbl.t));
	fp=fopen(str, "w");
//printf("%s %d %4d\n", __FILE__, __LINE__, cnt_tbl.n);
	if (fp) {
		// header
		if (cnt_tbl.mode==1) {
			fprintf(fp, "no.,count,flag,n,light,time\r\n");
		}
		for (i=0, j=0; i<cnt_tbl.n; j++) {
			unsigned long total=0L;
			int m=0;
			int over_led=0;
//printf("%s %d %4d\n", __FILE__, __LINE__, i);
			while (i<cnt_tbl.n && (fpga_time(cnt_tbl.tm_fpga[i])/100)==j) {
				//過大光確認(count_dev_rcv関数でビット入れ替え操作)
				over_led = (cnt_tbl.buf[i]&0x80000000)?1:0;
				cnt_tbl.buf[i] &= 0x7fffffff;
				// 10ms生データ出力
				if (cnt_tbl.mode==0) {
//					fprintf(fp, "%4d,%10ld\r\n", i+1, dat);
					fprintf(fp, "0,%4d,%10ld,%d\r\n", i+1, cnt_tbl.buf[i], cnt_tbl.tm_fpga[i]);
				}
				// 10msecフィルタデータ出力
				else if (cnt_tbl.mode==2) {
					double e = f22*(double)e1 + f12*(double)e2 + f21*(double)dat_bk1 + f11*(double)dat_bk2;
					fprintf(fp, "%4d,%10.3lf\r\n", i+1, e);
					dat_bk2=dat_bk1;
					dat_bk1=cnt_tbl.buf[i];
					e2 = e1;
					e1 = e;
				}
				// min以上max以下の値の平均値を1秒間の測定値とする
				if (cnt_tbl.buf[i]>=cnt_tbl.min && cnt_tbl.buf[i]<=cnt_tbl.max) {
					total += cnt_tbl.buf[i];
					m++;
				}
				i++;
			}
			// 1secごとのファイル出力
			if (cnt_tbl.mode==1) {
				fprintf(fp, "%4d,%10ld,%d,%d,%d,%d\r\n", j+1, (m>0)?GATE_COUNT(total/m):0L, cnt_tbl.mkflag[j-1], m, over_led, cnt_tbl.tm_fpga[i-1]);
			}
			// 10ms生データ出力
			else if (cnt_tbl.mode==0) {
				fprintf(fp, "1,%4d,%10ld,%d,%d,%d,%d\r\n", j+1, (m>0)?GATE_COUNT(total/m):0L, cnt_tbl.mkflag[j-1], m, over_led, cnt_tbl.tm_fpga[i-1]);
			}
		}
		fclose(fp);
		ret=0;
	}
	return ret;
}

// シーケンス
static int sequence(int sock, int no)
{
	int i, error=0;
	char str[32];
	struct _seq_tbl *pseq = &seq_tbl[no-1];
	struct _action_tbl *pact = &action_tbl[no-1][pseq->current];
	int local_ret_line       = pact->line+1;

	switch (pact->act) {
	case 0x01:		// DCモーター初期化
	case 0x02:		// DCモーターCW(回転時間指定)
	case 0x03:		// DCモーターCW(STEPセンサーまで回転)
	case 0x04:		// DCモーターCCW(回転時間指定)
	case 0x05:		// DCモーターCCW(HOMEセンサーまで回転)
		{
#if 0
			/* Action */
			if (can_action_send(can, pseq->current, mno, pact)) {
				sprintf(str, "ERR 行番号 = %d CAN Error", pact->line);
				message(sock, no, 1, 1, str);
				pseq->run = 0;
			}
			else{
				pseq->slv_busy[pact->slvno]=2;
			}
#endif
			pseq->current++;
		}
		break;
	case 0x11:		// パルスモーター初期化
		switch (ppm_init(pact->mno-1)) {
		case 0:
			pseq->current++;
			break;
		case -1:
			pseq->run = 0;
			message(sock, no, 1, 1, "ERR HOME Out Error");
			L6470_write(pact->mno-1, 0xB8);	// HardStop
			break;
		case -2:
			pseq->run = 0;
			message(sock, no, 1, 1, "ERR HOME In Error");
			L6470_write(pact->mno-1, 0xB8);	// HardStop
			break;
		case -3:
			pseq->run = 0;
			message(sock, no, 1, 1, "ERR モーター未応答");
			L6470_write(pact->mno-1, 0xB8);	// HardStop
			break;
		default:
			break;
		}
		break;
	case 0x12:		// パルスモーターSTEP(相対パルス指定)
		if (!L6470_busy(pact->mno-1)) {
			if (pact->max_pulse>=15) {
				unsigned char cmd = (ppm_ctrl[pact->mno-1].dir.step)?0x40:0x41;
				L6470_change_spd(pact->mno-1, pact->start_pulse, pact->max_pulse, pact->st_slope, pact->ed_slope);
				L6470_cmd_write(pact->mno-1, cmd, 3, pact->move_pulse);//Move
			}
			// 低速ドライブ
			else{
				struct _ppm_ctrl *pctrl = &ppm_ctrl[pact->mno-1];
				unsigned char cmd = (pctrl->dir.step)?0x50:0x51;
				unsigned long abs_pos=L6470_abs_pos(pact->mno-1);
				pctrl->move_pulse = abs_pos+(long)pact->move_pulse;
				pctrl->driving    = 0x10;
				L6470_change_spd(pact->mno-1, 0, 0x3ff, 0, 0);
				L6470_cmd_write(pact->mno-1, cmd, 3, (long)((double)pact->max_pulse*(250*pow(10,-9))/(pow(2,-28))));//Run
			}
			pseq->current++;
		}
		break;
	case 0x13:		// パルスモーターHOME(相対パルス指定)
		if (!L6470_busy(pact->mno-1)) {
			if (pact->max_pulse>=15) {
				unsigned char cmd = (ppm_ctrl[pact->mno-1].dir.home)?0x40:0x41;
				L6470_change_spd(pact->mno-1, pact->start_pulse, pact->max_pulse, pact->st_slope, pact->ed_slope);
				L6470_cmd_write(pact->mno-1, cmd, 3, pact->move_pulse);//Move
			}
			// 低速ドライブ
			else{
				struct _ppm_ctrl *pctrl = &ppm_ctrl[pact->mno-1];
				unsigned char cmd = (pctrl->dir.home)?0x50:0x51;
				unsigned long abs_pos=L6470_abs_pos(pact->mno-1);
				pctrl->move_pulse = abs_pos-(long)pact->move_pulse;
				pctrl->driving    = 0x11;
				L6470_change_spd(pact->mno-1, 0, 0x3ff, 0, 0);
				L6470_cmd_write(pact->mno-1, cmd, 3, (long)((double)pact->max_pulse*(250*pow(10,-9))/(pow(2,-28))));//Run
			}
			pseq->current++;
		}
		break;
	case 0x14:		// パルスモーターHOME(現在位置パルス分をCCW回転する)
		if (!L6470_busy(pact->mno-1)) {
			unsigned char cmd = (ppm_ctrl[pact->mno-1].dir.home)?0x40:0x41;
			unsigned long abs_pos=L6470_abs_pos(pact->mno-1);
			L6470_change_spd(pact->mno-1, pact->start_pulse, pact->max_pulse, pact->st_slope, pact->ed_slope);
			//L6470_write(pact->mno-1, 0x70);//GoHome
			L6470_cmd_write(pact->mno-1, cmd, 3, abs_pos);//Move
//			delay(10);
			pseq->current++;
		}
		break;
	case 0x15:		// パルスモーター絶対パルス移動
		if (!L6470_busy(pact->mno-1)) {
			L6470_change_spd(pact->mno-1, pact->start_pulse, pact->max_pulse, pact->st_slope, pact->ed_slope);
			L6470_cmd_write(pact->mno-1, 0x60, 3, pact->move_pulse);//GoTo
//			delay(10);
			pseq->current++;
		}
		break;
	case 0x16:		// パルスモーター停止までまつ
		if (ppm_ctrl[pact->mno-1].driving==0x10 || ppm_ctrl[pact->mno-1].driving==0x11) {
			struct _ppm_ctrl *pctrl = &ppm_ctrl[pact->mno-1];
			unsigned long abs_pos=L6470_abs_pos(pact->mno-1);
			// STEP
			if (ppm_ctrl[pact->mno-1].driving==0x10 && pctrl->move_pulse<=abs_pos) {
				pctrl->driving=0;
				L6470_write(pact->mno-1, 0xB8);	// HardStop
				pseq->current++;
			}
			// HOME
			else if (ppm_ctrl[pact->mno-1].driving==0x11 && pctrl->move_pulse>=abs_pos) {
				pctrl->driving=0;
				L6470_write(pact->mno-1, 0xB8);	// HardStop
				pseq->current++;
			}
		}
		else if (!L6470_busy(pact->mno-1)) {
			pseq->current++;
		}
		break;
	case 0x17:		// パルスモーター励磁解除
		L6470_write(pact->mno-1, 0xA8);	// HardHiZ
		pseq->current++;
		break;
	case 0x18:		// パルスモーター励磁ON
		L6470_write(pact->mno-1, 0xB0);	// SoftStop
		pseq->current++;
		break;
//	case 0x31:		// パルスモーターI/O指定ビットON
//	case 0x32:		// パルスモーターI/O指定ビットOFF
//	case 0x33:		// パルスモーターI/O指定ビットをONまでまつ
//	case 0x34:		// パルスモーターI/O指定ビットをOFFまでまつ
//	case 0x35:		// パルスモーターI/O 16bitデータ書き込み
//	case 0x36:		// パルスモーターI/O 16bitデータ読み込んで表示
#if 0
		{
			/* パルスモーター停止までまつ */
			if (pact->act==0x16) {
				// 何もしない
			}
			/* Action */
			else if (can_action_send(can, pseq->current, mno, pact)) {
				sprintf(str, "ERR 行番号 = %d CAN Error", pact->line);
				message(sock, no, 1, 1, str);
				pseq->run = 0;
			}
			else{
				pseq->slv_busy[pact->slvno]=1;
			}
			pseq->current++;
		}
		break;
#endif
	case 0x41:		// DIO指定ビットON
		digitalWrite(GPIO_CH[pact->mno-1], 1);
		pseq->current++;
		break;
	case 0x42:		// DIO指定ビットOFF
		digitalWrite(GPIO_CH[pact->mno-1], 0);
		pseq->current++;
		break;
	case 0x43:		// DIO指定ビットをONまでまつ
		if (digitalRead(GPIO_CH[pact->mno-1]) == 1) {
			pseq->current++;
		}
		break;
	case 0x44:		// DIO指定ビットをOFFまでまつ
		if (digitalRead(GPIO_CH[pact->mno-1]) == 0) {
			pseq->current++;
		}
		break;
	case 0x46:		// DIO ポートREAD=RegA
		pseq->reg_flag = digitalRead(GPIO_CH[pact->mno-1] == 1);
		pseq->current++;
		sprintf(str, "RegA=%XH", pseq->reg_flag);
		message(sock, no, 1, 2, str);
		break;
	case 0x51:		// 指定時間まち(×10msec)
		if (pseq->busy==0) {
			pseq->busy=1;
			pseq->wai_start=tim_get_now();
		}
		else{
			struct timespec tim_now=tim_get_now();
			if (tim_timeup(tim_now, pseq->wai_start, pact->move_pulse)) {
				struct timespec tim=tim_diff(tim_now, pseq->wai_start);
				sprintf(str, "Loop:%d Time:%lu.%lu秒", pseq->count+1, tim.tv_sec, tim.tv_nsec/1000000);
				pseq->busy=0;
				pseq->current++;
			}
		}
		break;
	case 0x52:		// if文
		/* 条件が揃えば指定行へ */
		if (((unsigned short)pseq->reg_flag & (unsigned short)pact->start_pulse)==(unsigned short)pact->max_pulse) {
			pseq->current++;
			local_ret_line=pact->move_pulse;

       		for (i=0; i<pseq->max_line; i++) {
				struct _action_tbl *next = &action_tbl[no-1][i];
				if (pact->move_pulse==next->line) {
					pseq->current = i;
    	           	break;
        	   	}
       		}
		}
		/* 次の行へ */
		else{
			pseq->current++;
		}
		break;
	case 0x53:		// unless文
		if (((unsigned short)pseq->reg_flag & (unsigned short)pact->start_pulse)!=(unsigned short)pact->max_pulse) {
			pseq->current++;
			local_ret_line=pact->move_pulse;

       		for (i=0; i<pseq->max_line; i++) {
				struct _action_tbl *next = &action_tbl[no-1][i];
				if (pact->move_pulse==next->line) {
					pseq->current = i;
    	           	break;
        	   	}
       		}
		}
		/* 次の行へ */
		else{
			pseq->current++;
		}
		break;
	case 0x54:		// 最終行へ移動
		if (pseq->max_line>0 && pseq->ret_line==0) {
			pseq->current = pseq->max_line-1;
		}
		else{
			pseq->current++;
		}
		local_ret_line=-1;
		break;
	case 0x55:		// 指定行へ移動
		pseq->current++;
		local_ret_line=pact->move_pulse;

       	for (i=0; i<pseq->max_line; i++) {
			struct _action_tbl *next = &action_tbl[no-1][i];
			if (pact->move_pulse==next->line) {
				pseq->current = i;
               	break;
           	}
       	}
		break;
	case 0x56:		// SIO送信
		break;
	case 0x57:		// エラー停止
		sprintf(str, "ERR 行番号 = %d", pact->line);
		message(sock, no, 1, 1, str);
		error=1;
		break;
	case 0x61:		// イベントセット
        event[pact->move_pulse-1]=1;
		pseq->current++;
		break;
	case 0x62:		// イベントクリア
        event[pact->move_pulse-1]=0;
		pseq->current++;
		break;
	case 0x63:		// イベントセットまち
        if (event[pact->move_pulse-1]) {
		    pseq->current++;
        }
		break;
	case 0x64:		// いべんとクリアまち
        if (!event[pact->move_pulse-1]) {
		    pseq->current++;
        }
		break;
	case 0x71:		// カウント取り込み
	case 0x72:		// カウント取り込み(10msec)
	case 0x73:		// カウント取込(フィルタ)
		{
			time_t t = time(NULL);
			cnt_tbl.busy     =1;
			switch (pact->act) {
			case 0x71: // カウント取込み',
				cnt_tbl.mode = 1;
				break;
			case 0x72: // カウント取込(10ms)',
				cnt_tbl.mode = 0;
				break;
			case 0x73: // カウント取込(フィルタ)',
				cnt_tbl.mode = 2;
				break;
			}
			cnt_tbl.n 	     =0;
			cnt_tbl.sec      =0;
			cnt_tbl.times    =pact->move_pulse;
			cnt_tbl.owner    =no;
			cnt_tbl.min      =pact->start_pulse + (pact->max_pulse<<16);
			cnt_tbl.max      =pact->st_slope + (pact->ed_slope<<16);

			cnt_tbl.t = t;
			cnt_tbl.tim_start=count_dev_reset();	// リセット&取込み開始時刻の取得
			cnt_tbl.tim_start_fpga = -1;
			memset(cnt_tbl.mkflag, 0, sizeof(cnt_tbl.mkflag));
		}
		pseq->current++;
		break;
	case 0x74:		// カウント取り込み終了まち
		if (cnt_tbl.busy==0) {
			pseq->current++;
		}
		break;
	case 0x75:		// マーキングフラグ
		cnt_tbl.mkflag[cnt_tbl.sec]=pact->move_pulse;
		pseq->current++;
		break;
	case 0x81:		// 音声1
		if (fork()==0) {
			execl("/usr/bin/mpg321", "mpg321", "sound1.mp3");
			exit(0);
		}
		pseq->current++;
		break;
	case 0x82:		// 音声2
		if (fork()==0) {
			execl("/usr/bin/mpg321", "mpg321", "sound2.mp3");
			exit(0);
		}
		pseq->current++;
		break;
	case 0x83:		// 音声3
		if (fork()==0) {
			execl("/usr/bin/mpg321", "mpg321", "sound3.mp3");
			exit(0);
		}
		pseq->current++;
		break;
	case 0x91:		// LED ON
	    // LED電流セット
		{
			unsigned char data[4]={0x28,0x40,0x80|((pact->move_pulse>>6)&0x3f), 0xc0|(pact->move_pulse&0x3f)};
			pthread_mutex_lock(&shm->mutex);
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, 4);
			pthread_mutex_unlock(&shm->mutex);
		}
        // 励起光をONにする
		{
        	// まずレジスタ2を読込む
			unsigned char data[4]={0x2,0x40,0x80,0xc0};
			pthread_mutex_lock(&shm->mutex);
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, 4);
        	// レジスタ2へONを書込む
			data[0]=0x22;
			data[3]=data[3]|0x01;
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, 4);
			pthread_mutex_unlock(&shm->mutex);
		}
		pseq->current++;
		break;
	case 0x92:		// LED OFF
        // 励起光をOFFにする
		{
        	// まずレジスタ2を読込む
			unsigned char data[4]={0x2,0x40,0x80,0xc0};
			pthread_mutex_lock(&shm->mutex);
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, 4);
        	// レジスタ2へOFFを書込む
			data[0]=0x22;
			data[3]=data[3]&~0x01;
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, 4);
			pthread_mutex_unlock(&shm->mutex);
		}
		pseq->current++;
		break;
	case 0x93:		// ポンプOFF
		{
			unsigned char dataW[4]={0x2f,0x40,0x80,0xc0};
			unsigned char dataR[4]={0x0f,0x40,0x80,0xc0};
			pthread_mutex_lock(&shm->mutex);
			// duty読出し
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, dataR, 4);
			// duty値セット
			dataW[2] = dataR[2];
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, dataW, 4);
			pthread_mutex_unlock(&shm->mutex);
		}
		pseq->current++;
		break;
	case 0x94:		// ポンプ吐出
		{
			unsigned char dataW1[4]={0x2f,0x40,0x80,0xc0};
			unsigned char dataW2[4]={0x2f,0x42,0x80,0xc0|(pact->move_pulse&0x3f)};
			unsigned char dataR[4]={0x0f,0x40,0x80,0xc0};
			pthread_mutex_lock(&shm->mutex);
			// duty読出し
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, dataR, 4);
			// duty値セット
			dataW1[2] = dataR[2];
			dataW2[2] = dataR[2];
			// まずSTOP
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, dataW1, 4);
			usleep(1000);
			// 次に吐出	
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, dataW2, 4);
			pthread_mutex_unlock(&shm->mutex);
		}
		pseq->current++;
		break;
	case 0x95:		// ポンプ連続
		{
			unsigned char dataW[4]={0x2f,0x41,0x80, 0xc0};
			unsigned char dataR[4]={0x0f,0x40,0x80,0xc0};
			pthread_mutex_lock(&shm->mutex);
			// duty読出し
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, dataR, 4);
			// duty値セット
			dataW[2] = dataR[2];
			wiringPiSPIDataRW(MAX_SPI_CHANNEL, dataW, 4);
			pthread_mutex_unlock(&shm->mutex);
		}
		pseq->current++;
		break;
	default:
		pseq->current++;
		break;
	}

	// カウント取込み処理
	if (cnt_tbl.busy==1) {
		// カウント取込み
		int n=count_dev_n();
		if ((cnt_tbl.n+n)<(2000*CNT_SZ)) {
			count_dev_read(&cnt_tbl.buf[cnt_tbl.n], &cnt_tbl.tm[cnt_tbl.n], &cnt_tbl.tm_fpga[cnt_tbl.n], n);
			cnt_tbl.n+=n;
		}

		// 取り込み終了判定
		if (tim_timeup(tim_get_now(), cnt_tbl.tim_start, (cnt_tbl.times+1)*1000)) {
			cnt_tbl.busy=0;
		}
		// 1秒ごとの表示
		else if (tim_timeup(tim_get_now(), cnt_tbl.tim_start, (cnt_tbl.sec+1)*1000)) {
			int sts=0;
			shm->count=(cnt_tbl.n>100)?get_1st_data(&cnt_tbl.buf[cnt_tbl.n-100], 100, &sts):0L;
			cnt_tbl.sec += 1;
			sprintf(str, "取込:%lu秒 :%ld %s", cnt_tbl.sec, shm->count, (sts)?"Ov":"");
			message(sock, no, 1, 3, str);
			// ベース画面への表示	
			sprintf(str, "%d℃   取込:%lu秒 :%ld %s", temp, cnt_tbl.sec, shm->count, (sts)?"Ov":"");
			message(sock, 0, 1, 1, str);
		}
	}

	// 実行行の出力
	if (pseq->run_line!=pact->line) {
		sprintf(str, "%d", pact->line);
		message(sock, no, 3, 1, str);
	}
	pseq->run_line=pact->line;

	// 終了判定
	if (pseq->current >= pseq->max_line || error) {
		/* 経過時間を表示する */
		struct timespec tim=tim_diff(tim_get_now(), tim_start);
		sprintf(str, "Loop:%d Time:%lu.%lu秒", pseq->count+1, tim.tv_sec, tim.tv_nsec/1000000);
		message(sock, no, 1, 2, str);

		pseq->count++;
		// 終了
		if (pseq->count >= pseq->run_times || error) {
			pseq->run = 0;
			if (error==0) {
				message(sock, no, 1, 1, "success!!");
			}
			/* 次の行番号を送る */
			if (pseq->ret_line) {
				sprintf(str, "%d", local_ret_line);
				message(sock, no, 2, 0, str);
			}
			// カウント値取得済であればファイルに保存する
			if (cnt_tbl.owner==no && cnt_tbl.n>0) {
				char tmp[32];
				cnt_tbl.busy=0;
				if (count_save(tmp)==0) {
					sprintf(str, "FILE %s", tmp);
					message(sock, no, 1, 1, str);
				}
				cnt_tbl.n=0;
			}
		}
		// Next
		else{
			pseq->current = 0;
		}
	}
	return 0;
}

// コマンド受信処理
static int dispatch(int sock, char *buf)
{
static int local_param_err=0;		// 1:DC、PPMパラメータ送信エラー発生
static int local_reg_flag[CONSOLE_MAX]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	char str[32];
	unsigned short id = ((unsigned short)((unsigned char)buf[4])<<8) + (unsigned short)((unsigned char)buf[5]);
	int i, run_flag=0;

	for (i=0; i<CONSOLE_MAX; i++) {
		if (seq_tbl[i].run) {
			run_flag=1;
			break;
		}
	}

	// パルスモーター設定
	if (id==0xC101 && run_flag==0) {
		unsigned char no = (unsigned char)buf[7];
		struct _ppm_ctrl *pctrl = &ppm_ctrl[no-1];
		pctrl->move_pulse          = 0;
		pctrl->ratio               = ((int)((unsigned char)buf[10])<<8) + (int)((unsigned char)buf[11]);
		pctrl->init_pulse.init     = ((int)((unsigned char)buf[ 8])<<8) + (int)((unsigned char)buf[ 9]);
		pctrl->init_pulse.home_out = ((int)((unsigned char)buf[12])<<8) + (int)((unsigned char)buf[13]);
		pctrl->init_pulse.home_in  = 0;
		pctrl->init_pulse.home_add = ((int)((unsigned char)buf[14])<<8) + (int)((unsigned char)buf[15]);
		pctrl->init_dir.home_out   = (int)((unsigned char)buf[17]);
		pctrl->init_dir.home_in    = (int)((unsigned char)buf[18]);
		pctrl->dir.home            = (int)((unsigned char)buf[20]);
		pctrl->dir.step            = (int)((unsigned char)buf[19]);
		pctrl->r9   = ((int)((unsigned char)buf[21])<<24) + ((int)((unsigned char)buf[22])<<16) + ((int)((unsigned char)buf[23])<<8) + (int)((unsigned char)buf[24]);
		pctrl->r10  = ((int)((unsigned char)buf[25])<<24) + ((int)((unsigned char)buf[26])<<16) + ((int)((unsigned char)buf[27])<<8) + (int)((unsigned char)buf[28]);
		pctrl->r11  = ((int)((unsigned char)buf[29])<<24) + ((int)((unsigned char)buf[30])<<16) + ((int)((unsigned char)buf[31])<<8) + (int)((unsigned char)buf[32]);
		pctrl->speed.start =0;
		pctrl->speed.max   =0;
		pctrl->speed.acc   =0;
		pctrl->speed.dec   =0;
		if (L6470_init(no-1)) {
			sprintf(str, "ERR モーター未応答 %d", no);
			message(sock, 0, 1, 1, str);
		}
	}
	// DCモーター設定
	else if (id==0xC103 && run_flag==0) {
	}
	// 動作シーケンス
	else if (id==0xC102 && run_flag==0) {
		int no = (int)((unsigned char)buf[3]);
		struct _seq_tbl *pseq = &seq_tbl[no-1];
		struct _action_tbl *pact = &action_tbl[no-1][pseq->max_line];
		pact->line        = ((int)((unsigned char)buf[6])<<8) + (int)((unsigned char)buf[7]);
		pact->slvno       = (int)((unsigned char)buf[8]);
		pact->mno         = (int)((unsigned char)buf[9]);
		pact->act         = ((int)((unsigned char)buf[10])<<8) + (int)((unsigned char)buf[11]);
		pact->move_pulse  = ((unsigned long)((unsigned char)buf[12])<<24) + ((unsigned long)((unsigned char)buf[13])<<16) + ((unsigned long)((unsigned char)buf[14])<<8) + (unsigned long)((unsigned char)buf[15]);
		pact->start_pulse = ((int)((unsigned char)buf[16])<<8) + (int)((unsigned char)buf[17]);
		pact->max_pulse   = ((int)((unsigned char)buf[18])<<8) + (int)((unsigned char)buf[19]);
		pact->st_slope    = ((int)((unsigned char)buf[20])<<8) + (int)((unsigned char)buf[21]);
		pact->ed_slope    = ((int)((unsigned char)buf[22])<<8) + (int)((unsigned char)buf[23]);
		pact->ratio       = ((int)((unsigned char)buf[24])<<8) + (int)((unsigned char)buf[25]);
		pseq->max_line++;
	}
	// 動作準備
	else if (id==0xC014) {
		int no = (int)((unsigned char)buf[6]);
		struct _seq_tbl *pseq = &seq_tbl[no-1];
		if (pseq->run==0) {
			cnt_tbl.n=0;
			local_reg_flag[no-1]=pseq->reg_flag;			// レジスタ値を保存
			memset(pseq, 0, sizeof(seq_tbl));
			memset(event, 0, sizeof(event));
			for (i=0; i<PPM_MAX; i++) {
				ppm_ctrl[i].driving=0;
			}
			message(sock, no, 1, 1, "START");
			message(sock, no, 1, 2, "");
			message(sock, no, 1, 3, "");
		}
	}
	// 動作開始
	else if (id==0xC015) {
		int no = (int)((unsigned char)buf[6]);
		struct _seq_tbl *pseq = &seq_tbl[no-1];
		if (pseq->run) {
			// 動作中の場合は何もしない
		}
		else if (local_param_err) {
			message(sock, no, 1, 1, "ERR DC/PPM Param Send Error");
		}
		else{
			pseq->ret_line     = (int)((unsigned char)buf[3]);
			pseq->run_times    = ((int)((unsigned char)buf[7])<<24) + ((int)((unsigned char)buf[8])<<16) + ((int)((unsigned char)buf[9])<<8) + (int)((unsigned char)buf[10]);
			pseq->run          = 1;
			message(sock, no, 1, 1, "RUN");
			tim_start=tim_get_now();

			/* Step実行の場合は前回のレジスタ値に戻す */
			if (pseq->ret_line) {
				pseq->reg_flag=local_reg_flag[no-1];
			}
		}
	}
	// 動作停止
	else if (id==0xC016) {
		int no = (int)((unsigned char)buf[6]);
		// 全停止
		if (no==0) {
			for (i=0; i<CONSOLE_MAX; i++) {
				struct _seq_tbl *pseq = &seq_tbl[i];
				struct _action_tbl *pact = &action_tbl[i][pseq->current];
				if (pseq->run) {
					pseq->run = 0;
					sprintf(str, "STOP 行番号 = %d", pact->line);
					message(sock, i+1, 1, 1, str);
				}
			}
		}
		// 指定スレッド停止
		else {
			struct _seq_tbl *pseq = &seq_tbl[no-1];
			struct _action_tbl *pact = &action_tbl[no-1][pseq->current];
			pseq->run = 0;
			sprintf(str, "STOP 行番号 = %d", pact->line);
			message(sock, no, 1, 1, str);
		}
		// パルスモーター強制停止
		for (i=0; ;i++) {
			if (L6470_CH[i]>0) {
				L6470_write(i, 0xB8);	// HardStop
			}
			else{
				break;
			}
		}
		cnt_tbl.busy=0;
		// カウント値取得済であればファイルに保存する
		if (cnt_tbl.n>0) {
			char tmp[32];
			if (count_save(tmp)==0) {
				sprintf(str, "FILE %s", tmp);
				message(sock, no, 1, 1, str);
			}
			cnt_tbl.n=0;
		}
	}
	// スレッド生成
	else if (id==0xC012) {
		int gpio = (int)((unsigned char)buf[7]);
		int count, i;
		/* GPIO IN/OUT設定 */
		for (count=0; GPIO_CH[count]; count++);
		for (i=0; GPIO_CH[i]; i++) {
			if ((gpio>>(count-i-1))&0x01) {
				pinMode(GPIO_CH[i], OUTPUT);
				digitalWrite(GPIO_CH[i], 0);
			}
			else{
				pinMode(GPIO_CH[i], INPUT);
			}
		}
	}
	// スレッド停止
	else if (id==0xC013) {
	}
	return 0;
}

// コンソールとのソケット送受信
static int execute(int sock)
{
	char buf[1024], str[32];
	int i, n, len=1;
	struct timespec tim_last=tim_get_now();

	count_dev_reset();

	while (len!=0) {

		// Local Sokcet
		while (1) {
			// STX受信 0:Close -1:nothing
			char c;
			len = recv(sock, &c, 1, 0);

			if (len<1) {
				break;
			}

			// Recv
			memset(buf, 0, sizeof(buf));
			len = nt_recv(sock, c, buf);
			// ACK返信
			nt_send(sock, ack_msg_data, sizeof(ack_msg_data));
			// コマンド割り振り
			dispatch(sock, buf);
		}

		// シーケンス
		for (i=0; i<CONSOLE_MAX; i++) {
			if (seq_tbl[i].run) {
				sequence(sock, i+1);
			}
			// フォトンカウント取り込み
			count_dev_rcv();
		}

		// 温度を取り込んで表示
		if (tim_timeup(tim_get_now(), tim_last, 1000)) {
        static int flag=0, led_conf=0;
			pthread_mutex_lock(&shm->mutex);
            flag^=1;
            // 温度
            if (flag) {
    			unsigned char data[4]={0x06,0x40,0x80,0xc0};
	    		wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, 4);
				//temp = HEX2TEMP(((data[2]&0x3f)<<6) + (data[3]&0x3f));
				temp = (((data[2]&0x3f)<<6) + (data[3]&0x3f));
	    		// ERR
		    	if (data[0]&0x20) {
			    	unsigned char err_reset[4]={0x3f,0x40,0x80,0xc1};
				    wiringPiSPIDataRW(MAX_SPI_CHANNEL, err_reset, 4);
                }
			}
            // LED設定値
            else{
    			unsigned char data[4]={0x05,0x40,0x80,0xc0};
	    		wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, 4);
				led_conf = ((data[2]&0x3f)<<6) + (data[3]&0x3f);
            }
			pthread_mutex_unlock(&shm->mutex);

			// カウント取り込みが非動作であればここでデータを取り込む
			if (cnt_tbl.busy==0) {
				int sts=0;
				int n=count_dev_n();
				count_dev_read((long*)buf, NULL, NULL, n);
				shm->count=get_1st_data((long*)buf, n, &sts);

			//	sprintf(str, "%03X %.2f℃", led_conf, temp);
				sprintf(str, "%d℃   %ld %s", temp, shm->count, (sts)?"過大光":"");
				message(sock, 0, 1, 1, str);
			}

			tim_last=tim_get_now();
		}
	}
	return 0;
}

static speed_t symbolic_speed( int speednum )
{
	if( speednum >= 460800 ) return B460800;
	if( speednum >= 230400 ) return B230400;
	if( speednum >= 115200 ) return B115200;
	if( speednum >= 57600 ) return B57600;
	if( speednum >= 38400 ) return B38400;
	if( speednum >= 19200 ) return B19200;
	if( speednum >= 9600 ) return B9600;
	if( speednum >= 4800 ) return B4800;
	if( speednum >= 2400 ) return B2400;
	if( speednum >= 1200 ) return B1200;
	if( speednum >= 600 ) return B600;
	if( speednum >= 300 ) return B300;
	if( speednum >= 200 ) return B200;
	if( speednum >= 150 ) return B150;
	if( speednum >= 134 ) return B134;
	if( speednum >= 110 ) return B110;
	if( speednum >= 75 ) return B75;
	return B50;
}

/**************************************************
 * 機能: ttySデバイスを開く
 * 引数: char *devname : デバイスファイル名
 *       int speed     : ボーレート:50～460800
 *       int bits      : ビット長:8/7
 *       int par       : パリティ: 0:無し 1:奇数 2:偶数
 *       int stop      : ストップ: 1/2
 * 戻値: -1:open error
 **************************************************/
static int open_ttyS( char *devname, int speed, int bits, int par, int stop )
{
	int	flag=0;
	struct	termios	pts;	/* ポートのtermios設定 */

	int	pf = open( devname, O_RDWR );
	if( pf < 0 ) return -1;

	/* ポート設定の変更 */
	tcgetattr( pf, &pts );
	pts.c_lflag &= ~(ICANON | ECHO | ECHOCTL | ECHONL | ISIG | IEXTEN);
	pts.c_cflag = HUPCL;
	pts.c_cc[ VMIN ] = 1;
	pts.c_cc[ VTIME ] = 0;

	/* 改行/復帰のマッピングはしない */
	/* 改行のマッピングはしない */
	pts.c_oflag &= ~ONLCR;
	pts.c_iflag &= ~ICRNL;
	/* フロー制御無し */
	pts.c_cflag &= ~CRTSCTS;
	pts.c_iflag &= ~(IXON|IXOFF|IXANY);

	/* ボーレート */
	cfsetospeed( &pts, symbolic_speed( speed ) );
	cfsetispeed( &pts, symbolic_speed( speed ) );

	/* ビット長、パリティ、ストップ */
	if( bits == 7 ) flag |= CS7;
	if( stop == 2 ) flag |= CSTOPB;
	if( par > 0 ){
		flag |= PARENB;
		if( par == 1 ) flag |= PARODD;
	}
	pts.c_cflag |= flag;

	/* 変更したtermios設定を変更する */
	tcsetattr( pf, TCSANOW, &pts );

	return pf;
}

static int mutex_init(void)
{
	pthread_mutexattr_t mat;
	int shmid;
	/* mutex用に共有メモリを利用 */
	const key_t key = 112;
	shmid = shmget(key, sizeof(struct _shm), 0600);
	/* 初回 */
	if (shmid < 0) {
		shmid = shmget(key, sizeof(struct _shm), 0600|IPC_CREAT);
		if (shmid < 0) {
			return -1;
		}
		shm = shmat(shmid, NULL, 0);
		if ((intptr_t)shm == -1) {
			return -1;
		}

		/* mutexのattributeを設定する準備 */
		pthread_mutexattr_init(&mat);

		/* mutexをプロセス間で利用する設定を行う */
		/* これを行わないとプロセス内でのみ有効のmutexになります */
		if (pthread_mutexattr_setpshared(&mat, PTHREAD_PROCESS_SHARED) != 0) {
			return -1;
		}

		pthread_mutex_init(&shm->mutex, &mat);
		shm->count=0L;
		shm->gate_time=10L;
	}
	/* 既に起動済 */
	else{
		shm = shmat(shmid, NULL, 0);
		if ((intptr_t)shm == -1) {
			return -1;
		}
	}
	return 0;
}

int main(void)
{
	int servSock;							//server socket descripter
	int clitSock;							//client socket descripter
	struct sockaddr_in servSockAddr;		//server internet socket address
	struct sockaddr_in clitSockAddr;		//client internet socket address
	unsigned int clitLen;					//client internet socket address length
	unsigned short servPort = 9001;			//server port number
	int on = 1, ret, i;

	mutex_init();

	pthread_mutex_lock(&shm->mutex);
	/* SPI channel 0 を 1MHz で開始 */
	if (wiringPiSPISetupMode(MAX_SPI_CHANNEL, 1000000, 3) < 0) {
		perror("SPI Setup failed:\n");
		exit(EXIT_FAILURE);
	}
	pthread_mutex_unlock(&shm->mutex);
	// 温度設定
//	config_temp(TARGET_TEMP);

	if (wiringPiSetupGpio()==-1) {
		perror("GPIO Setup failed:\n");
		exit(EXIT_FAILURE);
	}
	// SPI CS
	for (i=0; L6470_CH[i]; i++) {
		pinMode(L6470_CH[i], OUTPUT);
		digitalWrite(L6470_CH[i], 1);
	}
	// SPI1 SCLK
	pinMode(GPIO21, OUTPUT);
	digitalWrite(GPIO21, 1);
	// SPI1 MOSI
	pinMode(GPIO20, OUTPUT);
	// SPI1 MISO
	pinMode(GPIO19, INPUT);

	if ((servSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 ){
		perror("socket() failed.");
		exit(EXIT_FAILURE);
	}
	/* Enable address reuse */
	setsockopt (servSock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	memset(&servSockAddr, 0, sizeof(servSockAddr));
	servSockAddr.sin_family      = AF_INET;
	servSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servSockAddr.sin_port        = htons(servPort);

	if (bind(servSock, (struct sockaddr *) &servSockAddr, sizeof(servSockAddr) ) < 0 ) {
		perror("bind() failed.");
		exit(EXIT_FAILURE);
	}

	if (listen(servSock, QUEUELIMIT) < 0) {
		perror("listen() failed.");
		exit(EXIT_FAILURE);
	}

	while(1) {
		clitLen = sizeof(clitSockAddr);
		if ((clitSock = accept(servSock, (struct sockaddr *) &clitSockAddr, &clitLen)) < 0) {
			perror("accept() failed.");
			exit(EXIT_FAILURE);
		}
		printf("connected from %s.\n", inet_ntoa(clitSockAddr.sin_addr));
#if 1
		// ソケットでの通信でブロックしないように設定する。
		ret = fcntl(clitSock, F_GETFL, 0);
	printf("%s %d %d %x\n", __FILE__, __LINE__, ret, O_NONBLOCK);
		if (ret != -1) {
			ret = fcntl(clitSock, F_SETFL, ret | O_NONBLOCK);
		}
#else
		ret = 0;
#endif
		execute(clitSock);

		close(clitSock);
		printf("close\n");
	}
	close(servSock);

	return 0;
}


