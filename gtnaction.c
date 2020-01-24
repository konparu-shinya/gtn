/********************************************************************************
 * gtn for ラズパイ のAction実行部
 *  パルスモーターコントローラー:L6470
 *  gcc -o test test.c -lwiringPi
 ********************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <net/if.h>
#include <sys/wait.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define L6470_SPI_CHANNEL 0
#define	GPIO17	17
#define	GPIO18	18
const unsigned char L6470_CH[] = {GPIO17, GPIO18, 0};

#define QUEUELIMIT 5
#define	CONSOLE_MAX	20
#define	PPM_MAX	10

#define ACK		0x06
#define NAK		0x15
#define STX		0x02
#define ETX		0x03
#define ENQ		0x05
#define EOT		0x04
#define TIMEOUT	30000

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
	struct timespec wai_start;	// Wait開始
} static seq_tbl[CONSOLE_MAX]={
		{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0}
	};

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
	int driving;			// 0:not use 1:init home out 2:init home in 3:init home add 4:home/step 5:free
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
	int len, length, tm, i, count=0;
	// STX
	if (c!=STX) return -1;
	p[count++] = c;

	// length
	for (tm=0; tm<TIMEOUT; tm++) {
		len = recv(sock, &c, 1, 0);
		if (len==0) return 0;
		if (len>0) break;
		usleep(100);
	}
	if (tm>=TIMEOUT) return -1;
	length = (int)c;
	p[count++] = c;

	for (i=1; i<length; i++) {
		for (tm=0; tm<TIMEOUT; tm++) {
			len = recv(sock, &c, 1, 0);
			if (len==0) return 0;
			if (len>0) break;
			usleep(100);
		}
		if (tm>=TIMEOUT) return -1;
		p[count++] = c;
	}

	// ETX
	for (tm=0; tm<TIMEOUT; tm++) {
		len = recv(sock, &c, 1, 0);
		if (len==0) return 0;
		if (len>0) break;
		usleep(100);
	}
	if (tm>=TIMEOUT) return -1;
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

// L6470に1byte送信
static unsigned char L6470_write(unsigned char ch, unsigned char data)
{
	unsigned char org=data;
	digitalWrite(L6470_CH[ch], 0);
	wiringPiSPIDataRW(L6470_SPI_CHANNEL, &data, 1);
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

static void L6470_softstop(unsigned char ch)
{
	L6470_write(ch, 0xB0);
}

static void L6470_softhiz(unsigned char ch)
{
	L6470_write(ch, 0xA8);
}

// L6470 駆動確認
static int L6470_busy(unsigned char ch)
{
	return ((L6470_param_read(ch, 0x19)&0x02) == 0x00)?1:0;
}

// L6470 初期設定
static void L6470_init(unsigned char ch)
{
	// NOP x4 -> Reset
	L6470_write(ch, 0);
	L6470_write(ch, 0);
	L6470_write(ch, 0);
	L6470_write(ch, 0);
	L6470_write(ch, 0xc0);
	L6470_softstop(ch);
	L6470_softhiz(ch);

	// MIN_SPEED設定。
	L6470_param_write(ch, 0x08, 0x15);
	// MAX_SPEED設定。
	L6470_param_write(ch, 0x07, 2000);
	// KVAL_HOLD設定。
	L6470_param_write(ch, 0x09, 0x0f);
	// KVAL_RUN設定。
	L6470_param_write(ch, 0x0A, 0xff);
	// KVAL_ACC設定。
	L6470_param_write(ch, 0x0B, 0xff);
	// KVAL_DEC設定。
	L6470_param_write(ch, 0x0C, 0xff);
	// OCD_TH設定。
	L6470_param_write(ch, 0x13, 0x0f);
	// STALL_TH設定。
	L6470_param_write(ch, 0x14, 0x7f);
	// STEP MODE
	L6470_param_write(ch, 0x16, ppm_ctrl[ch].r9);
}

// L6470 スピード変更
static void L6470_change_spd(unsigned char ch, int start_pulse, int max_pulse, int st_slope, int ed_slope)
{
	struct _ppm_ctrl *pctrl = &ppm_ctrl[ch];
	if (pctrl->speed.start!=start_pulse) {
		pctrl->speed.start=start_pulse;
		// MIN_SPEED設定。
		L6470_param_write(ch, 0x08, start_pulse);
	}
	if (pctrl->speed.max!=max_pulse) {
		pctrl->speed.max=max_pulse;
		// MAX_SPEED設定。
		L6470_param_write(ch, 0x07, max_pulse);
	}
	if (pctrl->speed.acc!=st_slope) {
		pctrl->speed.acc=st_slope;
		// ACC設定。
		L6470_param_write(ch, 0x05, st_slope);
	}
	if (pctrl->speed.dec!=ed_slope) {
		pctrl->speed.dec=ed_slope;
		// DEC設定。
		L6470_param_write(ch, 0x06, ed_slope);
	}
}

// パルスモーターの初期化動作
// 戻値: 0:初期化完了 1:初期化中
static int ppm_init(int no)
{
	if (L6470_busy(no)) return 1;

	switch (ppm_ctrl[no].driving) {
	case 1:		// 1:init home out
		// home out完了?
		ppm_ctrl[no].driving=2;
		break;
	case 2:		// 2:init home in
		// home in完了?
		ppm_ctrl[no].driving=3;
		break;
	case 3:		// 3:init home add
		// home add完了?
		ppm_ctrl[no].driving=4;
		break;
	default:
		// 初期化開始
		// センサーIN?
		ppm_ctrl[no].driving=1;
		ppm_ctrl[no].driving=2;
		break;
	}
	return (ppm_ctrl[no].driving==4) ? 0:1;
}

// シーケンス
static int sequence(int sock, int no)
{
    int i;
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
		if (ppm_init(pact->mno-1)==0) {
			pseq->current++;
		}
		break;
	case 0x12:		// パルスモーターSTEP(相対パルス指定)
		if (!L6470_busy(pact->mno-1)) {
			unsigned char cmd = (ppm_ctrl[pact->mno-1].dir.step)?0x40:0x41;
			L6470_change_spd(pact->mno-1, pact->start_pulse, pact->max_pulse, pact->st_slope, pact->ed_slope);
			L6470_cmd_write(pact->mno-1, cmd, 3, pact->move_pulse);//Move
//			delay(10);
			pseq->current++;
		}
		break;
	case 0x13:		// パルスモーターHOME(相対パルス指定)
		if (!L6470_busy(pact->mno-1)) {
			unsigned char cmd = (ppm_ctrl[pact->mno-1].dir.home)?0x40:0x41;
			L6470_change_spd(pact->mno-1, pact->start_pulse, pact->max_pulse, pact->st_slope, pact->ed_slope);
			L6470_cmd_write(pact->mno-1, cmd, 3, pact->move_pulse);//Move
//			delay(10);
			pseq->current++;
		}
		break;
	case 0x14:		// パルスモーターHOME(現在位置パルス分をCCW回転する)
		if (!L6470_busy(pact->mno-1)) {
			L6470_change_spd(pact->mno-1, pact->start_pulse, pact->max_pulse, pact->st_slope, pact->ed_slope);
			L6470_cmd_write(pact->mno-1, 0x70, 1, 0);//GoHome
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
		if (!L6470_busy(pact->mno-1)) {
			pseq->current++;
		}
		break;
	case 0x17:		// パルスモーター励磁解除
		// KVAL_HOLD設定。
		L6470_param_write(pact->mno-1, 0x09, 0x0);
		pseq->current++;
		break;
	case 0x18:		// パルスモーター励磁ON
		// KVAL_HOLD設定。
		L6470_param_write(pact->mno-1, 0x09, 0x0f);
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
	case 0x42:		// DIO指定ビットOFF
	case 0x43:		// DIO指定ビットをONまでまつ
	case 0x44:		// DIO指定ビットをOFFまでまつ
	case 0x45:		// DIO 32bitデータ書き込み
	case 0x46:		// DIO 32bitデータ読み込んで表示
		{
#if 0
			/* Action */
			if (can_action_send(can, pseq->current, dno, pact)) {
				sprintf(str, "ERR 行番号 = %d CAN Error", pact->line);
				message(sock, no, 1, 1, str);
				pseq->run = 0;
			}
			/* DIO 32bitデータ読み込んで表示 */
			else if (pact->act==0x46) {
				pseq->slv_busy[pact->slvno]=12;
			}
			/* DIO 32bitデータ読み込んで表示 以外 */
			else{
				pseq->slv_busy[pact->slvno]=11;
			}
#endif
			/* 	I/Oは動作完了するまで進めないのでコメントにする */
			//pseq->current++;
		}
		break;
	case 0x51:		// 指定時間まち(×10msec)
		if (pseq->busy==0) {
			pseq->busy=1;
			clock_gettime(CLOCK_MONOTONIC, &pseq->wai_start);
		}
		else{
			time_t msec;
			long sec;
			struct timespec tim_end;
			clock_gettime(CLOCK_MONOTONIC, &tim_end);
			if((tim_end.tv_nsec - pseq->wai_start.tv_nsec) < 0){
				tim_end.tv_nsec += 1000000000;
				tim_end.tv_sec  -= 1;
			}
			msec = (tim_end.tv_nsec - pseq->wai_start.tv_nsec)/1000000;
			sec  = tim_end.tv_sec - pseq->wai_start.tv_sec;

			if (((sec*1000)+msec) >= pact->move_pulse) {
				sprintf(str, "Loop:%d Time:%lu.%lu秒", pseq->count+1, sec, msec/100);
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
		pseq->run = 0;
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
	case 0x71:		// A/D取り込み
		pseq->current++;
		break;
	default:
		pseq->current++;
		break;
	}

	// 終了判定
	if (pseq->current >= pseq->max_line) {
		/* 経過時間を表示する */
		time_t msec;
		long sec;
		struct timespec tim_end;
		clock_gettime(CLOCK_MONOTONIC, &tim_end);
		if((tim_end.tv_nsec - tim_start.tv_nsec) < 0){
			tim_end.tv_nsec += 1000000000;
			tim_end.tv_sec  -= 1;
		}
		msec = (tim_end.tv_nsec - tim_start.tv_nsec)/1000000;
		sec  = tim_end.tv_sec - tim_start.tv_sec;

		sprintf(str, "Loop:%d Time:%lu.%lu秒", pseq->count+1, sec, msec/100);
		message(sock, no, 1, 2, str);


		pseq->count++;
		// 終了
		if (pseq->count >= pseq->run_times) {
			pseq->run = 0;
			message(sock, no, 1, 1, "success!!");
			/* 次の行番号を送る */
			if (pseq->ret_line) {
				sprintf(str, "%d", local_ret_line);
				message(sock, no, 2, 0, str);
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
		L6470_init(no-1);
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
			clock_gettime(CLOCK_MONOTONIC, &tim_start);

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
				L6470_softstop(i);
				L6470_softhiz(i);
			}
			else{
				break;
			}
		}
	}
	// スレッド生成
	else if (id==0xC012) {
	}
	// スレッド停止
	else if (id==0xC013) {
	}
	return 0;
}

// コンソールとのソケット送受信
static int execute(int sock)
{
	char buf[512], str[32];
	int i, len=1;

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

	/* SPI channel 0 を 1MHz で開始 */
	if (wiringPiSPISetupMode(L6470_SPI_CHANNEL, 1000000, 3) < 0) {
		perror("SPI Setup failed:\n");
		exit(EXIT_FAILURE);
	}

	if (wiringPiSetupGpio()==-1) {
		perror("GPIO Setup failed:\n");
		exit(EXIT_FAILURE);
	}
	for (i=0; L6470_CH[i]; i++) {
		pinMode(L6470_CH[i], OUTPUT);
		digitalWrite(L6470_CH[i], 1);
	}

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


