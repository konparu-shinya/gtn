/********************************************************************************
 * can版gtnのAction実行部
 *  gcc -o gtnact_can gtnact_can.c
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
#include <linux/can.h>
#include <linux/can/raw.h>

#define QUEUELIMIT 5
#define	CONSOLE_MAX	20

#define ACK		0x06
#define NAK		0x15
#define STX		0x02
#define ETX		0x03
#define ENQ		0x05
#define EOT		0x04
#define TIMEOUT	3000

static char ack_msg_data[] = {STX, 0x04,0x01,0x00,0x11, ETX};
                                                                //1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2
static char msg_msg_data[] = {STX, 0x29,0x00,0x00,0xC9,0x99,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3, ETX};

// 最大30のスレーブ 0:停止中 1:PMモータ動作中 2:DCモータ動作中 11:I/O処理中 12:I/O読込み要求 13:I/O読込み中
struct _slv_tbl {
	int	req;			// 1:PMモータ動作中 2:DCモータ動作中 11:I/O処理中 12:I/O読込み要求 13:I/O読込み中
	int busy;			// 0:停止中 1:動作中 2:動作完了
	int no;				// CONSOLE番号
	struct timespec slv_start;	// 動作開始
} static slv_tbl[30];// 最大30のスレーブ

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
	int stop_req;				// スレーブと連動するための停止要求
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
	char cmd[40];		// 受け取った102コマンドの状態で代入するエリア
} static action_tbl[CONSOLE_MAX][1000];

// イベント
static int event[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static struct timespec tim_start={0,0}, stop_timer={0,0};

#define CAN_NAME "can0"
#define	ID	0x10
#define	DLC	8

// データ識別
#define	CAN_SPLIT_INFO		0x2A		// 分割ﾃﾞｰﾀ 識別情報
#define	CAN_SPLIT_START		0x2B		// 分割ﾃﾞｰﾀ 先頭ﾌﾚｰﾑ
#define	CAN_SPLIT_PROC		0x2C		// 分割ﾃﾞｰﾀ 転送中
#define CAN_SPLIT_END		0x2D		// 分割ﾃﾞｰﾀ 最終ﾌﾚｰﾑ
#define	CAN_TNET_CMD_START	0x1E		// ｼｰｹﾝｽ動作開始

// データの種類
#define	INFO_C101_MSG	0x12	// 設定ﾃﾞｰﾀ
#define	INFO_C102_MSG	0x11	// ｼｰｹﾝｽ動作ﾒｯｾｰｼﾞ
#define	INFO_C103_MSG	0x12	// 設定ﾃﾞｰﾀ

// レコード番号
#define	SEQ_CTL_REC_DCM_1		 0		// DC ﾓｰﾀ #1
#define	SEQ_CTL_REC_DCM_2		 1		// DC ﾓｰﾀ #2
#define	SEQ_CTL_REC_DCM_3		 2		// DC ﾓｰﾀ #3
#define	SEQ_CTL_REC_PSM_1		 3		// ﾊﾟﾙｽﾓｰﾀ #1
#define	SEQ_CTL_REC_PSM_2		 4		// ﾊﾟﾙｽﾓｰﾀ #2
#define	SEQ_CTL_REC_PSM_3		 5		// ﾊﾟﾙｽﾓｰﾀ #3
#define	SEQ_CTL_REC_DI_1		 6		// DIO In #1
#define	SEQ_CTL_REC_DO_1		 7		// DIO Out #1
#define	SEQ_CTL_REC_DI_32		 8		// DIO 4ﾊﾞｲﾄ入力
#define	SEQ_CTL_REC_DO_32		 9		// DIO 4ﾊﾞｲﾄ出力
#define	SEQ_CTL_REC_TM_WAIT		10		// 指定時間待ち
#define	SEQ_CTL_REC_GOTO_ROW	11		// 行の移動
#define	SEQ_CTL_REC_BC_READ		12		// ﾊﾞｰｺｰﾄﾞ･ﾘｰﾀﾞｰ

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

// CAN送信
static int can_send(int can, unsigned char data[])
{
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));
	frame.can_id  = ID;
	frame.can_dlc = DLC;
	memcpy(&(frame.data[0]), &(data[0]), DLC); 

   	write (can, &frame, CAN_MTU);
	return 0;
}

// CANジグフォーマットでの送信(送信後は返信を待つ)
static int can_jigfmt_send(int can, unsigned char sid, unsigned char repnum, unsigned char datident, unsigned char d1, unsigned char d2, unsigned char d3, unsigned char d4)
{
	struct timespec tim_last=tim_get_now();
	struct can_frame frame;
	int len;
#define MAX_SEQNUM 0x10
static unsigned char seqnum = 0;
	unsigned char data[DLC]={sid, repnum, datident, seqnum, d1, d2, d3, d4};
	seqnum = (seqnum+1)%MAX_SEQNUM;		// 次回のシーケンス番号のインクリメント
	can_send(can, data);

	/* 返信をまつ */
	while (1) {
		len = read(can, &frame, sizeof(frame));
		/* IDが照合できればOK */
		if (len==sizeof(frame) && frame.can_id==sid) {
			break;
		}
		if (tim_timeup(tim_get_now(), tim_last, TIMEOUT)) return -1;
	}
	return 0;
}

// スレーブへパルスモーター設定を送信する
static int can_ppm_conf_send(int can, char *buf)
{
	unsigned char sid    = (unsigned char)buf[6]+0x10;
	unsigned char repnum = (unsigned char)buf[6];
	unsigned char datident, mno;
	int i, ret;

	switch (buf[7]) {
	case 1: mno = SEQ_CTL_REC_PSM_1; break;
	case 2: mno = SEQ_CTL_REC_PSM_2; break;
	default: mno = SEQ_CTL_REC_PSM_3; break;
	}
	ret = can_jigfmt_send(can, sid, repnum, CAN_SPLIT_INFO, 0, 0, INFO_C101_MSG, mno);
	if (ret) return -1;

	for (i=0; i<9; i++) {
		if (i==0) {
			datident = CAN_SPLIT_START;
		}
		else if (i==8) {
			datident = CAN_SPLIT_END;
		}
		else{
			datident = CAN_SPLIT_PROC;
		}
		ret = can_jigfmt_send(can, sid, repnum, datident, buf[(i*4)+0], buf[(i*4)+1], buf[(i*4)+2], buf[(i*4)+3]);
		if (ret) return -1;
	}
	return 0;
}

// スレーブへDCモーター設定を送信する
static int can_dc_conf_send(int can, char *buf)
{
	unsigned char sid    = (unsigned char)buf[6]+0x10;
	unsigned char repnum = (unsigned char)buf[6];
	unsigned char datident, mno;
	int i, ret;

	switch (buf[7]) {
	case 1: mno = SEQ_CTL_REC_DCM_1; break;
	case 2: mno = SEQ_CTL_REC_DCM_2; break;
	default: mno = SEQ_CTL_REC_DCM_3; break;
	}
	ret = can_jigfmt_send(can, sid, repnum, CAN_SPLIT_INFO, 0, 0, INFO_C103_MSG, mno);
	if (ret) return -1;

	for (i=0; i<4; i++) {
		if (i==0) {
			datident = CAN_SPLIT_START;
		}
		else if (i==3) {
			datident = CAN_SPLIT_END;
		}
		else{
			datident = CAN_SPLIT_PROC;
		}
		ret = can_jigfmt_send(can, sid, repnum, datident, buf[(i*4)+0], buf[(i*4)+1], buf[(i*4)+2], buf[(i*4)+3]);
		if (ret) return -1;
	}
	return 0;
}

// スレーブへ1つの動作を送信する
static int can_action_send(int can, unsigned char datnum, struct _action_tbl *act)
{
	unsigned char sid    = (unsigned char)act->slvno+0x10;
static int count=0;
	unsigned char repnum = 100 + count;	// 100からの通し番号を入れたいとの事だが...
	unsigned char datident;
	int i, ret;

	count = (++count)%125;	// 0-124

	ret = can_jigfmt_send(can, sid, repnum, CAN_SPLIT_INFO, 0, 0, INFO_C102_MSG, datnum);
	if (ret) return -1;

	for (i=0; i<9; i++) {
		if (i==0) {
			datident = CAN_SPLIT_START;
		}
		else if (i==8) {
			datident = CAN_SPLIT_END;
		}
		else{
			datident = CAN_SPLIT_PROC;
		}
		ret = can_jigfmt_send(can, sid, repnum, datident, act->cmd[(i*4)+0], act->cmd[(i*4)+1], act->cmd[(i*4)+2], act->cmd[(i*4)+3]);
		if (ret) return -1;
	}

	/* 開始コマンドを送る */
	ret = can_jigfmt_send(can, sid, repnum, CAN_TNET_CMD_START, 0, 0, 0, 0);
	if (ret) return -1;

	return 0;
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

// シーケンス
static int sequence(int sock, int no, int can)
{
	int i, error=0;
	char str[64];
	struct _seq_tbl *pseq = &seq_tbl[no-1];
	struct _action_tbl *pact = &action_tbl[no-1][pseq->current];
	struct _slv_tbl *pslv    = &slv_tbl[pact->slvno];
	int local_ret_line       = pact->line+1;

	switch (pact->act) {
	case 0x01:		// DCモーター初期化
	case 0x02:		// DCモーターCW(回転時間指定)
	case 0x03:		// DCモーターCW(STEPセンサーまで回転)
	case 0x04:		// DCモーターCCW(回転時間指定)
	case 0x05:		// DCモーターCCW(HOMEセンサーまで回転)
	case 0x06:		// DCモーター停止までまつ
		{
			unsigned char mno;
			switch (pact->mno) {
			case 1: mno = SEQ_CTL_REC_DCM_1; break;
			case 2: mno = SEQ_CTL_REC_DCM_2; break;
			default: mno = SEQ_CTL_REC_DCM_3; break;
			}
			/* スレーブ動作完了 */
			if (pslv->busy==2) {
				/* DCモーター停止までまつ */
				if (pact->act==0x06 || pact->act==0x01) {
					pslv->busy=0;
					pseq->current++;
				}
				break;
			}
			/* スレーブが動作中の場合は動作を待つ */	
			else if (pslv->busy) {
				break;
			}
			/* DCモーター停止までまつ */
			if (pact->act==0x06) {
				// 何もしない
				break;
			}
			/* Action */
			else if (can_action_send(can, mno, pact)) {
				sprintf(str, "ERR 行番号 = %d CAN Error", pact->line);
				message(sock, no, 1, 1, str);
				pseq->run = 0;
			}
			else{
				pslv->req=2;
				pslv->busy=1;
				pslv->no=no;
			}
			/* DCモーター初期化以外 */
			if (pact->act!=0x01) {
				pseq->current++;
			}
		}
		break;
	case 0x11:		// パルスモーター初期化
	case 0x12:		// パルスモーターCW(相対パルス指定)
	case 0x13:		// パルスモーターCCW(相対パルス指定)
	case 0x14:		// パルスモーターHOME(現在位置パルス分をCCW回転する)
	case 0x15:		// パルスモーター絶対パルス移動
	case 0x16:		// パルスモーター停止までまつ
	case 0x17:		// パルスモーター励磁解除
	case 0x18:		// パルスモーター励磁ON
//	case 0x31:		// パルスモーターI/O指定ビットON
//	case 0x32:		// パルスモーターI/O指定ビットOFF
//	case 0x33:		// パルスモーターI/O指定ビットをONまでまつ
//	case 0x34:		// パルスモーターI/O指定ビットをOFFまでまつ
//	case 0x35:		// パルスモーターI/O 16bitデータ書き込み
//	case 0x36:		// パルスモーターI/O 16bitデータ読み込んで表示
		{
			unsigned char mno;
			switch (pact->mno) {
			case 1: mno = SEQ_CTL_REC_PSM_1; break;
			case 2: mno = SEQ_CTL_REC_PSM_2; break;
			default: mno = SEQ_CTL_REC_PSM_3; break;
			}
			/* スレーブ動作完了 */
			if (pslv->busy==2) {
				/* パルスモーター停止までまつ */
				if (pact->act==0x16 || pact->act==0x11) {
					pslv->busy=0;
					pseq->current++;
					break;
				}
			}
			/* スレーブが動作中の場合は動作を待つ */	
			else if (pslv->busy) {
				break;
			}
			/* パルスモーター停止までまつ */
			if (pact->act==0x16) {
				// 何もしない
				break;
			}
			/* Action */
			else if (can_action_send(can, mno, pact)) {
				sprintf(str, "ERR 行番号 = %d CAN Error", pact->line);
				message(sock, no, 1, 1, str);
				pseq->run = 0;
			}
			else{
				pslv->req=1;
				pslv->busy=1;
				pslv->no=no;
			}
			/* パルスモーター初期化以外 */
			if (pact->act!=0x11) {
				pseq->current++;
			}
		}
		break;
	case 0x41:		// DIO指定ビットON
	case 0x42:		// DIO指定ビットOFF
	case 0x43:		// DIO指定ビットをONまでまつ
	case 0x44:		// DIO指定ビットをOFFまでまつ
	case 0x45:		// DIO 32bitデータ書き込み
	case 0x46:		// DIO 32bitデータ読み込んで表示
		{
			unsigned char dno;
			switch (pact->act) {
			case 0x41: dno = SEQ_CTL_REC_DO_1; break;
			case 0x42: dno = SEQ_CTL_REC_DO_1; break;
			case 0x43: dno = SEQ_CTL_REC_DI_1; break;
			case 0x44: dno = SEQ_CTL_REC_DI_1; break;
			case 0x45: dno = SEQ_CTL_REC_DO_32; break;
			default: dno = SEQ_CTL_REC_DI_32; break;
			}
			/* スレーブ動作完了 */
			if (pslv->busy==2) {
				pslv->busy=0;
				pseq->current++;
				break;
			}
			/* スレーブが動作中の場合は動作を待つ */	
			else if (pslv->busy) {
				// 5秒無応答でSKIP
				if (pslv->req==11 && tim_timeup(tim_get_now(), pslv->slv_start, 5000)) {
					pslv->busy=2;
				}
				break;
			}
			/* Action */
			if (can_action_send(can, dno, pact)) {
				sprintf(str, "ERR 行番号 = %d CAN Error", pact->line);
				message(sock, no, 1, 1, str);
				pseq->run = 0;
			}
			/* DIO 32bitデータ読み込んで表示 */
			else if (pact->act==0x46) {
				pslv->req=12;
				pslv->busy=1;
				pslv->no=no;
			}
			/* DIO 32bitデータ読み込んで表示 以外 */
			else{
				pslv->req=11;
				pslv->busy=1;
				pslv->no=no;
				pslv->slv_start=tim_get_now();
			}
		}
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
	default:
		pseq->current++;
		break;
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
//printf("%s %d %d %d\n", __FILE__, __LINE__, pseq->ret_line, local_ret_line);
		}
		// Next
		else{
			pseq->current = 0;
		}
	}
	return 0;
}

// コマンド受信処理
static int dispatch(int sock, char *buf, int can)
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
	// 停止監視タイマーSTOP
	if (!run_flag) {
		stop_timer.tv_sec=0;
	}

	// パルスモーター設定
	if (id==0xC101 && can>=0 && run_flag==0) {
		if (can_ppm_conf_send(can, buf)) {
			local_param_err = -1;
		}
		else{
			local_param_err = 0;
		}
	}
	// DCモーター設定
	else if (id==0xC103 && can>=0 && run_flag==0) {
		if (can_dc_conf_send(can, buf)) {
			local_param_err = -1;
		}
		else{
			local_param_err = 0;
		}
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
		memset(pact->cmd, 0xff, sizeof(pact->cmd));
		memcpy(pact->cmd, buf, 0x22);
//printf("%s %d %d %d\n", __FILE__, __LINE__, pact->line, pseq->max_line);
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
			memset(slv_tbl, 0, sizeof(slv_tbl));
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
		else if (can<0) {
			message(sock, no, 1, 1, "ERR CAN Socket Error");
		}
		else if (local_param_err) {
			message(sock, no, 1, 1, "ERR DC/PPM Param Send Error");
		}
		else{
			pseq->ret_line     = (int)((unsigned char)buf[3]);
			pseq->run_times    = ((int)((unsigned char)buf[7])<<24) + ((int)((unsigned char)buf[8])<<16) + ((int)((unsigned char)buf[9])<<8) + (int)((unsigned char)buf[10]);
			pseq->run          = 1;
			pseq->stop_req     = 0;
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
				pseq->stop_req=1;
			}
		}
		// 指定スレッド停止
		else {
			struct _seq_tbl *pseq = &seq_tbl[no-1];
			pseq->stop_req=1;
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
static int execute(int sock, int can)
{
	char buf[1024], str[32];
	int i, n, len=1;
	struct timespec tim_last=tim_get_now();

	while (len!=0) {
		// CAN Sokcet
		if (can>=0) {
			struct can_frame frame;
			int len = read(can, &frame, sizeof(frame));
			if (len==sizeof(frame)) {
				// Actionの戻りを受信する
				if (frame.can_id>ID) {
					int rid=frame.can_id-ID;
					struct _slv_tbl *pslv = &slv_tbl[rid];
					struct _seq_tbl *pseq = &seq_tbl[pslv->no-1];
					struct _action_tbl *pact = &action_tbl[pslv->no-1][pseq->current];
//printf("%s %d %d %d\n", __FILE__, __LINE__, rid, pslv->busy);
					switch (pslv->busy) {
					case 1:		// PPM Busy
						pslv->busy=2;
						break;
					case 2:		// DC Busy
						pslv->busy=2;
						break;
					case 11:	// I/O Busy
						pslv->busy=2;
						break;
					case 12:	// I/O Read Req
						pslv->req=13;
						break;
					case 13:	// I/O Data Transfer 
						pseq->reg_flag=(((int)frame.data[4])<<24) + (((int)frame.data[5])<<16) + (((int)frame.data[6])<<8) + ((int)frame.data[7]);
						sprintf(str, "Port %d-%d = %02X%02X %02X%02Xh", pact->slvno, pact->mno,
																		(unsigned char)(frame.data[4]), (unsigned char)(frame.data[5]),
																		(unsigned char)(frame.data[6]), (unsigned char)(frame.data[7]));
						message(sock, pslv->no, 1, 3, str);

						pslv->busy=2;
						break;
					default:
						break;
					}
				}
			}
		}

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
			dispatch(sock, buf, can);
		}

		// シーケンス
		for (i=0; i<CONSOLE_MAX; i++) {
			struct _seq_tbl *pseq = &seq_tbl[i];
			if (pseq->run) {
				// STOP
				if (pseq->stop_req) {
					int j;
					int busy_flag=0;
					// 停止監視タイマースタート
					if (stop_timer.tv_sec==0) {
						stop_timer=tim_get_now();
					}
					// 指定スレッドのスレーブ動作状況を確認する
					for (j=0; j<30; j++) {
						if (slv_tbl[j].no==(i+1) && slv_tbl[j].busy==1) {
							busy_flag=1;
							break;
						}
					}
					// スレーブ停止もしくは10秒経過した場合に停止処理
					if (!busy_flag || tim_timeup(tim_get_now(), stop_timer, 10000)) {
						struct _action_tbl *pact = &action_tbl[i][pseq->current];
						pseq->run = 0;
						sprintf(str, "STOP 行番号 = %d", pact->line);
						message(sock, i+1, 1, 1, str);
					}
				}
				// RUN
				else{
					sequence(sock, i+1, can);
				}
			}
		}
	}
	return 0;
}

// CAN初期化
static int can_init(void)
{
	struct ifreq ifr;
	struct sockaddr_can addr;
	int s;

	system("/bin/ip link set can0 type can bitrate 500000");
	system("/bin/ip link set can0 up");

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		return -2;
	}

	memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
	strncpy(ifr.ifr_name, CAN_NAME, sizeof(ifr.ifr_name));

	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		return -3;
	}

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

//	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		return -4;
	}

	return s;
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
	int canSock = can_init();

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
		ret = fcntl(canSock, F_GETFL, 0);
	printf("%s %d %d %x\n", __FILE__, __LINE__, ret, O_NONBLOCK);
		if (ret != -1) {
			ret = fcntl(canSock, F_SETFL, ret | O_NONBLOCK);
		}
#else
		ret = 0;
#endif
		execute(clitSock, canSock);

		close(clitSock);
		printf("close\n");
	}
	close(servSock);

	if (canSock>=0) {
		close(canSock);
	}
	return 0;
}


