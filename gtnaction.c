/********************************************************************************
 * gtn for ラズパイ のAction実行部
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
#include <pthread.h>

#define QUEUELIMIT 5

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
	int my_thread_no;			// コンソールから受け取ったスレッド番号
	int run_times;				// コンソールから受け取った繰り返し回数
	int ret_line;				// 1:Step実行(動作終了時に次のlineを送信する)
	int reg_flag;				// レジスタ読み込み値の保存
	int	slv_busy[30];			// 最大30のスレーブ 0:停止中 1:PMモータ動作中 2:DCモータ動作中 11:I/O処理中 12:I/O読込み要求 13:I/O読込み中
} static seq_tbl={0,0,0,0,0,0,0};


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
} static action[1000];

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

// シーケンス
static int sequence(int sock)
{
	int local_ret_line=-1;
    int i;
	char str[32];

	switch (action[seq_tbl.current].act) {
	case 0x01:		// DCモーター初期化
	case 0x02:		// DCモーターCW(回転時間指定)
	case 0x03:		// DCモーターCW(STEPセンサーまで回転)
	case 0x04:		// DCモーターCCW(回転時間指定)
	case 0x05:		// DCモーターCCW(HOMEセンサーまで回転)
		{
			/* スレーブが動作中の場合は動作を待つ */	
			if (seq_tbl.slv_busy[action[seq_tbl.current].slvno]) {
				usleep(50000);
				break;
			}
#if 0
			/* Action */
			if (can_action_send(can, seq_tbl.current, mno, &action[seq_tbl.current])) {
				sprintf(str, "ERR 行番号 = %d CAN Error", action[seq_tbl.current].line);
				message(sock, seq_tbl.my_thread_no, 1, 1, str);
				seq_tbl.run = 0;
			}
			else{
				seq_tbl.slv_busy[action[seq_tbl.current].slvno]=2;
			}
#endif
			seq_tbl.current++;
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
			/* スレーブが動作中の場合は動作を待つ */	
			if (seq_tbl.slv_busy[action[seq_tbl.current].slvno]) {
				usleep(50000);
				break;
			}
			/* パルスモーター停止までまつ */
			if (action[seq_tbl.current].act==0x16) {
				// 何もしない
			}
#if 0
			/* Action */
			else if (can_action_send(can, seq_tbl.current, mno, &action[seq_tbl.current])) {
				sprintf(str, "ERR 行番号 = %d CAN Error", action[seq_tbl.current].line);
				message(sock, seq_tbl.my_thread_no, 1, 1, str);
				seq_tbl.run = 0;
			}
#endif
			else{
				seq_tbl.slv_busy[action[seq_tbl.current].slvno]=1;
			}
			seq_tbl.current++;
		}
		break;
	case 0x41:		// DIO指定ビットON
	case 0x42:		// DIO指定ビットOFF
	case 0x43:		// DIO指定ビットをONまでまつ
	case 0x44:		// DIO指定ビットをOFFまでまつ
	case 0x45:		// DIO 32bitデータ書き込み
	case 0x46:		// DIO 32bitデータ読み込んで表示
		{
			/* スレーブが動作中の場合は動作を待つ */	
			if (seq_tbl.slv_busy[action[seq_tbl.current].slvno]) {
				usleep(500);
				break;
			}
#if 0
			/* Action */
			if (can_action_send(can, seq_tbl.current, dno, &action[seq_tbl.current])) {
				sprintf(str, "ERR 行番号 = %d CAN Error", action[seq_tbl.current].line);
				message(sock, seq_tbl.my_thread_no, 1, 1, str);
				seq_tbl.run = 0;
			}
			/* DIO 32bitデータ読み込んで表示 */
			else if (action[seq_tbl.current].act==0x46) {
				seq_tbl.slv_busy[action[seq_tbl.current].slvno]=12;
			}
			/* DIO 32bitデータ読み込んで表示 以外 */
			else{
				seq_tbl.slv_busy[action[seq_tbl.current].slvno]=11;
			}
#endif
			/* 	I/Oは動作完了するまで進めないのでコメントにする */
			//seq_tbl.current++;
		}
		break;
	case 0x51:		// 指定時間まち(×10msec)
		usleep(action[seq_tbl.current].move_pulse*1000);
		seq_tbl.current++;
		break;
	case 0x52:		// if文
		/* 条件が揃えば指定行へ */
		if (((unsigned short)seq_tbl.reg_flag & (unsigned short)action[seq_tbl.current].start_pulse)==(unsigned short)action[seq_tbl.current].max_pulse) {
       		for (i=0; i<seq_tbl.max_line; i++) {
				if (action[seq_tbl.current].move_pulse==action[i].line) {
					seq_tbl.current = i;
					local_ret_line=action[i].line;
    	           	break;
        	   	}
       		}
		}
		/* 次の行へ */
		else{
			seq_tbl.current++;
		}
		break;
	case 0x53:		// unless文
		if (((unsigned short)seq_tbl.reg_flag & (unsigned short)action[seq_tbl.current].start_pulse)!=(unsigned short)action[seq_tbl.current].max_pulse) {
       		for (i=0; i<seq_tbl.max_line; i++) {
				if (action[seq_tbl.current].move_pulse==action[i].line) {
					seq_tbl.current = i;
					local_ret_line=action[i].line;
    	           	break;
        	   	}
       		}
		}
		/* 次の行へ */
		else{
			seq_tbl.current++;
		}
		break;
	case 0x54:		// 最終行へ移動
		if (seq_tbl.max_line>0) {
			seq_tbl.current = seq_tbl.max_line-1;
		}
		break;
	case 0x55:		// 指定行へ移動
       	for (i=0; i<seq_tbl.max_line; i++) {
			if (action[seq_tbl.current].move_pulse==action[i].line) {
				seq_tbl.current = i;
               	break;
           	}
       	}
		break;
	case 0x56:		// SIO送信
		break;
	case 0x57:		// エラー停止
		sprintf(str, "ERR 行番号 = %d", action[seq_tbl.current].line);
		message(sock, seq_tbl.my_thread_no, 1, 1, str);
		seq_tbl.run = 0;
		break;
	case 0x61:		// イベントセット
		break;
	case 0x62:		// イベントクリア
		break;
	case 0x63:		// イベントセットまち
		break;
	case 0x64:		// いべんとクリアまち
		break;
	case 0x71:		// A/D取り込み
		break;
	default:
		seq_tbl.current++;
		break;
	}

	// 終了判定
	if (seq_tbl.current >= seq_tbl.max_line) {
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

		sprintf(str, "Loop:%d Time:%lu.%lu秒", seq_tbl.count+1, sec, msec/100);
		message(sock, seq_tbl.my_thread_no, 1, 2, str);


		seq_tbl.count++;
		// 終了
		if (seq_tbl.count >= seq_tbl.run_times) {
			seq_tbl.run = 0;
			message(sock, seq_tbl.my_thread_no, 1, 1, "success!!");
			/* 次の行番号を送る */
			if (seq_tbl.ret_line) {
				sprintf(str, "%d", local_ret_line);
				message(sock, seq_tbl.my_thread_no, 2, 0, str);
			}
		}
		// Next
		else{
			seq_tbl.current = 0;
		}
	}
	return 0;
}


// スレッド動作
void *thread_main(void *ptr)
{
	int no=*(int*)ptr;
	int a=0;
	printf("%s %d %d %d\n", __FILE__, __LINE__, no, a++);
	while(1);
}

// コマンド受信処理
static int dispatch(int sock, char *buf)
{
#if 0
static int local_param_err=0;		// 1:DC、PPMパラメータ送信エラー発生
static int local_reg_flag=0;
	char str[32];
	unsigned short id = ((unsigned short)((unsigned char)buf[4])<<8) + (unsigned short)((unsigned char)buf[5]);

	// パルスモーター設定
	if (id==0xC101 && can>=0 && seq_tbl.run==0) {
		if (can_ppm_conf_send(can, buf)) {
			local_param_err = -1;
		}
		else{
			local_param_err = 0;
		}
	}
	// DCモーター設定
	else if (id==0xC103 && can>=0 && seq_tbl.run==0) {
		if (can_dc_conf_send(can, buf)) {
			local_param_err = -1;
		}
		else{
			local_param_err = 0;
		}
	}
	// 動作シーケンス
	else if (id==0xC102 && seq_tbl.run==0) {
		action[seq_tbl.max_line].line        = ((int)((unsigned char)buf[6])<<8) + (int)((unsigned char)buf[7]);
		action[seq_tbl.max_line].slvno       = (int)((unsigned char)buf[8]);
		action[seq_tbl.max_line].mno         = (int)((unsigned char)buf[9]);
		action[seq_tbl.max_line].act         = ((int)((unsigned char)buf[10])<<8) + (int)((unsigned char)buf[11]);
		action[seq_tbl.max_line].move_pulse  = ((unsigned long)((unsigned char)buf[12])<<24) + ((unsigned long)((unsigned char)buf[13])<<16) + ((unsigned long)((unsigned char)buf[14])<<8) + (unsigned long)((unsigned char)buf[15]);
		action[seq_tbl.max_line].start_pulse = ((int)((unsigned char)buf[16])<<8) + (int)((unsigned char)buf[17]);
		action[seq_tbl.max_line].max_pulse   = ((int)((unsigned char)buf[18])<<8) + (int)((unsigned char)buf[19]);
		action[seq_tbl.max_line].st_slope    = ((int)((unsigned char)buf[20])<<8) + (int)((unsigned char)buf[21]);
		action[seq_tbl.max_line].ed_slope    = ((int)((unsigned char)buf[22])<<8) + (int)((unsigned char)buf[23]);
		action[seq_tbl.max_line].ratio       = ((int)((unsigned char)buf[24])<<8) + (int)((unsigned char)buf[25]);
		memset(action[seq_tbl.max_line].cmd, 0xff, sizeof(action[seq_tbl.max_line].cmd));
		memcpy(action[seq_tbl.max_line].cmd, buf, 0x22);
		seq_tbl.max_line++;
	}
	// 動作準備
	else if (id==0xC014 && seq_tbl.run==0) {
		local_reg_flag=seq_tbl.reg_flag;			// レジスタ値を保存
		memset(&seq_tbl, 0, sizeof(seq_tbl));
		seq_tbl.my_thread_no = (int)((unsigned char)buf[6]);
		message(sock, seq_tbl.my_thread_no, 1, 1, "START");
		message(sock, seq_tbl.my_thread_no, 1, 2, "");
		message(sock, seq_tbl.my_thread_no, 1, 3, "");
	}
	// 動作開始
	else if (id==0xC015 && seq_tbl.run==0) {
		if (can<0) {
			message(sock, seq_tbl.my_thread_no, 1, 1, "ERR CAN Socket Error");
		}
		else if (local_param_err) {
			message(sock, seq_tbl.my_thread_no, 1, 1, "ERR DC/PPM Param Send Error");
		}
		else{
			seq_tbl.ret_line     = (int)((unsigned char)buf[3]);
			seq_tbl.my_thread_no = (int)((unsigned char)buf[6]);
			seq_tbl.run_times    = ((int)((unsigned char)buf[7])<<24) + ((int)((unsigned char)buf[8])<<16) + ((int)((unsigned char)buf[9])<<8) + (int)((unsigned char)buf[10]);
			seq_tbl.run          = 1;
			message(sock, seq_tbl.my_thread_no, 1, 1, "RUN");
			clock_gettime(CLOCK_MONOTONIC, &tim_start);

			/* Step実行の場合は前回のレジスタ値に戻す */
			if (seq_tbl.ret_line) {
				seq_tbl.reg_flag=local_reg_flag;
			}
		}
	}
	// 動作停止
	else if (id==0xC016 && seq_tbl.run) {
		seq_tbl.run = 0;
		sprintf(str, "STOP 行番号 = %d", action[seq_tbl.current].line);
		message(sock, seq_tbl.my_thread_no, 1, 1, str);
	}
	// スレッド生成
	else if (id==0xC012) {
	}
	// スレッド停止
	else if (id==0xC013) {
	}
#endif
	return 0;
}

// コンソールとのソケット送受信
static int execute(int sock)
{
	char buf[512], str[32];

	while (1) {
#if 0
		// CAN Sokcet
		if (can>=0) {
			struct can_frame frame;
			int len = read(can, &frame, sizeof(frame));
			if (len==sizeof(frame)) {
				// Actionの戻りを受信する
				if (seq_tbl.run && frame.can_id>0x10) {
					int rid=frame.can_id-0x10;
					switch (seq_tbl.slv_busy[rid]) {
					case 1:		// PPM Busy
						seq_tbl.slv_busy[rid]=0;
						break;
					case 2:		// DC Busy
						seq_tbl.slv_busy[rid]=0;
						break;
					case 11:	// I/O Busy
						seq_tbl.slv_busy[rid]=0;
						seq_tbl.current++;
						break;
					case 12:	// I/O Read Req
						seq_tbl.slv_busy[rid]=13;
						break;
					case 13:	// I/O Data Transfer 
						seq_tbl.reg_flag=(((int)frame.data[4])<<24) + (((int)frame.data[5])<<16) + (((int)frame.data[6])<<8) + ((int)frame.data[7]);
						sprintf(str, "Port %d-%d = %02X%02X %02X%02Xh", action[seq_tbl.current].slvno, action[seq_tbl.current].mno,
																		(unsigned char)(frame.data[4]), (unsigned char)(frame.data[5]),
																		(unsigned char)(frame.data[6]), (unsigned char)(frame.data[7]));
						message(sock, seq_tbl.my_thread_no, 1, 3, str);

						seq_tbl.slv_busy[rid]=0;
						seq_tbl.current++;
						break;
					default:
						break;
					}
				}
		printf("%s %d %d %X\n", __FILE__,__LINE__,seq_tbl.run,frame.can_id);
			}
		}
#endif

		// Local Sokcet
		{
			// STX受信 0:Close -1:nothing
			char c;
			int len = recv(sock, &c, 1, 0);

			// Recv
			if (len>0) {
				memset(buf, 0, sizeof(buf));
				len = nt_recv(sock, c, buf);
		printf("%s %d %d %02X %02X\n", __FILE__,__LINE__,len,(unsigned char)buf[4],(unsigned char)buf[5]);
			}

			// ACK返信
			if (len>0) {
				nt_send(sock, ack_msg_data, sizeof(ack_msg_data));
			}

			// コマンド割り振り
			if (len>0) {
				dispatch(sock, buf);
			}

			// Close
			if (len==0) break;
		}

		// シーケンス
		if (seq_tbl.run) {
			sequence(sock);
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
	int on = 1, ret, i, arg[20];
	pthread_t pthread_id[20];

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

	// スレッド起動
	for (i=0; i<20; i++) {
		arg[i]=i;
		pthread_create( &pthread_id[i], NULL, &thread_main, &arg[i]);
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


