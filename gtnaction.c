/********************************************************************************
 * gtnのAction実行部
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

// 動作管理テーブル
struct _seq_tbl {
	int run;			// 0:停止 1:実行中
	int count;			// 動作中の繰り返し回数
	int current;		// 動作中のカレント行番号
	int max_line;		// actionテーブルの最大行数
	int my_thread_no;	// コンソールから受け取ったスレッド番号
	int run_times;		// コンソールから受け取った繰り返し回数
	int reg_flag;		// レジスタ読み込み値の保存
} static seq_tbl={0,0,0,0,0,0,0};


// 動作シーケンス格納テーブル
struct _action_tbl {
	int line;
	int node;
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
} static action[1000];

static struct timespec tim_start;

#define CAN_NAME "can0"
#define	ID	0x13
#define	DLC	8

// CAN送信
static int can_send(int can, unsigned char data[])
{
	struct can_frame frame;
	memset(&frame, 0, sizeof(frame));
	frame.can_id  = ID;
	frame.can_dlc = DLC;
	memcpy(&(frame.data[0]), &(data[0]), DLC); 

   	write (can, &frame, CAN_MTU);
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
	int len, length, tm, i, count=0;
	// STX
	if (c!=STX) return -1;
	p[count++] = c;

	// length
	for (tm=0; tm<TIMEOUT; tm++) {
		len = recv(sock, &c, 1, 0);
		if (len==0) return 0;
		if (len>0) break;
		usleep(1000);
	}
	if (tm>=TIMEOUT) return -1;
	length = (int)c;
	p[count++] = c;

	for (i=1; i<length; i++) {
		for (tm=0; tm<TIMEOUT; tm++) {
			len = recv(sock, &c, 1, 0);
			if (len==0) return 0;
			if (len>0) break;
			usleep(1000);
		}
		if (tm>=TIMEOUT) return -1;
		p[count++] = c;
	}

	// ETX
	for (tm=0; tm<TIMEOUT; tm++) {
		len = recv(sock, &c, 1, 0);
		if (len==0) return 0;
		if (len>0) break;
		usleep(1000);
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

// コマンド受信処理
static int dispatch(int sock, char *buf, int can)
{
	char str[32];
	unsigned short id = ((unsigned short)((unsigned char)buf[4])<<8) + (unsigned short)((unsigned char)buf[5]);

	// パルスモーター設定
	if (id==0xC101 && can>=0) {
 unsigned char data[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
		can_send(can, data);
	}
	// DCモーター設定
	else if (id==0xC103 && can>=0) {
	}
	// 動作シーケンス
	else if (id==0xC102) {
		action[seq_tbl.max_line].line        = ((int)((unsigned char)buf[6])<<8) + (int)((unsigned char)buf[7]);
		action[seq_tbl.max_line].node        = (int)((unsigned char)buf[8]);
		action[seq_tbl.max_line].mno         = (int)((unsigned char)buf[9]);
		action[seq_tbl.max_line].act         = ((int)((unsigned char)buf[10])<<8) + (int)((unsigned char)buf[11]);
		action[seq_tbl.max_line].move_pulse  = ((unsigned long)((unsigned char)buf[12])<<24) + ((unsigned long)((unsigned char)buf[13])<<16) + ((unsigned long)((unsigned char)buf[14])<<8) + (unsigned long)((unsigned char)buf[15]);
		action[seq_tbl.max_line].start_pulse = ((int)((unsigned char)buf[16])<<8) + (int)((unsigned char)buf[17]);
		action[seq_tbl.max_line].max_pulse   = ((int)((unsigned char)buf[18])<<8) + (int)((unsigned char)buf[19]);
		action[seq_tbl.max_line].st_slope    = ((int)((unsigned char)buf[20])<<8) + (int)((unsigned char)buf[21]);
		action[seq_tbl.max_line].ed_slope    = ((int)((unsigned char)buf[22])<<8) + (int)((unsigned char)buf[23]);
		action[seq_tbl.max_line].ratio       = ((int)((unsigned char)buf[24])<<8) + (int)((unsigned char)buf[25]);
		seq_tbl.max_line++;
	}
	// 動作準備
	else if (id==0xC014) {
		memset(&seq_tbl, 0, sizeof(seq_tbl));
		seq_tbl.my_thread_no = (int)((unsigned char)buf[6]);
		message(sock, seq_tbl.my_thread_no, 1, 1, "START");
		message(sock, seq_tbl.my_thread_no, 1, 2, "");
		message(sock, seq_tbl.my_thread_no, 1, 3, "");
	}
	// 動作開始
	else if (id==0xC015) {
		if (can>=0) {
			seq_tbl.my_thread_no = (int)((unsigned char)buf[6]);
			seq_tbl.run_times    = ((int)((unsigned char)buf[7])<<24) + ((int)((unsigned char)buf[8])<<16) + ((int)((unsigned char)buf[9])<<8) + (int)((unsigned char)buf[10]);
			seq_tbl.run          = 1;
			message(sock, seq_tbl.my_thread_no, 1, 1, "RUN");
			clock_gettime(CLOCK_MONOTONIC, &tim_start);
		}
		else{
			message(sock, seq_tbl.my_thread_no, 1, 1, "CAN Socket Error");
		}

	}
	// 動作停止
	else if (id==0xC016) {
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
	return 0;
}

// シーケンス
static int sequence(int sock, int can)
{
    int i;
	char str[32];

	switch (action[seq_tbl.current].act) {
	case 0x01:		// DCモーター初期化
		break;
	case 0x02:		// DCモーターCW(回転時間指定)
		break;
	case 0x03:		// DCモーターCW(STEPセンサーまで回転)
		break;
	case 0x04:		// DCモーターCCW(回転時間指定)
		break;
	case 0x05:		// DCモーターCCW(HOMEセンサーまで回転)
		break;
	case 0x11:		// パルスモーター初期化
		break;
	case 0x12:		// パルスモーターCW(相対パルス指定)
		break;
	case 0x13:		// パルスモーターCCW(相対パルス指定)
		break;
	case 0x14:		// パルスモーターHOME(現在位置パルス分をCCW回転する)
		break;
	case 0x15:		// パルスモーター絶対パルス移動
		break;
	case 0x16:		// パルスモーター停止までまつ
		break;
	case 0x17:		// パルスモーター励磁解除
		break;
	case 0x18:		// パルスモーター励磁ON
		break;
	case 0x31:		// パルスモーターI/O指定ビットON
		break;
	case 0x32:		// パルスモーターI/O指定ビットOFF
		break;
	case 0x33:		// パルスモーターI/O指定ビットをONまでまつ
		break;
	case 0x34:		// パルスモーターI/O指定ビットをOFFまでまつ
		break;
	case 0x35:		// パルスモーターI/O 16bitデータ書き込み
		break;
	case 0x36:		// パルスモーターI/O 16bitデータ読み込んで表示
		break;
	case 0x41:		// DIO指定ビットON
		break;
	case 0x42:		// DIO指定ビットOFF
		break;
	case 0x43:		// DIO指定ビットをONまでまつ
		break;
	case 0x44:		// DIO指定ビットをOFFまでまつ
		break;
	case 0x45:		// DIO 32bitデータ書き込み
		break;
	case 0x46:		// DIO 32bitデータ読み込んで表示
		break;
	case 0x51:		// 指定時間まち(×10msec)
		usleep(action[seq_tbl.current].move_pulse*1000);
		seq_tbl.current++;
		break;
	case 0x52:		// if文
		break;
	case 0x53:		// unless文
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
		seq_tbl.count++;
		break;
	}

	// 終了判定
	if (seq_tbl.current >= seq_tbl.max_line) {
		seq_tbl.count++;
		// 終了
		if (seq_tbl.count >= seq_tbl.run_times) {
			seq_tbl.run = 0;
			message(sock, seq_tbl.my_thread_no, 1, 1, "Action success!!");
		}
		// Next
		else{
			uint64_t msec, sec;
			struct timespec tim_end;

			seq_tbl.current = 0;

			clock_gettime(CLOCK_MONOTONIC, &tim_end);
			if((tim_end.tv_nsec - tim_start.tv_nsec) < 0){
				tim_end.tv_nsec += 1000000000;
				tim_end.tv_sec  -= 1;
			}
			msec = (tim_end.tv_nsec - tim_start.tv_nsec)/1000000;
			sec  = tim_end.tv_sec - tim_start.tv_sec;

			sprintf(str, "Loop:%d Time:%lu.%lu秒", seq_tbl.count, sec, msec/100);
			message(sock, seq_tbl.my_thread_no, 1, 1, str);
		}
	}

	return 0;
}

// 全体の実行処理
static int execute(int sock, int can)
{
	char buf[512];

	while (1) {
		// CAN Sokcet
		if (can>=0) {
			struct can_frame frame;
			int len = read(can, &frame, sizeof(frame));
			if (len==sizeof(frame)) {
//  *p_id = frame.can_id;
//  *p_dlc = frame.can_dlc;
//  memcpy(data, frame.data, CAN_MAX_DLEN);
		printf("%s %d %d\n", __FILE__,__LINE__,len);
			}
		}

		// Local Sokcet
		{
			// STX受信 0:Close -1:nothing
			char c;
			int len = recv(sock, &c, 1, 0);

			// Recv
			if (len>0) {
				len = nt_recv(sock, c, buf);
		printf("%s %d %d %02X %02X\n", __FILE__,__LINE__,len,(unsigned char)buf[4],(unsigned char)buf[5]);
			}

			// ACK返信
			if (len>0) {
				nt_send(sock, ack_msg_data, sizeof(ack_msg_data));
			}

			// コマンド割り振り
			if (len>0) {
				dispatch(sock, buf, can);
			}

			// Close
			if (len==0) break;
		}

		// シーケンス
		if (seq_tbl.run) {
			sequence(sock, can);
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
	int servSock;				//server socket descripter
	int clitSock;				//client socket descripter
	int on, ret;
	struct sockaddr_in servSockAddr;	//server internet socket address
	struct sockaddr_in clitSockAddr;	//client internet socket address
	unsigned short servPort = 9001;		//server port number
	unsigned int clitLen;			// client internet socket address length
	int canSock = can_init();

	if ((servSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 ){
		perror("socket() failed.");
		exit(EXIT_FAILURE);
	}
	/* Enable address reuse */
	on = 1;
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
		if (ret != -1) {
			execute(clitSock, canSock);
		}

		close(clitSock);
		printf("close\n");
	}
	close(servSock);

	if (canSock>=0) {
		close(canSock);
	}

	return 0;
}


