#include <unistd.h>
#include <sys/shm.h>
#include <pthread.h>
#include <sys/wait.h>
#include "ruby.h"
#include "wiringPi.h"
#include "wiringPiSPI.h"

// MAX1000との接続SPI０
#define MAX_SPI_CHANNEL 0

struct _shm {
	pthread_mutex_t mutex;	// ミューテックス
	long	count;			// フォトンカウント値
	long	gate_time;		// ゲートタイムmsec
	long	meas_st1;		// 測定開始区間1
	long	meas_ed1;		// 測定終了区間1
	long	meas_st2;		// 測定開始区間2
	long	meas_ed2;		// 測定終了区間2
	long	meas_data1;		// 測定値1
	long	meas_data2;		// 測定値2
	float	fact1_a;		// 反応部温度係数a
	float	fact1_b;		// 反応部温度係数b
	float	fact2_a;		// 外部温度係数a
	float	fact2_b;		// 外部温度係数b
} static *shm;

static VALUE fwiringPiSpi_datarw(VALUE self, VALUE arg)
{
	VALUE ret;
	if (TYPE(arg)!=T_ARRAY) {
		return Qnil;
	}
	else{
		int i;
		int len = RARRAY_LEN(arg);
		unsigned char data[len];
		for (i=0; i<len; i++) {
//printf("%s %d %d %02X\n", __FILE__, __LINE__, i, (unsigned char)FIX2INT(rb_ary_entry(arg, i)));
			data[i]=FIX2INT(rb_ary_entry(arg, i));
		}
		pthread_mutex_lock(&shm->mutex);
		wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, len);
		pthread_mutex_unlock(&shm->mutex);

		ret = rb_ary_new2(len);
		for (i=0; i<len; i++) {
			rb_ary_store(ret, i, INT2FIX(data[i]));
		}
	}
	return ret;
}

static VALUE fwiringPiSpi_setup_mode(VALUE self, VALUE arg1, VALUE arg2)
{
printf("%s %d %d %d\n", __FILE__, __LINE__, FIX2INT(arg1), FIX2INT(arg2));
	pthread_mutex_lock(&shm->mutex);
	/* SPI channel 0 を 1MHz で開始 */
	wiringPiSPISetupMode(MAX_SPI_CHANNEL, FIX2INT(arg1), FIX2INT(arg2));
	pthread_mutex_unlock(&shm->mutex);
	return self;
}

static VALUE fwiringPiSpi_foton_count(VALUE self)
{
	return INT2FIX(shm->count);
}

static VALUE fwiringPiSpi_gate_time(VALUE self, VALUE arg1)
{
	shm->gate_time = FIX2INT(arg1);
	return self;
}

static VALUE fwiringPiSpi_meas_points(VALUE self, VALUE arg1, VALUE arg2, VALUE arg3, VALUE arg4)
{
	shm->meas_st1 = FIX2INT(arg1);
	shm->meas_ed1 = FIX2INT(arg2);
	shm->meas_st2 = FIX2INT(arg3);
	shm->meas_ed2 = FIX2INT(arg4);
	return self;
}

static VALUE fwiringPiSpi_meas_data1(VALUE self)
{
	return INT2FIX(shm->meas_data1);
}

static VALUE fwiringPiSpi_meas_data2(VALUE self)
{
	return INT2FIX(shm->meas_data2);
}

static VALUE fwiringPiSpi_fact2(VALUE self, VALUE arg1, VALUE arg2, VALUE arg3, VALUE arg4)
{
	shm->fact1_a = NUM2DBL(arg1);
	shm->fact1_b = NUM2DBL(arg2);
	shm->fact2_a = NUM2DBL(arg3);
	shm->fact2_b = NUM2DBL(arg4);
	return self;
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
		shm->meas_st1=10L;
		shm->meas_ed1=11L;
		shm->meas_st2=10L;
		shm->meas_ed2=11L;
		shm->meas_data1=0L;
		shm->meas_data2=0L;
		shm->fact1_a=1;
		shm->fact1_b=0;
		shm->fact2_a=1;
		shm->fact2_b=0;
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

void Init_WiringPiSpi(void)
{
	VALUE cWiringPi = rb_define_class("WiringPiSpi", rb_cObject);
	rb_define_method(cWiringPi, "setup_mode", fwiringPiSpi_setup_mode, 2);
	rb_define_method(cWiringPi, "dataRW", fwiringPiSpi_datarw, 1);
	rb_define_method(cWiringPi, "foton_count", fwiringPiSpi_foton_count, 0);
	rb_define_method(cWiringPi, "gate_time", fwiringPiSpi_gate_time, 1);
	rb_define_method(cWiringPi, "meas_points", fwiringPiSpi_meas_points, 4);
	rb_define_method(cWiringPi, "meas_data1", fwiringPiSpi_meas_data1, 0);
	rb_define_method(cWiringPi, "meas_data2", fwiringPiSpi_meas_data2, 0);
	rb_define_method(cWiringPi, "fact2", fwiringPiSpi_fact2, 4);


	mutex_init();
}
