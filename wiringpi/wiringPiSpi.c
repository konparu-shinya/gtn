#include <unistd.h>
#include <sys/shm.h>
#include <pthread.h>
#include <sys/wait.h>
#include "ruby.h"
#include "wiringPi.h"
#include "wiringPiSPI.h"

// MAX1000との接続SPI０
#define MAX_SPI_CHANNEL 0

static pthread_mutex_t *mutex;

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
printf("%s %d %d %02X\n", __FILE__, __LINE__, i, (unsigned char)FIX2INT(rb_ary_entry(arg, i)));
			data[i]=FIX2INT(rb_ary_entry(arg, i));
		}
		pthread_mutex_lock(mutex);
		wiringPiSPIDataRW(MAX_SPI_CHANNEL, data, len);
		pthread_mutex_unlock(mutex);

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
	pthread_mutex_lock(mutex);
	/* SPI channel 0 を 1MHz で開始 */
	wiringPiSPISetupMode(MAX_SPI_CHANNEL, FIX2INT(arg1), FIX2INT(arg2));
	pthread_mutex_unlock(mutex);
	return self;
}

static int mutex_init(void)
{
	pthread_mutexattr_t mat;
	int shmid;
	/* mutex用に共有メモリを利用 */
	const key_t key = 112;
	shmid = shmget(key, sizeof(pthread_mutex_t), 0600);
	/* 初回 */
	if (shmid < 0) {
		shmid = shmget(key, sizeof(pthread_mutex_t), 0600|IPC_CREAT);
		if (shmid < 0) {
			return -1;
		}
		mutex = shmat(shmid, NULL, 0);
		if ((intptr_t)mutex == -1) {
			return -1;
		}

		/* mutexのattributeを設定する準備 */
		pthread_mutexattr_init(&mat);

		/* mutexをプロセス間で利用する設定を行う */
		/* これを行わないとプロセス内でのみ有効のmutexになります */
		if (pthread_mutexattr_setpshared(&mat, PTHREAD_PROCESS_SHARED) != 0) {
			return -1;
		}

		pthread_mutex_init(mutex, &mat);

	}
	/* 既に起動済 */
	else{
		mutex = shmat(shmid, NULL, 0);
		if ((intptr_t)mutex == -1) {
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

	mutex_init();
}