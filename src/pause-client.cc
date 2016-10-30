
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <pruss_intc_mapping.h>
#include <prussdrv.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include<time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/time.h>

#include "motor-interface-constants.h"
#include "motion-queue.h"

// Target PRU
#define PRU_NUM 0

#if PRU_NUM == 0
#  define PRU_DATARAM PRUSS0_PRU0_DATARAM
#elif PRU_NUM == 1
#  define PRU_DATARAM PRUSS0_PRU1_DATARAM
#endif

// Mirror of the actual PRU struct in beagleg
struct PRUCommunication {
  volatile struct QueueStatus status;
  volatile struct MotionSegment ring_buffer[QUEUE_LEN];
  volatile uint32_t time_factor;
} __attribute__((packed));

enum state {
  UNDEF,
  AUX,
  PAUSE,
  RESUME
};

void anlog_control(const int analog_pin,
                   volatile struct PRUCommunication *pru_data) {
  int fd;
  char path[100];
  char raw[4];
  sprintf(path,
          "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw",
          analog_pin);
  fd = open(path, O_RDONLY);
  if (fd < 0) {
    fprintf(stderr, "ADC - problem opening ADC");
  }

  float val = -1;
  for (;;) {
    read(fd, &raw, 4);
    val = atoi(raw) * 1.0 / (1 << 12);
    printf("%%: %f\n", val);
    val = 1.0 / val;
    pru_data->time_factor = val * (1 << 16);
    lseek(fd, 0, SEEK_SET);
    usleep(1e4);
  }

  close(fd);
}

float get_time(struct timeval s_time) {
  struct timeval t;
  gettimeofday(&t, 0);
  long elapsed = (t.tv_sec - s_time.tv_sec) * 1e6 + t.tv_usec - s_time.tv_usec;
  return elapsed / 1e6f;
}

void start(const enum state action, const float t,
           volatile struct PRUCommunication *pru_data) {

  struct timeval start_time;
  gettimeofday(&start_time, 0);
  float ct;
  float factor;

  ct = get_time(start_time);

  if (action == PAUSE) {
    while(ct < t) {
      factor = 1 - ct / t;
      factor = 1 / factor;
      pru_data->time_factor = factor * (1 << 16);
      ct = get_time(start_time);
    }
    pru_data->time_factor = 0xffffffff;
    printf("Pause Completed\n");
  } else {
    while(ct < t) {
      factor = ct / t;
      factor = 1 / factor;
      pru_data->time_factor = factor * (1 << 16);
      ct = get_time(start_time);
    }
    pru_data->time_factor = 0x00010000;
    printf("Resume Completed\n");
  }
}

static int usage(const char *progname) {
    fprintf(stderr, "usage: %s [option]\n", progname);
    fprintf(stderr, "Options:\n"
            "\t-a <number>      : Use analog to change realtime speed.\n"
            "\t-p <seconds>     : Pause in <seconds>.\n"
            "\t-r <seconds>     : Resume in <seconds>.\n"
            );
    return 1;
}

int main(int argc, char *argv[]) {

  if (argc < 2) {
    fprintf(stderr, "Mandatory argument(s) missing\n");
    return usage(argv[0]);
  }

  float action_time = -1;
  enum state action= UNDEF;
  int opt;
  int analog_pin = -1;
  while ((opt = getopt(argc, argv, "a:p:r:")) != -1) {
    switch (opt) {
    case 'p':
      action_time = atof(optarg);
      if (action_time < 0) {
          fprintf(stderr, "Invalide negative ratio.'%f'\n", action_time);
          return usage(argv[0]);
      }
      action = PAUSE;
      break;
    case 'r':
      action_time = atof(optarg);
      if (action_time < 0) {
          fprintf(stderr, "Invalide negative ratio.'%f'\n", action_time);
          return usage(argv[0]);
      }
      action = RESUME;
      break;
    case 'a':
      analog_pin = atoi(optarg);
      action = AUX;
      break;
    default:
      return usage(argv[0]);
    }
  }

  volatile struct PRUCommunication *pru_data;

  // Init pru memory
  prussdrv_init();
  /* Get the interrupt initialized */
  int ret = prussdrv_open(PRU_EVTOUT_0);  // allow access.
  if (ret) {
    printf("prussdrv_open() failed (%d) %s\n", ret, strerror(errno));
    return 1;
  }
  void *pru_mmap;
  // map the struct
  prussdrv_map_prumem(PRU_DATARAM, &pru_mmap);
  if (pru_mmap == NULL) {
    printf("Couldn't map PRU memory.\n");
    return 1;
  }

  pru_data = (struct PRUCommunication *) pru_mmap;

  if (action != AUX) {
    start(action, action_time, pru_data);
  } else {
    anlog_control(analog_pin, pru_data);
  }

  return 0;
}
