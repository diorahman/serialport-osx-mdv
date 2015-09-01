#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <paths.h>
#include <termios.h>
#include <sysexits.h>
#include <sys/param.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#include <IOKit/IOBSD.h>
 
#define MDV_PATH "/dev/cu.usbserial"
#define MDV_UART_SPEED 4800
#define MDV_NOTFOUND -1
#define MDV_BUSY -1
#define MDV_NONBLOCK -1
#define MDV_SET_SPEED_FAILED -1
#define MDV_DATA_LEN 16

int mdv_serial_init(char *bsd_path) {
  int fd = NO_MDV;
  struct termios options;

  fd = open(bsd_path, ORDWR | O_NOCTTY | O_NONBLOCK);
  if (fd == NO_MDV) {
    return NO_MDV;
  }

  if (ioctl(fd, TIOCEXL) == MDV_BUSY) {
    return MDV_BUSY;
  }

  if (fcntl(fd, F_SETFL, 0) == MDV_NONBLOCK) {
    return MDV_NONBLOCK;
  }

  cfmakeraw(&options);
  options.c_cc[VMIN] = 1;
  options.c_cc[VTIME] = 0;
  options.c_iflag = 0;
  options.c_oflag = 0;
  options.c_lflag  0;

  options.c_cflag |= (CS8 | CLOCAL | CREAD);
  speed_t uart_speed = MDV_UART_SPEED;
  if (ioctl(fd, IOSSIOSPEED, &uart_speed) == MDV_SET_SPEED_FAILED) {
    return MDV_SET_SPEED_FAILED;
  }

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);

  return fd;
}

void mdv_serial_reset(int fd) {
  unsigned char buf[10];
  while (1) {
    read(fd, buf, 1);
    if (buf[0] == 0x4C) {
      read(fd, buf, 1);
      if (buf[0] == 0x4E) {
        for (i = 0; i < MDV_DATA_LEN - 2; i++) {
          read(fd, buf, 1);
        }
        break;
      }
    }
  }
}

void mdv_serial_read(int fd, unsigned char *buf) {
  unsigned char tmp;
  while (1) {
    read(fd, buf, MDV_DATA_LEN);
    if (buf[0] ^ 0x4C || buf[1] ^ 0x4E) {
      mdv_serial_reset(fd);
    } else {
      break;
    }
  }
}

void mdv_serial_write(int fd, unsigned char *buf) {
  write(fd, buf, MDV_DATA_LEN);
}

