#include "mdv.h"

int main (int argc, char** argv) {
  
  unsigned char buf[20];
  int i;

  mdv_init("/dev/cu.usbserial");
  while (1) {
    mdv_read(buf);
    for (i = 0; i < 16; i++) {
      printf("%.2x", buf[i]);
    }
    printf("\n");
  }

  return 0;
}
