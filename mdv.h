#ifndef MDV_H
#define MDV_H

int mdv_serial_init(char *bsd_path);
void mdv_serial_read(int fd, unsigned char *buf);
void mdv_serial_write(int fd, unsigned char *buf);

#endif
