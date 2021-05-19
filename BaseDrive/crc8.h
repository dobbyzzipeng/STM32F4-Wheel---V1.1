#ifndef __CRC8_H
#define __CRC8_H

#ifdef __cplusplus
 extern "C" {
#endif

unsigned char CRC8_Table(unsigned char *p, char counter);
unsigned char CRC8_Table_Check(unsigned char *p, char counter,char CRC8);

#ifdef __cplusplus
}
#endif

#endif
