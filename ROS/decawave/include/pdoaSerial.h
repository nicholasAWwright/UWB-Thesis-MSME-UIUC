#ifndef _pdoaDriver_h
#define _pdoaDriver_h

typedef struct pdoa_s
{
    int addr;  //(uint16_t) tag address
    int count; //(uint8_t) consecutive transmissions
    int dist;  //(long int) distance from tag to node in mm
    int pdoa;  //(long int) phase difference of arrival in mDeg
}pdoa_t;

#endif

