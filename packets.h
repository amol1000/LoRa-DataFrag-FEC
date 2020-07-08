#ifndef __PACKETS_H__
#define __PACKETS_H__

typedef struct dataFragment {
    uint8_t seqNum;
    uint8_t data[10]; //frag_size
    uint8_t padding[10]; //since total size is 21.. need to change
} dataFrag;

#endif