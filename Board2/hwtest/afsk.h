#ifndef _AFSK_H_INCLUDED_
#define _AFSK_H_INCLUDED_

#include <stdint.h>

void afsk_setup(void);

void afsk_send(uint8_t *message, uint8_t lengthInBits);
void afsk_stop(void);

#endif
