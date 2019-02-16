#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

void interrupt( void );
void setControlFlag( int8_t _flag );

void checkWallOut( void );

#endif /* __TIMER_H */