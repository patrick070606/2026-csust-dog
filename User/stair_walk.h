#ifndef __STAIR_WALK_H
#define __STAIR_WALK_H

#include <stdint.h>

void StairWalk_Init(void);
void StairWalk_Start(void);
void StairWalk_Update(void);
uint8_t StairWalk_IsFinished(void);
uint8_t StairWalk_IsRunning(void);

#endif
