#ifndef __STAIR_WALK_H
#define __STAIR_WALK_H

#include <stdint.h>

extern volatile uint8_t g_stair_walk_stage;
extern volatile uint8_t g_stair_walk_pitch_band_mm;
extern volatile uint8_t g_stair_walk_pitch_stable_samples;

void StairWalk_Init(void);
void StairWalk_Start(void);
void StairWalk_Update(void);
float StairWalk_GetFilteredRollDeg(void);
uint8_t StairWalk_IsFinished(void);
uint8_t StairWalk_IsRunning(void);

#endif
