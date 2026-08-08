#ifndef __DOG_TASK_H
#define __DOG_TASK_H

/* 直角三角形足端轨迹参数（宏定义可调）：
 * DOG_TASK_TRI_H_MM 竖直直角边（抬脚高度），命名参考 h 系列；
 * DOG_TASK_TRI_R_MM 水平直角边（底边长度），命名参考 r 系列。 */
#define DOG_TASK_TRI_H_MM   15.0f
#define DOG_TASK_TRI_R_MM   15.0f

void DogTask_Init(void);
void DogTask_Run(void);
void DogTask_RunSpeedBumpTest(void);

#endif
