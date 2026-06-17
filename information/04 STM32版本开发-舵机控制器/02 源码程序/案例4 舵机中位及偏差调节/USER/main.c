#include "delay.h"
#include "uart.h"
#include "LobotServoController.h"
#include "bool.h"

void delay_s(int s)
{
	for(;s>0;s--)
		delay_ms(1000);
}

 int main(void)
 {
	int deviation;
 	SystemInit();//系统时钟等初始化
	delay_init(72);	     //延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uartInit(9600);//串口初始化为9600

	moveServo(1, 500, 500);	//1号舵机，转动到位置500，用时500ms
	delay_s(2);	//延时程序，这里延时2s
	deviation = 80;
	moveServo(1, 500+deviation, 200);	//1号舵机，转动到位置580，用时200ms
	while(1);
}
