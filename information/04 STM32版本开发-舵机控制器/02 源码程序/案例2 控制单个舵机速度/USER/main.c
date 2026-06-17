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
 	SystemInit();//系统时钟等初始化
	delay_init(72);	     //延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uartInit(9600);//串口初始化为9600
	
	
	while(1){
		moveServo(1, 1000, 1200); //1号舵机至1000位置
		delay_s(2);
		moveServo(1, 0, 1200); //1号舵机至0位置
		delay_s(2);
		moveServo(1, 1000, 500); //1号舵机至1000位置
		delay_s(1);
		moveServo(1, 0, 500); //1号舵机至0位置
		delay_s(1);
 }
}
