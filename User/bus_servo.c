#include "bus_servo.h"
#include "usart.h"

#define BUS_SERVO_FRAME_HEADER      0x55
#define BUS_SERVO_CMD_MOVE          0x03

// 你现在如果用的是 USART1，就保持 huart1。
// 如果你用的是 USART2，把这里改成 huart2。
#define BUS_SERVO_UART              huart1

volatile uint32_t g_bus_servo_tx_count;
volatile int32_t g_bus_servo_last_status;
volatile uint16_t g_bus_servo_last_time_ms;
volatile uint16_t g_bus_servo_last_frame_len;
volatile uint8_t g_bus_servo_last_num;

static uint8_t BusServo_LowByte(uint16_t data)
{
    return (uint8_t)(data & 0xFF);
}

static uint8_t BusServo_HighByte(uint16_t data)
{
    return (uint8_t)((data >> 8) & 0xFF);
}

/**
 * @brief 控制单个总线舵机转到指定位置
 * @param id 舵机 ID，例如你当前舵机是 ID7
 * @param position 目标位置，HTS-35H 建议范围 0~1000
 * @param time_ms 运行时间，单位 ms
 */
void BusServo_SetPosition(uint8_t id, uint16_t position, uint16_t time_ms)
{
    uint8_t buf[10];

    if (position > 1000)
    {
        position = 1000;
    }

    if (time_ms < 20)
    {
        time_ms = 20;
    }

    buf[0] = BUS_SERVO_FRAME_HEADER; // 帧头需要发两次
    buf[1] = BUS_SERVO_FRAME_HEADER; // 否则舵机会误以为 0x55 是数据而不是帧头，导致后续数据解析错误
    buf[2] = 8; // 后续数据长度，固定为 8，因为我们只控制一个舵机，数据部分是：命令(1) + 舵机数量(1) + 运行时间(2) + 舵机 ID(1) + 目标位置(2) = 8 字节
    buf[3] = BUS_SERVO_CMD_MOVE; // 命令字，0x03 是控制舵机移动的命令
    buf[4] = 1; // 舵机数量，这里是 1，因为我们只控制一个舵机
    buf[5] = BusServo_LowByte(time_ms);
    buf[6] = BusServo_HighByte(time_ms); // 运行时间，分成低字节和高字节发送
    buf[7] = id; 
    buf[8] = BusServo_LowByte(position);
    buf[9] = BusServo_HighByte(position); // 目标位置，分成低字节和高字节发送

    HAL_UART_Transmit(&BUS_SERVO_UART, buf, sizeof(buf), 100); // 发送数据，长度是 sizeof(buf)，超时时间 100 ms
}

/**
 * @brief 同时控制多个总线舵机
 */
void BusServo_SetPositions(BusServo_t *servos, uint8_t num, uint16_t time_ms)
{
    uint8_t buf[103];
    uint8_t index = 7;
    uint16_t frame_len;
    HAL_StatusTypeDef status;

    if (servos == NULL)
    {
        return;
    }

    if (num < 1 || num > 32)
    {
        return;
    }

    if (time_ms < 20)
    {
        time_ms = 20;
    }

    buf[0] = BUS_SERVO_FRAME_HEADER;
    buf[1] = BUS_SERVO_FRAME_HEADER;
    buf[2] = num * 3 + 5; // 后续数据长度，计算方式是：命令(1) + 舵机数量(1) + 运行时间(2) + 每个舵机数据(3) * 舵机数量
    buf[3] = BUS_SERVO_CMD_MOVE;
    buf[4] = num;
    buf[5] = BusServo_LowByte(time_ms);
    buf[6] = BusServo_HighByte(time_ms);

    for (uint8_t i = 0; i < num; i++)
    {
        uint16_t position = servos[i].position;

        if (position > 1000)
        {
            position = 1000;
        }

        buf[index++] = servos[i].id;
        buf[index++] = BusServo_LowByte(position);
        buf[index++] = BusServo_HighByte(position);
    } // 设置每个舵机的数据，格式是：舵机 ID(1) + 目标位置(2)

    frame_len = (uint16_t)(buf[2] + 2U);
    status = HAL_UART_Transmit(&BUS_SERVO_UART, buf, frame_len, 100);
    g_bus_servo_tx_count++;
    g_bus_servo_last_status = (int32_t)status;
    g_bus_servo_last_time_ms = time_ms;
    g_bus_servo_last_frame_len = frame_len;
    g_bus_servo_last_num = num;
}

/**
 * @brief 测试你当前的 ID7 舵机
 */
void BusServo_TestID7(void)
{
    BusServo_SetPosition(7, 500, 1000);
    HAL_Delay(5000);

    BusServo_SetPosition(7, 600, 1000);
    HAL_Delay(5000);

    BusServo_SetPosition(7, 400, 1000);
    HAL_Delay(5000);

    BusServo_SetPosition(7, 500, 1000);
    HAL_Delay(5000);
}
