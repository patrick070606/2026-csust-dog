#ifndef __JY61P_IMU_H
#define __JY61P_IMU_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint8_t is_ready;
    uint8_t is_valid;
    uint8_t has_accel;
    uint8_t has_gyro;
    uint8_t has_angle;
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t last_update_ms;
    uint32_t frame_count;
    uint32_t checksum_error_count;
} Jy61PImuStatus_t;

void Jy61PImu_Init(void);
void Jy61PImu_Update(uint32_t now_ms);
uint8_t Jy61PImu_GetStatus(Jy61PImuStatus_t *status);
void Jy61PImu_OnUartRxCplt(UART_HandleTypeDef *huart);
/* Called by HAL_UART_ErrorCallback to recover USART3 IMU reception. */
void Jy61PImu_OnUartError(UART_HandleTypeDef *huart);

/* Mirrors kept as simple globals so Cortex Live Watch can read them reliably. */
extern volatile float g_jy61p_imu_roll_deg;
extern volatile float g_jy61p_imu_pitch_deg;
extern volatile float g_jy61p_imu_yaw_deg;
extern volatile float g_jy61p_imu_gyro_z_dps;
extern volatile uint8_t g_jy61p_imu_is_valid;
extern volatile uint32_t g_jy61p_imu_frame_count;
extern volatile uint32_t g_jy61p_imu_checksum_error_count;
/* UART diagnostics: error count, last HAL error code, and RX re-arm count. */
extern volatile uint32_t g_jy61p_imu_uart_error_count;
extern volatile uint32_t g_jy61p_imu_last_uart_error;
extern volatile uint32_t g_jy61p_imu_rx_restart_count;

#endif
