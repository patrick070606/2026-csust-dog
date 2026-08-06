#include "jy61p_imu.h"
#include "usart.h"

#define JY61P_IMU_UART                  huart2
#define JY61P_IMU_FRAME_HEAD            0x55U
#define JY61P_IMU_FRAME_LEN             11U
#define JY61P_IMU_FRAME_ACCEL           0x51U
#define JY61P_IMU_FRAME_GYRO            0x52U
#define JY61P_IMU_FRAME_ANGLE           0x53U
#define JY61P_IMU_STALE_TIMEOUT_MS      500U
#define JY61P_IMU_ACC_SCALE_G           (16.0f / 32768.0f)
#define JY61P_IMU_GYRO_SCALE_DPS        (2000.0f / 32768.0f)
#define JY61P_IMU_ANGLE_SCALE_DEG       (180.0f / 32768.0f)
#define JY61P_IMU_ROLL_BASELINE_DEG     180.0f

/* The IMU is mounted 180 degrees about the robot Z axis: X/Y are reversed,
 * while Z keeps the same direction.  Convert every raw vector at this
 * boundary so all users see robot-body coordinates. */
#define JY61P_IMU_BODY_X_SIGN           (-1.0f)
#define JY61P_IMU_BODY_Y_SIGN           (-1.0f)
#define JY61P_IMU_BODY_Z_SIGN           (1.0f)

/* These offsets are expressed in the robot-body frame.  Recalibrate them
 * after changing the IMU coordinate convention. */
#define JY61P_IMU_ROLL_OFFSET_DEG        (-0.8459f)
#define JY61P_IMU_PITCH_OFFSET_DEG       (0.6427f)


static uint8_t s_rx_data;
static uint8_t s_frame[JY61P_IMU_FRAME_LEN];
static uint8_t s_frame_len;
static volatile Jy61PImuStatus_t s_status;

volatile float g_jy61p_imu_roll_deg;
volatile float g_jy61p_imu_pitch_deg;
volatile float g_jy61p_imu_yaw_deg;
volatile float g_jy61p_imu_gyro_z_dps;
volatile uint8_t g_jy61p_imu_is_valid;
volatile uint32_t g_jy61p_imu_frame_count;
volatile uint32_t g_jy61p_imu_checksum_error_count;

/* Live Watch counters for checking whether USART2 reception stalls. */
volatile uint32_t g_jy61p_imu_uart_error_count;
volatile uint32_t g_jy61p_imu_last_uart_error;
volatile uint32_t g_jy61p_imu_rx_restart_count;

static int16_t Jy61PImu_ReadInt16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint8_t Jy61PImu_ChecksumOk(const uint8_t frame[JY61P_IMU_FRAME_LEN])
{
    uint8_t sum = 0U;

    for (uint8_t i = 0U; i < (JY61P_IMU_FRAME_LEN - 1U); i++)
    {
        sum = (uint8_t)(sum + frame[i]);
    }

    return (uint8_t)(sum == frame[JY61P_IMU_FRAME_LEN - 1U]);
}



static void Jy61PImu_UpdateDebugMirrors(void)
{
    g_jy61p_imu_roll_deg = s_status.roll_deg;
    g_jy61p_imu_pitch_deg = s_status.pitch_deg;
    g_jy61p_imu_yaw_deg = s_status.yaw_deg;
    g_jy61p_imu_gyro_z_dps = s_status.gyro_z_dps;
    g_jy61p_imu_is_valid = s_status.is_valid;
    g_jy61p_imu_frame_count = s_status.frame_count;
    g_jy61p_imu_checksum_error_count = s_status.checksum_error_count;
}

static void Jy61PImu_ParseFrame(const uint8_t frame[JY61P_IMU_FRAME_LEN])
{
    uint8_t type = frame[1];
    int16_t x = Jy61PImu_ReadInt16(&frame[2]);
    int16_t y = Jy61PImu_ReadInt16(&frame[4]);
    int16_t z = Jy61PImu_ReadInt16(&frame[6]);

    if (Jy61PImu_ChecksumOk(frame) == 0U)
    {
        s_status.checksum_error_count++;
        Jy61PImu_UpdateDebugMirrors();
        return;
    }

    if (type == JY61P_IMU_FRAME_ACCEL)
    {
        s_status.acc_x_g = (float)x * JY61P_IMU_ACC_SCALE_G * JY61P_IMU_BODY_X_SIGN;
        s_status.acc_y_g = (float)y * JY61P_IMU_ACC_SCALE_G * JY61P_IMU_BODY_Y_SIGN;
        s_status.acc_z_g = (float)z * JY61P_IMU_ACC_SCALE_G * JY61P_IMU_BODY_Z_SIGN;
        s_status.has_accel = 1U;
    }
    else if (type == JY61P_IMU_FRAME_GYRO)
    {
        s_status.gyro_x_dps = (float)x * JY61P_IMU_GYRO_SCALE_DPS * JY61P_IMU_BODY_X_SIGN;
        s_status.gyro_y_dps = (float)y * JY61P_IMU_GYRO_SCALE_DPS * JY61P_IMU_BODY_Y_SIGN;
        s_status.gyro_z_dps = (float)z * JY61P_IMU_GYRO_SCALE_DPS * JY61P_IMU_BODY_Z_SIGN;
        s_status.has_gyro = 1U;
    }
    else if (type == JY61P_IMU_FRAME_ANGLE)
    {
        s_status.roll_deg =
            ((float)x * JY61P_IMU_ANGLE_SCALE_DEG * JY61P_IMU_BODY_X_SIGN) -
            JY61P_IMU_ROLL_OFFSET_DEG;
        s_status.pitch_deg =
            ((float)y * JY61P_IMU_ANGLE_SCALE_DEG * JY61P_IMU_BODY_Y_SIGN) -
            JY61P_IMU_PITCH_OFFSET_DEG;
        s_status.yaw_deg =
            (float)z * JY61P_IMU_ANGLE_SCALE_DEG * JY61P_IMU_BODY_Z_SIGN;
        s_status.has_angle = 1U;
    }
    else
    {
        return;
    }

    s_status.last_update_ms = HAL_GetTick();
    s_status.frame_count++;
    s_status.is_valid = (uint8_t)((s_status.has_gyro != 0U) || (s_status.has_angle != 0U));
    Jy61PImu_UpdateDebugMirrors();
}

static void Jy61PImu_ProcessByte(uint8_t data)
{
    if (s_frame_len == 0U)
    {
        if (data == JY61P_IMU_FRAME_HEAD)
        {
            s_frame[s_frame_len] = data;
            s_frame_len++;
        }
        return;
    }

    if ((s_frame_len == 1U) &&
        (data != JY61P_IMU_FRAME_ACCEL) &&
        (data != JY61P_IMU_FRAME_GYRO) &&
        (data != JY61P_IMU_FRAME_ANGLE))
    {
        s_frame_len = (data == JY61P_IMU_FRAME_HEAD) ? 1U : 0U;
        s_frame[0] = data;
        return;
    }

    s_frame[s_frame_len] = data;
    s_frame_len++;

    if (s_frame_len >= JY61P_IMU_FRAME_LEN)
    {
        Jy61PImu_ParseFrame(s_frame);
        s_frame_len = 0U;
    }
}

void Jy61PImu_Init(void)
{
    __disable_irq();
    s_status.is_ready = 0U;
    s_status.is_valid = 0U;
    s_status.has_accel = 0U;
    s_status.has_gyro = 0U;
    s_status.has_angle = 0U;
    s_status.acc_x_g = 0.0f;
    s_status.acc_y_g = 0.0f;
    s_status.acc_z_g = 0.0f;
    s_status.gyro_x_dps = 0.0f;
    s_status.gyro_y_dps = 0.0f;
    s_status.gyro_z_dps = 0.0f;
    s_status.roll_deg = 0.0f;
    s_status.pitch_deg = 0.0f;
    s_status.yaw_deg = 0.0f;
    s_status.last_update_ms = 0U;
    s_status.frame_count = 0U;
    s_status.checksum_error_count = 0U;
    s_frame_len = 0U;
    Jy61PImu_UpdateDebugMirrors();
    __enable_irq();

    if (HAL_UART_Receive_IT(&JY61P_IMU_UART, &s_rx_data, 1U) == HAL_OK)
    {
        s_status.is_ready = 1U;
    }
}

void Jy61PImu_Update(uint32_t now_ms)
{
    if ((s_status.is_ready != 0U) &&
        (s_status.last_update_ms != 0U) &&
        ((uint32_t)(now_ms - s_status.last_update_ms) > JY61P_IMU_STALE_TIMEOUT_MS))
    {
        s_status.is_valid = 0U;
        Jy61PImu_UpdateDebugMirrors();
    }
}

uint8_t Jy61PImu_GetStatus(Jy61PImuStatus_t *status)
{
    if (status == 0)
    {
        return 0U;
    }

    __disable_irq();
    *status = s_status;
    __enable_irq();

    return status->is_valid;
}

void Jy61PImu_OnUartRxCplt(UART_HandleTypeDef *huart)
{
    if ((huart == 0) || (huart->Instance != USART2))
    {
        return;
    }

    Jy61PImu_ProcessByte(s_rx_data);

    /* Re-arm the one-byte interrupt receive after every byte from the IMU. */
    if (HAL_UART_Receive_IT(&JY61P_IMU_UART, &s_rx_data, 1U) == HAL_OK)
    {
        g_jy61p_imu_rx_restart_count++;
    }
}

void Jy61PImu_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart == 0) || (huart->Instance != USART2))
    {
        return;
    }

    g_jy61p_imu_uart_error_count++;
    g_jy61p_imu_last_uart_error = huart->ErrorCode;
    s_frame_len = 0U;

    /* Clear transient UART errors so HAL_UART_Receive_IT can be armed again. */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    if (HAL_UART_Receive_IT(&JY61P_IMU_UART, &s_rx_data, 1U) == HAL_OK)
    {
        g_jy61p_imu_rx_restart_count++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Jy61PImu_OnUartError(huart);
}
