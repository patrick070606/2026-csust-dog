#include "image_command.h"
#include "usart.h"

#define IMAGE_COMMAND_UART huart2
#define IMAGE_FRAME_MAX_LEN 48U
#define IMAGE_FIELD_NOT_FOUND 0xFFU
#define IMAGE_EVENT_TURN_RIGHT 1000
#define IMAGE_EVENT_TURN_LEFT  (-1000)
#define IMAGE_EVENT_PLATFORM   2000
#define IMAGE_EVENT_PURPLE     3000
#define IMAGE_EVENT_BROWN      4000
#define IMAGE_EVENT_LOST       9999

static uint8_t s_rx_data;
static volatile ImageCommand_t s_latest_command = IMAGE_COMMAND_NONE;
static volatile uint8_t s_has_command;
static volatile ImageTrack_t s_latest_track;
static volatile uint8_t s_has_track;
static char s_frame[IMAGE_FRAME_MAX_LEN];
static uint8_t s_frame_len;

static uint8_t ImageCommand_IsDigit(uint8_t data)
{
    return (uint8_t)((data >= (uint8_t)'0') && (data <= (uint8_t)'9'));
}

static uint8_t ImageCommand_ToUpper(char data)
{
    if ((data >= 'a') && (data <= 'z'))
    {
        return (uint8_t)(data - ('a' - 'A'));
    }

    return (uint8_t)data;
}

static uint8_t ImageCommand_FindField(const char *frame, uint8_t len, char field)
{
    uint8_t target = ImageCommand_ToUpper(field);

    if (frame == 0)
    {
        return IMAGE_FIELD_NOT_FOUND;
    }

    for (uint8_t i = 0U; (uint8_t)(i + 1U) < len; i++)
    {
        if ((ImageCommand_ToUpper(frame[i]) == target) &&
            (frame[i + 1U] == ':'))
        {
            return (uint8_t)(i + 2U);
        }
    }

    return IMAGE_FIELD_NOT_FOUND;
}

static uint8_t ImageCommand_IsFieldDelimiter(char data)
{
    return (uint8_t)((data == ',') ||
                     (data == ' ') ||
                     (data == '\t'));
}

static uint8_t ImageCommand_MatchToken(const char *frame,
                                       uint8_t len,
                                       uint8_t start,
                                       const char *token)
{
    uint8_t i = start;
    uint8_t token_len = 0U;

    if ((frame == 0) || (token == 0))
    {
        return 0U;
    }

    while ((i < len) && (ImageCommand_IsFieldDelimiter(frame[i]) == 0U))
    {
        i++;
    }

    while (token[token_len] != '\0')
    {
        token_len++;
    }

    if ((uint8_t)(i - start) != token_len)
    {
        return 0U;
    }

    for (uint8_t j = 0U; j < token_len; j++)
    {
        if (ImageCommand_ToUpper(frame[(uint8_t)(start + j)]) !=
            ImageCommand_ToUpper(token[j]))
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t ImageCommand_ParseErrorField(const char *frame, uint8_t len, int16_t *error)
{
    uint8_t i;
    int16_t value = 0;
    int8_t sign = 1;
    uint8_t has_digit = 0U;

    if ((frame == 0) || (error == 0))
    {
        return 0U;
    }

    i = ImageCommand_FindField(frame, len, 'E');
    if (i == IMAGE_FIELD_NOT_FOUND)
    {
        i = 0U;
    }

    while ((i < len) && ((frame[i] == ' ') || (frame[i] == '\t')))
    {
        i++;
    }

    if (i >= len)
    {
        return 0U;
    }

    if (frame[i] == '-')
    {
        sign = -1;
        i++;
    }
    else if (frame[i] == '+')
    {
        i++;
    }

    while ((i < len) && ImageCommand_IsDigit((uint8_t)frame[i]) != 0U)
    {
        has_digit = 1U;
        value = (int16_t)((value * 10) + (frame[i] - '0'));
        i++;
    }

    if (has_digit == 0U)
    {
        return 0U;
    }

    *error = (int16_t)(sign * value);
    return 1U;
}

static uint8_t ImageCommand_ParseColorCommand(const char *frame,
                                             uint8_t len,
                                             ImageCommand_t *command)
{
    uint8_t i;

    if ((frame == 0) || (command == 0))
    {
        return 0U;
    }

    i = ImageCommand_FindField(frame, len, 'C');
    if (i == IMAGE_FIELD_NOT_FOUND)
    {
        return 0U;
    }

    while ((i < len) && ((frame[i] == ' ') || (frame[i] == '\t')))
    {
        i++;
    }

    if (i >= len)
    {
        return 0U;
    }

    if (ImageCommand_MatchToken(frame, len, i, "blue") != 0U)
    {
        *command = IMAGE_COMMAND_PLATFORM;
        return 1U;
    }

    if (ImageCommand_MatchToken(frame, len, i, "purple") != 0U)
    {
        *command = IMAGE_COMMAND_PURPLE;
        return 1U;
    }

    if (ImageCommand_MatchToken(frame, len, i, "brown") != 0U)
    {
        *command = IMAGE_COMMAND_BROWN;
        return 1U;
    }

    if ((ImageCommand_MatchToken(frame, len, i, "green") != 0U) ||
        (ImageCommand_MatchToken(frame, len, i, "none") != 0U))
    {
        *command = IMAGE_COMMAND_NONE;
        return 1U;
    }

    return 0U;
}

static uint8_t ImageCommand_ErrorToCommand(int16_t error, ImageCommand_t *command)
{
    if (command == 0)
    {
        return 0U;
    }

    if (error == IMAGE_EVENT_TURN_RIGHT)
    {
        *command = IMAGE_COMMAND_TURN_RIGHT;
        return 1U;
    }

    if (error == IMAGE_EVENT_TURN_LEFT)
    {
        *command = IMAGE_COMMAND_TURN_LEFT;
        return 1U;
    }

    if (error == IMAGE_EVENT_PLATFORM)
    {
        *command = IMAGE_COMMAND_PLATFORM;
        return 1U;
    }

    if (error == IMAGE_EVENT_PURPLE)
    {
        *command = IMAGE_COMMAND_PURPLE;
        return 1U;
    }

    if (error == IMAGE_EVENT_BROWN)
    {
        *command = IMAGE_COMMAND_BROWN;
        return 1U;
    }

    if (error == IMAGE_EVENT_LOST)
    {
        *command = IMAGE_COMMAND_STOP;
        return 1U;
    }

    return 0U;
}

static void ImageCommand_FinishFrame(void)
{
    int16_t error;
    ImageCommand_t command = IMAGE_COMMAND_NONE;
    uint8_t has_error;
    uint8_t has_color;

    if (s_frame_len == 0U) // 如果没有接收到任何数据，则直接返回。
    {
        return;
    }

    has_error = ImageCommand_ParseErrorField(s_frame, s_frame_len, &error);
    has_color = ImageCommand_ParseColorCommand(s_frame, s_frame_len, &command);

    if ((has_color != 0U) && (command != IMAGE_COMMAND_NONE))
    {
        s_latest_command = command;
        s_has_command = 1U;
        s_latest_track.valid = 0U;
        s_has_track = 0U;
    }
    else if (has_error != 0U) // 如果解析到误差字段，则根据误差值判断是循迹误差还是事件命令。
    {
        if (ImageCommand_ErrorToCommand(error, &command) != 0U) // 如果是事件命令，则记录最新命令并清除循迹误差标志。
        {
            s_latest_command = command;
            s_has_command = 1U;
            s_latest_track.valid = 0U;
            s_has_track = 0U;
        }
        else
        {
            s_latest_track.error = error;
            s_latest_track.valid = 1U;
            s_has_track = 1U;
        }
    }

    s_frame_len = 0U;
}

static void ImageCommand_ProcessByte(uint8_t data)
{
    if ((data == (uint8_t)'\n') || (data == (uint8_t)'\r'))
    {
        ImageCommand_FinishFrame();
        return;
    }

    if (s_frame_len < (IMAGE_FRAME_MAX_LEN - 1U))
    {
        s_frame[s_frame_len] = (char)data;
        s_frame_len++;
    }
    else
    {
        s_frame_len = 0U;
    }
}

uint8_t ImageCommand_DecodeByte(uint8_t data, ImageCommand_t *command)
{
    if (command == 0)
    {
        return 0U;
    }

    if ((data == 'T') || (data == 't') ||
        (data == 'F') || (data == 'f'))
    {
        *command = IMAGE_COMMAND_FORWARD;
        return 1U;
    }

    if ((data == 'L') || (data == 'l'))
    {
        *command = IMAGE_COMMAND_TURN_LEFT;
        return 1U;
    }

    if ((data == 'R') || (data == 'r') ||
        (data == 'C') || (data == 'c'))
    {
        *command = IMAGE_COMMAND_TURN_RIGHT;
        return 1U;
    }

    if ((data == 'S') || (data == 's'))
    {
        *command = IMAGE_COMMAND_STOP;
        return 1U;
    }

    if ((data == 'U') || (data == 'u'))
    {
        *command = IMAGE_COMMAND_PLATFORM;
        return 1U;
    }

    if ((data == 'P') || (data == 'p'))
    {
        *command = IMAGE_COMMAND_PURPLE;
        return 1U;
    }

    if ((data == 'B') || (data == 'b'))
    {
        *command = IMAGE_COMMAND_BROWN;
        return 1U;
    }

    return 0U;
}

void ImageCommand_Init(void)
{
    s_latest_command = IMAGE_COMMAND_NONE;
    s_has_command = 0U;
    s_latest_track.error = 0;
    s_latest_track.valid = 0U;
    s_has_track = 0U;
    s_frame_len = 0U;
    HAL_UART_Receive_IT(&IMAGE_COMMAND_UART, &s_rx_data, 1U);
}

ImageCommand_t ImageCommand_TakeLatest(void)
{
    ImageCommand_t command = IMAGE_COMMAND_NONE;

    __disable_irq();
    if (s_has_command != 0U)
    {
        command = s_latest_command;
        s_has_command = 0U;
    }
    __enable_irq();

    return command;
}

ImageTrack_t ImageCommand_TakeLatestTrack(void)
{
    ImageTrack_t track = {0, 0U};

    __disable_irq();
    if (s_has_track != 0U)
    {
        track = s_latest_track;
        s_has_track = 0U;
    }
    __enable_irq();

    return track;
}

/* 接收 K230 字节*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        ImageCommand_ProcessByte(s_rx_data);
        HAL_UART_Receive_IT(&IMAGE_COMMAND_UART, &s_rx_data, 1U);
    }
}
