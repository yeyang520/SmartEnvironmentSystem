
#include "app_protocol.h"
#include <stddef.h>
#include <string.h>

/* 内部命令接收缓冲区 */
static char rx_command[APP_COMMAND_MAX_LEN];

/* 当前接收长度 */
static uint16_t rx_index = 0;

/* 是否正在丢弃超长命令 */
static uint8_t discard_line = 0;

/* 是否需要忽略CR后面的LF */
static uint8_t skip_lf = 0;


/* 初始化命令解析器 */
void AppProtocol_Init(void)
{
    rx_index = 0;
    discard_line = 0;
    skip_lf = 0;

    for (uint16_t i = 0; i < APP_COMMAND_MAX_LEN; i++)
    {
        rx_command[i] = '\0';
    }
}


/* 逐字节接收并拼接完整命令 */
AppProtocolResult_t AppProtocol_FeedByte(
    uint8_t data,
    char *command,
    uint16_t command_size
)
{
    /* 处理CRLF，避免一条命令被识别两次 */
    if (skip_lf)
    {
        skip_lf = 0;

        if (data == '\n')
        {
            return APP_PROTOCOL_NONE;
        }
    }

    /* 检查命令结束符 */
    if (data == '\r' || data == '\n')
    {
        if (data == '\r')
        {
            skip_lf = 1;
        }

        /* 前面发生过命令超长 */
        if (discard_line)
        {
            discard_line = 0;
            rx_index = 0;

            return APP_PROTOCOL_TOO_LONG;
        }

        /* 忽略空命令 */
        if (rx_index == 0)
        {
            return APP_PROTOCOL_NONE;
        }

        /* 检查输出缓冲区 */
        if (command == NULL ||
            command_size <= rx_index)
        {
            rx_index = 0;

            return APP_PROTOCOL_TOO_LONG;
        }

        /* 添加字符串结束符 */
        rx_command[rx_index] = '\0';

        /* 将完整命令复制给调用者 */
        for (uint16_t i = 0; i <= rx_index; i++)
        {
            command[i] = rx_command[i];
        }

        /* 准备接收下一条命令 */
        rx_index = 0;

        return APP_PROTOCOL_READY;
    }

    /* 超长命令：丢弃剩余字符，直到换行 */
    if (discard_line)
    {
        return APP_PROTOCOL_NONE;
    }

    /* 支持退格键 */
    if (data == 0x08 || data == 0x7F)
    {
        if (rx_index > 0)
        {
            rx_index--;
        }

        return APP_PROTOCOL_NONE;
    }

    /* 只接收可显示的ASCII字符 */
    if (data < 0x20 || data > 0x7E)
    {
        return APP_PROTOCOL_NONE;
    }

    /* 检查内部缓冲区是否还有空间 */
    if (rx_index < APP_COMMAND_MAX_LEN - 1)
    {
        rx_command[rx_index++] = (char)data;
    }
    else
    {
        rx_index = 0;
        discard_line = 1;
    }

    return APP_PROTOCOL_NONE;
}

/* 解析十进制数字 */
static uint8_t ParseNumber(
    const char *str,
    uint32_t *value
)
{
    uint32_t result = 0;

    if (str == NULL || *str == '\0')
    {
        return 0;
    }

    while (*str != '\0')
    {
        if (*str < '0' || *str > '9')
        {
            return 0;
        }

        result = result * 10 + (*str - '0');

        /* 报警阈值最大为4095 */
        if (result > 4095)
        {
            return 0;
        }

        str++;
    }

    *value = result;

    return 1;
}


/* 命令解析函数 */
uint8_t AppProtocol_ParseCommand(
    const char *command,
    AppCommand_t *result
)
{
    if (command == NULL || result == NULL)
    {
        return 0;
    }

    result->type = CMD_NONE;
    result->value = 0;

    /* 自动模式 */
    if (strcmp(command, "MODE AUTO") == 0)
    {
        result->type = CMD_SET_MODE;
        result->value = 0;

        return 1;
    }

    /* 手动开启 */
    if (strcmp(command, "MODE MANUAL_ON") == 0)
    {
        result->type = CMD_SET_MODE;
        result->value = 1;

        return 1;
    }

    /* 手动关闭 */
    if (strcmp(command, "MODE MANUAL_OFF") == 0)
    {
        result->type = CMD_SET_MODE;
        result->value = 2;

        return 1;
    }

    /* 修改报警阈值 */
    if (strncmp(command, "SET ALARM ", 10) == 0)
    {
        uint32_t threshold;

        if (!ParseNumber(command + 10, &threshold))
        {
            return 0;
        }

        result->type = CMD_SET_ALARM;
        result->value = threshold;

        return 1;
    }

    /* 查询状态 */
    if (strcmp(command, "STATUS") == 0)
    {
        result->type = CMD_STATUS;
        return 1;
    }

    /* 帮助 */
    if (strcmp(command, "HELP") == 0)
    {
        result->type = CMD_HELP;
        return 1;
    }

    return 0;
}