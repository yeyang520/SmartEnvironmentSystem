
#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>

#define APP_COMMAND_MAX_LEN 64

/* 字符拼接结果 */
typedef enum
{
    APP_PROTOCOL_NONE = 0,
    APP_PROTOCOL_READY,
    APP_PROTOCOL_TOO_LONG

} AppProtocolResult_t;

/* 命令类型 */
typedef enum
{
    CMD_NONE = 0,
    CMD_SET_MODE,
    CMD_SET_ALARM,
    CMD_STATUS,
    CMD_HELP

} AppCommandType_t;

/* 命令结构体 */
typedef struct
{
    AppCommandType_t type;
    uint32_t value;

} AppCommand_t;

/* 初始化解析器 */
void AppProtocol_Init(void);

/* 字符拼接 */
AppProtocolResult_t AppProtocol_FeedByte(
    uint8_t data,
    char *command,
    uint16_t command_size
);

/* 命令解析 */
uint8_t AppProtocol_ParseCommand(
    const char *command,
    AppCommand_t *result
);

#endif
