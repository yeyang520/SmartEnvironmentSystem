#include "app_tasks.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>

#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "app_protocol.h"

#include "FreeRTOS.h"
#include "task.h"

/* 任务运行计数器 */
volatile uint32_t sensor_count = 0;
volatile uint32_t control_count = 0;
volatile uint32_t uart_count = 0;
volatile uint32_t key_count = 0;
volatile uint32_t monitor_count = 0;

volatile uint16_t latest_adc_value = 0;
volatile uint16_t latest_pwm_duty = 0;

volatile uint8_t alarm_state = 0;

volatile ControlMode_t system_mode = MODE_AUTO;

/* 完整命令接收计数 */
volatile uint32_t uart_command_count = 0;

/* 超长命令计数 */
volatile uint32_t uart_command_error_count = 0;

// uart1
extern UART_HandleTypeDef huart1;

/* 外部任务句柄，定义在 main.c */
extern osThreadId_t SensorTaskHandle;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t UartTaskHandle;
extern osThreadId_t KeyTaskHandle;
extern osThreadId_t MonitorTaskHandle;

volatile AppCommandType_t latest_command_type = CMD_NONE;
volatile uint32_t latest_command_value = 0;

volatile uint16_t alarm_on_threshold = 3000;
volatile uint16_t alarm_off_threshold = 2900;

/* 保存发生栈溢出的任务信息 */
TaskHandle_t volatile g_overflow_task = NULL;

char *volatile g_overflow_task_name=NULL;

/* 动态内存分配失败标志 */
volatile BaseType_t g_malloc_failed = pdFALSE;

/* ================= 任务心跳计数器 ================= */

/* SensorTask 心跳 */
volatile uint32_t g_sensor_heartbeat = 0;

/* ControlTask 心跳 */
volatile uint32_t g_control_heartbeat = 0;

/* UartTask 心跳 */
volatile uint32_t g_uart_heartbeat = 0;

/* KeyTask 心跳 */
volatile uint32_t g_key_heartbeat = 0;

/* ================= 任务健康状态 ================= */

/*
 * 故障标志位：
 * bit0：SensorTask 故障
 * bit1：ControlTask 故障
 * bit2：UartTask 故障
 * bit3：KeyTask 故障
 *
 * 0：所有任务正常
 */
volatile uint32_t g_task_health_fault_mask = 0;

/* 备份寄存器有效标志 */
#define APP_BACKUP_MAGIC    0xA55AU

volatile uint32_t g_iwdg_feed_count = 0;

/* 获取任务历史最小剩余栈空间，单位：Bytes */
static uint32_t App_GetStackMinFreeBytes(osThreadId_t task_handle)
{
    if (task_handle == NULL)
    {
        return 0;
    }

    UBaseType_t stack_words =
        uxTaskGetStackHighWaterMark((TaskHandle_t)task_handle);

    return (uint32_t)stack_words * sizeof(StackType_t);
}

/* 开启备份寄存器访问 */
static void App_Backup_EnableAccess(void)
{
    /* 开启 PWR 时钟 */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* 开启 BKP 时钟 */
    __HAL_RCC_BKP_CLK_ENABLE();

    /* 允许访问备份域 */
    PWR->CR |= PWR_CR_DBP;

    /* 读取回写，确保访问使能生效 */
    (void)PWR->CR;
}

/* 保存任务故障记录 */
/* 保存任务故障记录 */
void App_Backup_SaveTaskFault(uint32_t fault_mask)
{
    uint16_t mask;

    /* 只保留低4位 */
    mask = (uint16_t)(fault_mask & 0x000FU);

    App_Backup_EnableAccess();

    /* 先清除有效标志 */
    BKP->DR1 = 0;

    /* 写入故障掩码 */
    BKP->DR2 = mask;

    /* 写入反码校验 */
    BKP->DR3 = (uint16_t)(mask ^ 0xFFFFU);

    /* 最后写入有效标志 */
    BKP->DR1 = APP_BACKUP_MAGIC;

    /* 关闭备份域写访问 */
    PWR->CR &= ~PWR_CR_DBP;
}

/* 读取任务故障记录 */
uint8_t App_Backup_ReadTaskFault(uint32_t *fault_mask)
{
    uint16_t magic;
    uint16_t mask;
    uint16_t check;

    if (fault_mask == NULL)
    {
        return 0U;
    }

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();

    magic = BKP->DR1;
    mask = BKP->DR2;
    check = BKP->DR3;

    /* 检查有效标志和数据完整性 */
    if (magic != APP_BACKUP_MAGIC ||
        mask > 0x000FU ||
        check != (uint16_t)(mask ^ 0xFFFFU))
    {
        return 0U;
    }

    *fault_mask = (uint32_t)mask;

    return 1U;
}


/* 清除任务故障记录 */
void App_Backup_ClearTaskFault(void)
{
    App_Backup_EnableAccess();

    BKP->DR1 = 0;
    BKP->DR2 = 0;
    BKP->DR3 = 0;

    PWR->CR &= ~PWR_CR_DBP;
}

/* ================= 系统启动复位报告 ================= */
/**
看门狗复位
软件主动复位
外部复位引脚修复
上电复位
*/
static void App_UartReportResetReason(void)
{
    uint32_t flags = g_reset_flags_snapshot;
    uint8_t found = 0;

    static const char header[] =
        "\r\n========== SYSTEM BOOT ==========\r\n";

    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)header,
        sizeof(header) - 1,
        500
    );

    /* 独立看门狗复位 */
    if (flags & RCC_CSR_IWDGRSTF)
    {
        static const char msg[] = "RESET REASON: IWDG\r\n";

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            sizeof(msg) - 1,
            500
        );

        found = 1;
    }

    /* 窗口看门狗复位 */
    if (flags & RCC_CSR_WWDGRSTF)
    {
        static const char msg[] = "RESET REASON: WWDG\r\n";

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            sizeof(msg) - 1,
            500
        );

        found = 1;
    }

    /* 软件复位 */
    if (flags & RCC_CSR_SFTRSTF)
    {
        static const char msg[] = "RESET REASON: SOFTWARE\r\n";

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            sizeof(msg) - 1,
            500
        );

        found = 1;
    }

    /* 外部复位 */
    if (flags & RCC_CSR_PINRSTF)
    {
        static const char msg[] = "RESET FLAG: PIN RESET\r\n";

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            sizeof(msg) - 1,
            500
        );

        found = 1;
    }

    /* 上电或掉电复位 */
    if (flags & RCC_CSR_PORRSTF)
    {
        static const char msg[] = "RESET FLAG: POWER RESET\r\n";

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            sizeof(msg) - 1,
            500
        );

        found = 1;
    }

    /* 低功耗复位 */
    if (flags & RCC_CSR_LPWRRSTF)
    {
        static const char msg[] = "RESET REASON: LOW POWER\r\n";

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            sizeof(msg) - 1,
            500
        );

        found = 1;
    }

    if (found == 0)
    {
        static const char msg[] =
            "RESET REASON: UNKNOWN\r\n";

        HAL_UART_Transmit(
            &huart1,
            (uint8_t *)msg,
            sizeof(msg) - 1,
            500
        );
    }

    static const char footer[] =
        "=================================\r\n";

    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)footer,
        sizeof(footer) - 1,
        500
    );
}

/* Sensor Task */
void App_SensorTask(void *argument)
{
    SensorData_t sensor_data;

    for (;;)
    {
        // 启动ADC转换
        HAL_ADC_Start(&hadc1);

        // 等待ADC转换完成
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            // 获取ADC转换结果
            sensor_data.adc_value =
                HAL_ADC_GetValue(&hadc1);

            // 记录系统时间
            sensor_data.timestamp =
                osKernelGetTickCount();

            // 将数据发送到消息队列
            osMessageQueuePut(
                SensorQueueHandle,
                &sensor_data,
                0,
                0
            );

            sensor_count++;
        }

        // 停止ADC
        HAL_ADC_Stop(&hadc1);

        // SensorTask心跳
        g_sensor_heartbeat++;

        osDelay(100);
    }
}

/* Control Task */
void App_ControlTask(void *argument)
{
    SensorData_t received_data;
    AppCommand_t received_command;

    uint16_t adc;
    uint16_t pwm_duty;

    for (;;)
    {
        /* =================================
           1. 检查串口命令队列
           ================================= */

        while (osMessageQueueGet(
                   CommandQueueHandle,
                   &received_command,
                   NULL,
                   10
               ) == osOK)
        {
            switch (received_command.type)
            {
                case CMD_SET_MODE:

                    if (received_command.value <= MODE_MANUAL_OFF)
                    {
                        system_mode =
                            (ControlMode_t)received_command.value;
                    }

                    break;

                case CMD_SET_ALARM:

                    if (received_command.value >= 100 &&
                        received_command.value <= 4095)
                    {
                        alarm_on_threshold =
                            (uint16_t)received_command.value;

                        alarm_off_threshold =
                            (uint16_t)(received_command.value - 100);
                    }

                    break;

                default:
                    break;

                // 完成一条命令处理,心跳
                g_control_heartbeat++;
            }
        }

        /* =================================
           2. 等待传感器数据
           ================================= */

        osStatus_t status = osMessageQueueGet(
            SensorQueueHandle,
            &received_data,
            NULL,
            10
        );

        if (status == osOK)
        {
            adc = received_data.adc_value;

            latest_adc_value = adc;

            control_count++;

            /* 根据模式选择PWM */
            switch (system_mode)
            {
                case MODE_AUTO:

                    pwm_duty =
                        (uint32_t)adc * 999 / 4095;

                    break;

                case MODE_MANUAL_ON:

                    pwm_duty = 500;

                    break;

                case MODE_MANUAL_OFF:

                    pwm_duty = 0;

                    break;

                default:

                    pwm_duty = 0;

                    break;
            }

            /* 更新PWM */
            BSP_PWM_SetDuty(pwm_duty);

            latest_pwm_duty = pwm_duty;

            /* 滞回报警 */
            if (alarm_state == 0)
            {
                if (adc >= alarm_on_threshold)
                {
                    alarm_state = 1;

                    HAL_GPIO_WritePin(
                        LED_ALARM_GPIO_Port,
                        LED_ALARM_Pin,
                        GPIO_PIN_SET
                    );
                }
            }
            else
            {
                if (adc <= alarm_off_threshold)
                {
                    alarm_state = 0;

                    HAL_GPIO_WritePin(
                        LED_ALARM_GPIO_Port,
                        LED_ALARM_Pin,
                        GPIO_PIN_RESET
                    );
                }
            }
        }
        // 完成一轮控制任务循环
        g_control_heartbeat++;
    }
}

/* UART Task */
void App_UartTask(void *argument)
{
    uint8_t rx_data[32];

    char command[APP_COMMAND_MAX_LEN];

    AppCommand_t parsed_command;

    /* 启动DMA */
    HAL_StatusTypeDef status = BSP_UART_StartReceive();

    if (status != HAL_OK)
    {
        for (;;)
        {
            osDelay(1000);
        }
    }

    AppProtocol_Init();

    /* 输出系统上一次复位原因 */
    App_UartReportResetReason();

    for (;;)
    {
        /* DMA数据提取 */
        BSP_UART_PollReceive();

        /* 读取软件环形缓冲区 */
        uint16_t len = BSP_UART_Read(
            rx_data,
            sizeof(rx_data)
        );

        if (len > 0)
        {
            uart_count += len;
            for (uint16_t i = 0; i < len; i++)
            {
                AppProtocolResult_t state;

                state = AppProtocol_FeedByte(
                    rx_data[i],
                    command,
                    sizeof(command)
                );

                /* 收到完整命令 */
                if (state == APP_PROTOCOL_READY)
                {
                    if (AppProtocol_ParseCommand(command, &parsed_command))
                    {
                        if (parsed_command.type == CMD_SET_MODE ||
                            parsed_command.type == CMD_SET_ALARM)
                        {
                            /* 报警开启阈值至少为100 */
                            if (parsed_command.type == CMD_SET_ALARM &&
                                parsed_command.value < 100)
                            {
                                const char reply[] =
                                    "ERR: ALARM RANGE 100-4095\r\n";

                                HAL_UART_Transmit(
                                    &huart1,
                                    (uint8_t *)reply,
                                    sizeof(reply) - 1,
                                    500
                                );
                            }
                            else
                            {
                                /* 将命令放入消息队列 */
                                osStatus_t queue_status = osMessageQueuePut(
                                    CommandQueueHandle,
                                    &parsed_command,
                                    0,
                                    0
                                );

                                if (queue_status == osOK)
                                {
                                    uart_command_count++;

                                    const char reply[] =
                                        "OK: COMMAND QUEUED\r\n";

                                    HAL_UART_Transmit(
                                        &huart1,
                                        (uint8_t *)reply,
                                        sizeof(reply) - 1,
                                        500
                                    );
                                }
                                else
                                {
                                    const char reply[] =
                                        "ERR: COMMAND QUEUE FULL\r\n";

                                    HAL_UART_Transmit(
                                        &huart1,
                                        (uint8_t *)reply,
                                        sizeof(reply) - 1,
                                        500
                                    );
                                }
                            }
                        }
                        else if (parsed_command.type == CMD_STATUS)
                        {
                            /* 获取五个任务的历史最小剩余栈空间 */
                            uint32_t sensor_stack =
                                App_GetStackMinFreeBytes(SensorTaskHandle);

                            uint32_t control_stack =
                                App_GetStackMinFreeBytes(ControlTaskHandle);

                            uint32_t uart_stack =
                                App_GetStackMinFreeBytes(UartTaskHandle);

                            uint32_t key_stack =
                                App_GetStackMinFreeBytes(KeyTaskHandle);

                            uint32_t monitor_stack =
                                App_GetStackMinFreeBytes(MonitorTaskHandle);

                            /* 获取当前剩余 Heap */
                            size_t heap_free_bytes = xPortGetFreeHeapSize();

                            /* 获取历史最小剩余 Heap */
                            size_t heap_min_free_bytes =
                                xPortGetMinimumEverFreeHeapSize();

                            static char reply[256];

                            int reply_len = snprintf(
                                reply,
                                sizeof(reply),

                                "ADC:%u PWM:%u MODE:%d ALARM:%u\r\n"
                                "STACK_MIN_FREE(B): S=%lu C=%lu U=%lu K=%lu M=%lu\r\n"
                                "HEAP_FREE(B): %lu HEAP_MIN_FREE(B): %lu\r\n",

                                (unsigned int)latest_adc_value,
                                (unsigned int)latest_pwm_duty,
                                (int)system_mode,
                                (unsigned int)alarm_state,

                                (unsigned long)sensor_stack,
                                (unsigned long)control_stack,
                                (unsigned long)uart_stack,
                                (unsigned long)key_stack,
                                (unsigned long)monitor_stack,
                                (unsigned long)heap_free_bytes,
                                (unsigned long)heap_min_free_bytes
                            );

                            if (reply_len > 0 && reply_len < (int)sizeof(reply))
                            {
                                HAL_UART_Transmit(
                                    &huart1,
                                    (uint8_t *)reply,
                                    (uint16_t)reply_len,
                                    500
                                );
                            }
                        }
                        else if (parsed_command.type == CMD_HELP)
                        {
                            const char reply[] =
                                "MODE AUTO\r\n"
                                "MODE MANUAL_ON\r\n"
                                "MODE MANUAL_OFF\r\n"
                                "SET ALARM 3200\r\n"
                                "STATUS\r\n"
                                "HELP\r\n";

                            HAL_UART_Transmit(
                                &huart1,
                                (uint8_t *)reply,
                                sizeof(reply) - 1,
                                500
                            );
                        }
                    }
                    else
                    {
                        uart_command_error_count++;

                        const char reply[] = "UNKNOWN COMMAND\r\n";

                        HAL_UART_Transmit(
                            &huart1,
                            (uint8_t *)reply,
                            sizeof(reply) - 1,
                            500
                        );
                    }
                }
                else if (state == APP_PROTOCOL_TOO_LONG)
                {
                    uart_command_error_count++;
                }
            }
        }
        // uart心跳
        g_uart_heartbeat++;
        osDelay(5);
    }
}

/* Key Task */
void App_KeyTask(void *argument)
{
    GPIO_PinState last_sample;
    GPIO_PinState stable_state;

    uint8_t debounce_count = 0;

    // 初始化按键状态
    last_sample = HAL_GPIO_ReadPin(
        KEY_GPIO_Port,
        KEY_Pin
    );

    stable_state = last_sample;

    for (;;)
    {
        GPIO_PinState current_sample =
            HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);

        // 检测输入状态是否稳定
        if (current_sample == last_sample)
        {
            if (debounce_count < 3)
            {
                debounce_count++;
            }
        }
        else
        {
            last_sample = current_sample;
            debounce_count = 0;
        }

        // 连续采样稳定后，确认按键状态改变
        if (debounce_count >= 3 &&
            current_sample != stable_state)
        {
            stable_state = current_sample;

            // 只在按下时切换模式
            if (stable_state == GPIO_PIN_RESET)
            {
                system_mode++;

                if (system_mode > MODE_MANUAL_OFF)
                {
                    system_mode = MODE_AUTO;
                }

                key_count++;
            }
        }
        // KeyTask心跳
        g_key_heartbeat++;
        osDelay(10);
    }
}

/* Monitor Task */
void App_MonitorTask(void *argument)
{
    /* 保存上一次的心跳值 */
    static uint32_t last_sensor_heartbeat = 0;
    static uint32_t last_control_heartbeat = 0;
    static uint32_t last_uart_heartbeat = 0;
    static uint32_t last_key_heartbeat = 0;

    /* 首次检测标志 */
    static uint8_t first_check = 1;

    /* 当前心跳值 */
    uint32_t current_sensor;
    uint32_t current_control;
    uint32_t current_uart;
    uint32_t current_key;

    for (;;)
    {
        monitor_count++;

        /* 第一次运行：记录初始心跳 */
        if (first_check)
        {
            last_sensor_heartbeat = g_sensor_heartbeat;
            last_control_heartbeat = g_control_heartbeat;
            last_uart_heartbeat = g_uart_heartbeat;
            last_key_heartbeat = g_key_heartbeat;

            first_check = 0;

            /* 建立初始基准后允许第一次喂狗 */
            App_IWDG_Refresh();
        }
        /* 只有尚未发生故障时才继续检查 */
        else if (g_task_health_fault_mask == 0)
        {
            /* 读取当前心跳 */
            current_sensor = g_sensor_heartbeat;
            current_control = g_control_heartbeat;
            current_uart = g_uart_heartbeat;
            current_key = g_key_heartbeat;

            /* 检查 SensorTask */
            if (current_sensor == last_sensor_heartbeat)
            {
                g_task_health_fault_mask |= (1U << 0);
            }

            /* 检查 ControlTask */
            if (current_control == last_control_heartbeat)
            {
                g_task_health_fault_mask |= (1U << 1);
            }

            /* 检查 UartTask */
            if (current_uart == last_uart_heartbeat)
            {
                g_task_health_fault_mask |= (1U << 2);
            }

            /* 检查 KeyTask */
            if (current_key == last_key_heartbeat)
            {
                g_task_health_fault_mask |= (1U << 3);
            }

            /* 保存本次心跳，供下一轮比较 */
            last_sensor_heartbeat = current_sensor;
            last_control_heartbeat = current_control;
            last_uart_heartbeat = current_uart;
            last_key_heartbeat = current_key;

            /* 所有任务正常才允许喂狗 */
            if (g_task_health_fault_mask == 0)
            {
                g_iwdg_feed_count++;
                App_IWDG_Refresh();
            }
            else
            {
                /* 保存故障任务信息 */
                App_Backup_SaveTaskFault(g_task_health_fault_mask);
                /* 不再喂狗，等待 IWDG 自动复位 */
            }
        }
        osDelay(1000);
    }
}

/* FreeRTOS 栈溢出钩子 */
void vApplicationStackOverflowHook(
    TaskHandle_t xTask,
    char *pcTaskName)
{
    /* 保存异常任务信息 */
    g_overflow_task = xTask;
    g_overflow_task_name = pcTaskName;

    /* 关闭中断 */
    __disable_irq();

    /* 停止系统，等待调试 */
    for (;;)
    {
        __NOP();
    }
}

/* FreeRTOS 动态内存分配失败钩子 */
void vApplicationMallocFailedHook(void)
{
    /* 记录内存分配失败 */
    g_malloc_failed = pdTRUE;

    /* 停止系统，等待调试器检查 */
    for (;;)
    {
        __NOP();
    }
}