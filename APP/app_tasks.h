#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "cmsis_os2.h"
#include "bsp_uart.h"


/**
 滞回控制
 <2900 关闭警报
 2900-3000 保持
 >3000 开启警报
 */
#define ADC_ALARM_ON_THRESHOLD  3000U
#define ADC_ALARM_OFF_THRESHOLD 2900U
#define PWM_MAX_DUTY            999U

/* 传感器数据结构 */
typedef struct
{
    uint16_t adc_value;     // ADC原始值
    uint32_t timestamp;     // 采样时间
} SensorData_t;

/* 系统工作模式 
    自动模式：根据电位器数值控制输出LED灯亮灭
    手动模式：PWM固定为50，保留报警功能
    关闭模式：PWM强制为0，保留报警功能
*/
typedef enum
{
    MODE_AUTO = 0,
    MODE_MANUAL_ON,
    MODE_MANUAL_OFF

} ControlMode_t;

/* 消息队列句柄 */
extern osMessageQueueId_t SensorQueueHandle;
extern osMessageQueueId_t CommandQueueHandle;

extern volatile uint32_t sensor_count;
extern volatile uint32_t control_count;
extern volatile uint32_t uart_count;
extern volatile uint32_t key_count;
extern volatile uint32_t monitor_count;

extern volatile uint16_t latest_adc_value;

// 当前工作模式
extern volatile ControlMode_t system_mode;
// 当前PWM比较值
extern volatile uint16_t latest_pwm_duty;
// 当前报警状态
extern volatile uint8_t alarm_state;

/* main.c 中定义的系统复位信息 */
extern volatile uint32_t g_reset_flags_snapshot;
extern volatile uint8_t g_iwdg_reset_detected;

void App_SensorTask(void *argument);
void App_ControlTask(void *argument);
void App_UartTask(void *argument);
void App_KeyTask(void *argument);
void App_MonitorTask(void *argument);

/* 备份寄存器故障记录接口 */
void App_Backup_SaveTaskFault(uint32_t fault_mask);

uint8_t App_Backup_ReadTaskFault(uint32_t *fault_mask);

void App_Backup_ClearTaskFault(void);

#endif