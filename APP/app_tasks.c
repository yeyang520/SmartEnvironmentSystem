#include "app_tasks.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../BSP/bsp_pwm.h"

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

// uart1
extern UART_HandleTypeDef huart1;

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

        osDelay(100);
    }
}

/* Control Task */
void App_ControlTask(void *argument)
{
    SensorData_t received_data;

    uint16_t adc;
    uint16_t pwm_duty;

    for (;;)
    {
        osStatus_t status = osMessageQueueGet(
            SensorQueueHandle,
            &received_data,
            NULL,
            osWaitForever
        );

        if (status == osOK)
        {
            adc = received_data.adc_value;

            latest_adc_value = adc;

            control_count++;

            // 根据当前工作模式选择PWM
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

            // 更新PWM
            BSP_PWM_SetDuty(pwm_duty);

            latest_pwm_duty = pwm_duty;

            // 报警功能独立于工作模式
            if (alarm_state == 0)
            {
                if (adc >= 3000)
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
                if (adc <= 2900)
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
    }
}

/* UART Task */
void App_UartTask(void *argument)
{
    for (;;)
    {
        uart_count++;

        osDelay(1000);
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

        osDelay(10);
    }
}

/* Monitor Task */
void App_MonitorTask(void *argument)
{
    char buffer[120];

    const char *mode_name;

    for (;;)
    {
        monitor_count++;

        switch (system_mode)
        {
            case MODE_AUTO:
                mode_name = "AUTO";
                break;

            case MODE_MANUAL_ON:
                mode_name = "MANUAL_ON";
                break;

            case MODE_MANUAL_OFF:
                mode_name = "MANUAL_OFF";
                break;

            default:
                mode_name = "UNKNOWN";
                break;
        }

        int len = snprintf(
            buffer,
            sizeof(buffer),
            "ADC:%u | PWM:%u | ALARM:%u | MODE:%s\r\n",
            (unsigned int)latest_adc_value,
            (unsigned int)latest_pwm_duty,
            (unsigned int)alarm_state,
            mode_name
        );

        if (len > 0 && len < sizeof(buffer))
        {
            HAL_UART_Transmit(
                &huart1,
                (uint8_t *)buffer,
                (uint16_t)len,
                500
            );
        }

        osDelay(1000);
    }
}