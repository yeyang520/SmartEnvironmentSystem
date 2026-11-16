
#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"
#include <stdint.h>

#define UART_DMA_RX_SIZE 128
#define UART_RING_BUFFER_SIZE 256

/* DMA句柄 */
extern DMA_HandleTypeDef hdma_usart1_rx;

/* USART1句柄在main.c中定义 */
extern UART_HandleTypeDef huart1;

/* DMA启动状态 */
extern volatile uint32_t uart_dma_start_status;

/* 接收统计 */
extern volatile uint32_t uart_rx_count;
extern volatile uint32_t uart_rx_overflow_count;
extern volatile uint8_t uart_last_rx_byte;

/* 启动DMA接收 */
HAL_StatusTypeDef BSP_UART_StartReceive(void);

/* 检查DMA新数据 */
uint16_t BSP_UART_PollReceive(void);

/* 从环形缓冲区读取数据 */
uint16_t BSP_UART_Read(uint8_t *data, uint16_t max_len);

/* 获取 DMA 当前写入位置 */
uint16_t BSP_UART_GetRxPosition(void);

/* 获取 DMA 剩余容量 */
uint16_t BSP_UART_GetRxRemaining(void);

#endif
