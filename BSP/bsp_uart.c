
#include "bsp_uart.h"

/* USART1句柄在main.c中定义 */
extern UART_HandleTypeDef huart1;

/* DMA句柄：全局定义一次 */
DMA_HandleTypeDef hdma_usart1_rx;

/* DMA接收缓冲区 */
static uint8_t uart_dma_rx_buffer[UART_DMA_RX_SIZE];

/* 软件环形缓冲区 */
static uint8_t uart_ring_buffer[UART_RING_BUFFER_SIZE];

/* DMA读取位置 */
static uint16_t uart_dma_old_pos = 0;

/* 软件环形缓冲区读写位置 */
static volatile uint16_t ring_write_index = 0;
static volatile uint16_t ring_read_index = 0;

/* DMA启动状态 */
volatile uint32_t uart_dma_start_status = 999;

/* 接收统计 */
volatile uint32_t uart_rx_count = 0;
volatile uint32_t uart_rx_overflow_count = 0;
volatile uint8_t uart_last_rx_byte = 0;


/* =====================================================
   函数1：环形缓冲区写入一个字节
   ===================================================== */
static void UART_RingBuffer_Put(uint8_t data)
{
    uint16_t next_index;

    next_index = (ring_write_index + 1)
                 % UART_RING_BUFFER_SIZE;

    /* 判断环形缓冲区是否已满 */
    if (next_index == ring_read_index)
    {
        uart_rx_overflow_count++;
        return;
    }

    /* 写入数据 */
    uart_ring_buffer[ring_write_index] = data;

    /* 更新写入位置 */
    ring_write_index = next_index;

    uart_rx_count++;

    uart_last_rx_byte = data;
}


/* =====================================================
   函数2：启动DMA循环接收
   ===================================================== */
HAL_StatusTypeDef BSP_UART_StartReceive(void)
{
    uart_dma_old_pos = 0;

    ring_write_index = 0;
    ring_read_index = 0;

    uart_rx_count = 0;
    uart_rx_overflow_count = 0;
    uart_last_rx_byte = 0;

    /* 启动DMA */
    HAL_StatusTypeDef status =
        HAL_UART_Receive_DMA(
            &huart1,
            uart_dma_rx_buffer,
            UART_DMA_RX_SIZE
        );

    uart_dma_start_status = (uint32_t)status;

    return status;
}

/* 获取 DMA 剩余传输数量 */
uint16_t BSP_UART_GetRxRemaining(void)
{
    if (huart1.hdmarx == NULL)
    {
        return UART_DMA_RX_SIZE;
    }

    return __HAL_DMA_GET_COUNTER(huart1.hdmarx);
}


/* 获取 DMA 当前写入位置 */
uint16_t BSP_UART_GetRxPosition(void)
{
    uint16_t remaining;
    uint16_t position;

    remaining = BSP_UART_GetRxRemaining();

    if (remaining > UART_DMA_RX_SIZE)
    {
        return 0;
    }

    position = UART_DMA_RX_SIZE - remaining;

    if (position >= UART_DMA_RX_SIZE)
    {
        position = 0;
    }

    return position;
}

/* =====================================================
   函数3：读取DMA新数据
   ===================================================== */
uint16_t BSP_UART_PollReceive(void)
{
    uint16_t remaining;
    uint16_t dma_pos;
    uint16_t count = 0;

    /* 获取DMA剩余传输数量 */
    remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (remaining > UART_DMA_RX_SIZE)
    {
        return 0;
    }

    /* 计算DMA当前位置 */
    dma_pos = UART_DMA_RX_SIZE - remaining;

    if (dma_pos >= UART_DMA_RX_SIZE)
    {
        dma_pos = 0;
    }

    /* 没有新数据 */
    if (dma_pos == uart_dma_old_pos)
    {
        return 0;
    }

    /* 情况1：没有发生缓冲区回绕 */
    if (dma_pos > uart_dma_old_pos)
    {
        while (uart_dma_old_pos < dma_pos)
        {
            UART_RingBuffer_Put(
                uart_dma_rx_buffer[uart_dma_old_pos]
            );

            uart_dma_old_pos++;
            count++;
        }
    }
    /* 情况2：DMA发生缓冲区回绕 */
    else
    {
        /* 读取缓冲区末尾 */
        while (uart_dma_old_pos < UART_DMA_RX_SIZE)
        {
            UART_RingBuffer_Put(
                uart_dma_rx_buffer[uart_dma_old_pos]
            );

            uart_dma_old_pos++;
            count++;
        }

        /* 回到缓冲区开头 */
        uart_dma_old_pos = 0;

        /* 读取缓冲区开头 */
        while (uart_dma_old_pos < dma_pos)
        {
            UART_RingBuffer_Put(
                uart_dma_rx_buffer[uart_dma_old_pos]
            );

            uart_dma_old_pos++;
            count++;
        }
    }

    return count;
}


/* =====================================================
   函数4：从软件环形缓冲区读取数据
   ===================================================== */
uint16_t BSP_UART_Read(uint8_t *data, uint16_t max_len)
{
    uint16_t count = 0;

    if (data == NULL || max_len == 0)
    {
        return 0;
    }

    while ((ring_read_index != ring_write_index)
           && (count < max_len))
    {
        data[count] = uart_ring_buffer[ring_read_index];

        ring_read_index++;

        if (ring_read_index >= UART_RING_BUFFER_SIZE)
        {
            ring_read_index = 0;
        }

        count++;
    }

    return count;
}
