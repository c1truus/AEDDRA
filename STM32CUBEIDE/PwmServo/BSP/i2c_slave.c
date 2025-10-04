/**
 * @file i2c_slave.c
 * @brief I2C slave implementation for STM32F1.
 * @author turu
 * @date Sep 30, 2025
 */

#include "i2c_slave.h"
#include "bsp.h"
#include <string.h>
#include <stdio.h>

/* External I2C handle */
extern I2C_HandleTypeDef hi2c2;

/* Constants */
#define RX_SIZE 13
#define RX_BUFFER_SIZE 64

/* Static variables */
static uint8_t RxData[RX_SIZE];
static volatile uint8_t dataReady = 0;

/* External ring buffer */
RingBuffer rxRingBuffer;

/**
 * @brief Initialize I2C slave and ring buffer.
 */
void I2C_Slave_Init(void)
{
    RingBuffer_Init(&rxRingBuffer);
    if (HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK)
    {
//        Error_Handler();
        printf("It was not in listen mode\n");
        HAL_I2C_EnableListen_IT(&hi2c2);
    }
    printf("I2C Slave Initialized\n");
}

/**
 * @brief Check if new data is ready in the ring buffer.
 * @return 1 if data is ready, 0 otherwise.
 */
uint8_t I2C_Data_Ready(void)
{
    return dataReady;
}

/**
 * @brief Pop the next command from the ring buffer.
 * @param cmd Pointer to store the popped command.
 * @return 1 if a command was popped, 0 if the buffer is empty.
 */
uint8_t I2C_Get_Next_Command(Command_t *cmd)
{
    if (RingBuffer_Pop(&rxRingBuffer, cmd))
    {
        if (RingBuffer_IsEmpty(&rxRingBuffer))
        {
            dataReady = 0;  // No more data left
            printf("No data :( \n");
        }
        return 1;
    }
    else
    {
    	printf("Ring bf empty \n");
        dataReady = 0;  // Buffer empty
        return 0;
    }
}

/**
 * @brief I2C address callback to handle master addressing.
 * @param hi2c I2C handle.
 * @param dir Direction of transfer (I2C_DIRECTION_TRANSMIT or I2C_DIRECTION_RECEIVE).
 * @param addrMatchCode Address match code.
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addrMatchCode)
{
    if (dir == I2C_DIRECTION_TRANSMIT)
    {
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData, RX_SIZE, I2C_FIRST_AND_LAST_FRAME);
    }
    else
    {
        Error_Handler();  // Master read not supported
    }
}

/**
 * @brief I2C slave receive complete callback.
 * @param hi2c I2C handle.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    Command_t cmd;
    printf("Received raw data (%d bytes): ", RX_SIZE);
    for (int i = 0; i < RX_SIZE; i++) {
        printf("%02X ", RxData[i]);
    }
    printf("\n");
    memcpy(&cmd, RxData, sizeof(cmd));
    if (!RingBuffer_Push(&rxRingBuffer, &cmd))
    {
        printf("Ring buffer full, dropping command\n");
    }
    else
    {
        dataReady = 1;
    }
}

/**
 * @brief I2C listen complete callback.
 * @param hi2c I2C handle.
 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

/**
 * @brief I2C error callback.
 * @param hi2c I2C handle.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

/**
 * @brief Initialize the ring buffer.
 * @param rb Pointer to the ring buffer.
 */
void RingBuffer_Init(RingBuffer *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/**
 * @brief Check if the ring buffer is empty.
 * @param rb Pointer to the ring buffer.
 * @return 1 if empty, 0 otherwise.
 */
uint8_t RingBuffer_IsEmpty(RingBuffer *rb)
{
    return rb->count == 0;
}

/**
 * @brief Check if the ring buffer is full.
 * @param rb Pointer to the ring buffer.
 * @return 1 if full, 0 otherwise.
 */
uint8_t RingBuffer_IsFull(RingBuffer *rb)
{
    return rb->count == RX_BUFFER_SIZE;
}

/**
 * @brief Push a command to the ring buffer.
 * @param rb Pointer to the ring buffer.
 * @param cmd Pointer to the command to push.
 * @return 1 if successful, 0 if buffer is full.
 */
uint8_t RingBuffer_Push(RingBuffer *rb, Command_t *cmd)
{
    if (RingBuffer_IsFull(rb)) return 0;

    rb->buffer[rb->head] = *cmd;
    rb->head = (rb->head + 1) % RX_BUFFER_SIZE;
    rb->count++;
    return 1;
}

/**
 * @brief Pop a command from the ring buffer.
 * @param rb Pointer to the ring buffer.
 * @param cmd Pointer to store the popped command.
 * @return 1 if successful, 0 if buffer is empty.
 */
uint8_t RingBuffer_Pop(RingBuffer *rb, Command_t *cmd)
{
    if (RingBuffer_IsEmpty(rb)) return 0;

    *cmd = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RX_BUFFER_SIZE;
    rb->count--;
    return 1;
}
