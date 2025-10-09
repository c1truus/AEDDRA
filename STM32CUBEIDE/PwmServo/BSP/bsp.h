/**
 * @file bsp.h
 * @brief Board Support Package (BSP) header for STM32F1 robot control.
 */

#ifndef __BSP_H__
#define __BSP_H__

/* HAL Library Includes */
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"

#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "stm32f103xe.h"


/* Device Driver Includes */
#include "bsp_beep.h"
#include "bsp_key.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_motion.h"
#include "bsp_pid.h"
#include "bsp_pwmServo.h"
#include "bsp_uart.h"

#include <stdio.h>
/* Definitions */
#define MOTOR_ID_M1          1
#define MOTOR_ID_M2          2
#define MOTOR_ID_M3          3
#define MOTOR_ID_M4          4
#define LED_TOGGLE()         HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
#define RX_BUFFER_SIZE       512
/* Data Structures */
typedef struct __attribute__((packed)) {
    bool kill;          // Kill switch for safe state
    int16_t m1;         // Motor 1 PWM (-2000 to 2000)
    int16_t m2;         // Motor 2 PWM
    int16_t m3;         // Motor 3 PWM
    int16_t m4;         // Motor 4 PWM
    uint8_t s1;         // Servo 1 angle (0-180, base)
    uint8_t s2;         // Servo 2 angle (shoulder)
    uint8_t s3;         // Servo 3 angle (elbow)
    uint8_t s4;         // Servo 4 angle (gripper)
} Command_t;

typedef struct {
    Command_t buffer[RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} RingBuffer;

/* Externs */
extern RingBuffer rxRingBuffer;

/* Function Prototypes */
void Bsp_Init(void);
void Bsp_Loop(void);
void RingBuffer_Init(RingBuffer *rb);
uint8_t RingBuffer_IsEmpty(RingBuffer *rb);
uint8_t RingBuffer_IsFull(RingBuffer *rb);
uint8_t RingBuffer_Push(RingBuffer *rb, Command_t *cmd);
uint8_t RingBuffer_Pop(RingBuffer *rb, Command_t *cmd);
uint8_t I2C_Get_Next_Command(Command_t *cmd);
void I2C_Slave_Init(void);

#endif /* __BSP_H__ */
