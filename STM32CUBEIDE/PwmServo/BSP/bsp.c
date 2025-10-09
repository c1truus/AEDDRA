/**
 * @file bsp.c
 * @brief Board Support Package (BSP) implementation for STM32F1 robot control.
 */

#include "bsp.h"

/**
 * @brief Toggles LED to show status, called every 10ms, blinks every 200ms.
 */
void Bsp_Led_Show_State_Handle(void)
{
    static uint8_t led_count = 0;
    led_count++;
    if (led_count > 20)
    {
        led_count = 0;
        LED_TOGGLE();
    }
}

/**
 * @brief Initialize peripherals (servos, motors, I2C slave, buzzer).
 */
void Bsp_Init(void)
{
    I2C_Slave_Init();
    PwmServo_Init();
    Motor_Init();
    Encoder_Init();
    PID_Param_Init();
    USART1_Init();
    Beep_On_Time(50);
}

/**
 * @brief Main loop function, called repeatedly in main.c.
 * Handles button events, I2C commands, LED, and buzzer timeout.
 */
void Bsp_Loop(void)
{
	Motion_Handle();
    static uint8_t key_state = 0;
    static uint8_t empty_buffer_count = 0;  // Counter for consecutive empty buffer occurrences
    const uint8_t EMPTY_BUFFER_THRESHOLD = 16;  // Threshold for connection loss (10 consecutive empties ~100ms)

    // Detect button down events
    if (Key1_State(KEY_MODE_ONE_TIME))
    {
        Beep_On_Time(50);
        if (key_state)
        {
            key_state = 0;
            PwmServo_Set_Angle_All(50, 50, 50, 50);
            Motion_Ctrl(500,0,0);
            HAL_Delay(500);
            Motion_Ctrl(0,500,0);
            HAL_Delay(500);
            Motion_Ctrl(0,-500,0);
            HAL_Delay(500);
            Motion_Ctrl(-500,	0,0);
            printf("key state: %d\n", key_state );
        }
        else
        {
            key_state = 1;
            PwmServo_Set_Angle_All(90, 90, 90, 90);

            printf("key state: %d\n", key_state );
        }
    }

    // Process I2C commands
    Command_t cmd;
    if (I2C_Get_Next_Command(&cmd))
    {
        empty_buffer_count = 0;  // Reset counter on successful command
        if (cmd.kill)
        {
            printf("KILL SWITCH ACTIVATED - Stopping all motors and servos\n");
            Motion_Stop(STOP_BRAKE);
            PwmServo_Set_Angle_All(90, 90, 90, 90);  // Safe values
        }
        else
        {
            // TODO: If all motor values are 0, stop immediately (brake/coast all motors)
            if (cmd.m1 == 0 && cmd.m2 == 0 && cmd.m3 == 0 && cmd.m4 == 0) {
            	Motion_Stop(STOP_BRAKE);
                printf("All motors stopped (zero PWM command)\n");
            } else {
            	printf("Got command:\n");
            	Motion_Set_Speed(cmd.m1, cmd.m2, cmd.m3, cmd.m4);
                PwmServo_Set_Angle(1, cmd.s1);
                PwmServo_Set_Angle(2, cmd.s2);
                PwmServo_Set_Angle(3, cmd.s3);
                PwmServo_Set_Angle(4, cmd.s4);
            }
        }
    }
    else
    {
        empty_buffer_count++;  // Increment counter on empty buffer
        if (empty_buffer_count >= EMPTY_BUFFER_THRESHOLD)
        {
            printf("I2C connection lost (ring buffer empty for %d cycles) - Stopping all motors and resetting\n", EMPTY_BUFFER_THRESHOLD);
            Motion_Stop(STOP_BRAKE);  // Stop motors with brake
            PID_Clear_Motor(MAX_MOTOR);  // Reset PID states for all motors
            empty_buffer_count = 0;  // Reset counter after handling
            PwmServo_Set_Angle_All(90, 90, 90, 90);
//            I2C_Slave_Init();
            HAL_I2C_EnableListen_IT(&hi2c2);
        }
    }

    Bsp_Led_Show_State_Handle();
    Beep_Timeout_Close_Handle();
    HAL_Delay(10);
}
