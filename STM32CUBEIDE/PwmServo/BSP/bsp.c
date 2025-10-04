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
    USART1_Init();
    Beep_On_Time(50);
}

/**
 * @brief Main loop function, called repeatedly in main.c.
 * Handles button events, I2C commands, LED, and buzzer timeout.
 */
void Bsp_Loop(void)
{
    static uint8_t key_state = 0;

    // Detect button down events
    if (Key1_State(KEY_MODE_ONE_TIME))
    {
        Beep_On_Time(50);
        if (key_state)
        {
            key_state = 0;
            PwmServo_Set_Angle_All(50, 50, 50, 50);
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
    	printf("Got command:\n");
        if (cmd.kill)
        {
            printf("KILL SWITCH ACTIVATED - Stopping all motors and servos\n");
            Motor_Set_Pwm(MOTOR_ID_M1, 0);
            Motor_Set_Pwm(MOTOR_ID_M2, 0);
            Motor_Set_Pwm(MOTOR_ID_M3, 0);
            Motor_Set_Pwm(MOTOR_ID_M4, 0);
            PwmServo_Set_Angle_All(90, 90, 90, 90);  // Safe values
        }
        else
        {
            // TODO: If all motor values are 0, stop immediately (brake/coast all motors)
            if (cmd.m1 == 0 && cmd.m2 == 0 && cmd.m3 == 0 && cmd.m4 == 0) {
                Motor_Stop(MOTOR_ID_M1);
                Motor_Stop(MOTOR_ID_M2);
                Motor_Stop(MOTOR_ID_M3);
                Motor_Stop(MOTOR_ID_M4);
                printf("All motors stopped (zero PWM command)\n");
            } else {
                // Print all received values (uncomment for debugging)
                // printf("\n=== NEW COMMAND RECEIVED ===\n");
                // printf("Motors: M1=%5d, M2=%5d, M3=%5d, M4=%5d\n",
                //        cmd.m1, cmd.m2, cmd.m3, cmd.m4);
                // printf("Servos: S1=%3d°, S2=%3d°, S3=%3d°, S4=%3d°\n",
                //        cmd.s1, cmd.s2, cmd.s3, cmd.s4);
                // printf("=============================\n");

                Motor_Set_Pwm(MOTOR_ID_M1, cmd.m1);
                Motor_Set_Pwm(MOTOR_ID_M2, cmd.m2);
                Motor_Set_Pwm(MOTOR_ID_M3, cmd.m3);
                Motor_Set_Pwm(MOTOR_ID_M4, cmd.m4);
                PwmServo_Set_Angle(1, cmd.s1);
                PwmServo_Set_Angle(2, cmd.s2);
                PwmServo_Set_Angle(3, cmd.s3);
                PwmServo_Set_Angle(4, cmd.s4);
            }
        }
    }
    Bsp_Led_Show_State_Handle();
    Beep_Timeout_Close_Handle();
    HAL_Delay(10);
}
