#ifndef __BSP_INIT_H
#define __BSP_INIT_H

#include "esp_err.h"
#include "step_motor.h"
#include "gpioled.h"
#include "uart_comm.h"
#include "motor_feedback.h"
#include "app_task.h"
#include "servo.h"

extern gpio_led_t led1;
extern uart_comm_handle_t uart;

extern step_motor_handle_t motor1;
extern step_motor_handle_t motor2;
extern step_motor_handle_t motor3;

extern motor_feedback_handle_t fb;

extern Servo claw_servo, pump_servo, valve_servo;

void bsp_init(void);


#endif /* __BSP_INIT_H */

