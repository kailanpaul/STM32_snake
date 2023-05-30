#ifndef Herkulex_h
#define Herkulex_h

#include "stm32f4xx_hal.h"

#define HEEPWRITE 0x01 // Rom write
#define HEEPREAD 0x02  // Rom read
#define HRAMWRITE 0x03 // Ram write
#define HRAMREAD 0x04  // Ram read
#define HIJOG 0x05     // Write n servo with different timing
#define HSJOG 0x06     // Write n servo with same time
#define HSTAT 0x07     // Read error
#define HROLLBACK 0x08 // Back to factory value
#define HREBOOT 0x09   // Reboot

#define H_LED_GREEN 0x01
#define H_LED_BLUE 0x02
#define H_LED_CYAN 0x03
#define H_LED_RED 0x04
#define H_LED_GREEN2 0x05
#define H_LED_PINK 0x06
#define H_LED_WHITE 0x07

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart4;

void herkulex_init(void);
void reboot(uint8_t servo_ID);
void clear_error(uint8_t servo_ID);
void torque_on(uint8_t servo_ID);
void torque_off(uint8_t servo_ID);
void move_one(uint8_t servo_ID, int goal, uint16_t p_time, uint8_t i_LED);
void move_one_angle(uint8_t servo_ID, float angle, uint16_t p_time, uint8_t i_LED);
void move_all(uint8_t servo_ID, int goal, uint8_t i_LED);
void move_all_angle(uint8_t servo_ID, float angle, uint8_t i_LED);
void move_speed_all(uint8_t servo_ID, int goal, uint8_t i_LED);
void move_speed_one(uint8_t servo_ID, int goal, uint16_t p_time, uint8_t i_LED);
void action_all(uint16_t p_time);
void set_LED(uint8_t servo_ID, uint8_t LED_val);
uint8_t stat(uint8_t servo_ID);
uint8_t model(void);
void set_ID(uint8_t ID_old, uint8_t ID_new);
uint16_t get_position(uint8_t servo_ID);
float get_angle(uint8_t servo_ID);
uint16_t get_speed(uint8_t servo_ID);

#endif // Herkulex_h
