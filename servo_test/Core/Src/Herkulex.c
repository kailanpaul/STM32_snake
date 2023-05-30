#include "Herkulex.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

#define DATA_SIZE 30 // buffer for input data
#define DATA_MOVE 50 // max 10 servos <---- change this for more servos!
#define TIME_OUT 5	 // timeout serial communication
#define BROADCAST_ID 0xFE

static uint16_t count;
static uint8_t string_length;
static uint8_t p_size;
static uint8_t p_ID;
static uint8_t cmd;
static uint8_t data[DATA_SIZE];
static uint8_t data_ex[DATA_MOVE + 8];
static uint8_t move_data[DATA_MOVE];
static uint8_t ck1;
static uint8_t ck2;
static uint8_t XOR;
static uint8_t x_time;

uint8_t checksum1(uint8_t *data, uint8_t string_length);
uint8_t checksum2(uint8_t XOR);
void ACK(uint8_t val);
void write_register_RAM(uint8_t servo_ID, uint8_t address, uint8_t write_byte);
void write_register_EEP(uint8_t servo_ID, uint8_t address, uint8_t write_byte);
void add_data(uint16_t goal_LSB, uint16_t goal_MSB, uint8_t set, uint8_t servo_ID);
void send_data(uint8_t *buffer, uint8_t length);
void read_data(uint8_t size);

void herkulex_init(void)
{
	count = 0;
	string_length = 0;
	HAL_Delay(100);
	clear_error(BROADCAST_ID); // clear error for all servos
	HAL_Delay(10);
	ACK(1); // set ACK
	HAL_Delay(10);
	torque_on(BROADCAST_ID); // torqueON for all servos
	HAL_Delay(10);
}

uint8_t checksum1(uint8_t *data, uint8_t string_length)
{
	XOR = 0;
	XOR = XOR ^ p_size;
	XOR = XOR ^ p_ID;
	XOR = XOR ^ cmd;
	for (uint8_t i = 0; i < string_length; i++)
	{
		XOR = XOR ^ data[i];
	}
	return XOR & 0xFE;
}

uint8_t checksum2(uint8_t XOR)
{
	return (~XOR) & 0xFE;
}

// reboot single servo - pay attention 253 - all servos doesn't work!
void reboot(uint8_t servo_ID)
{

	p_size = 0x07;	 // 3.Packet size 7-58
	p_ID = servo_ID; // 4. Servo ID - 253=all servos
	cmd = HREBOOT;	 // 5. CMD
	ck1 = (p_size ^ p_ID ^ cmd) & 0xFE;
	ck2 = (~(p_size ^ p_ID ^ cmd)) & 0xFE;

	data_ex[0] = 0xFF;	 // Packet Header
	data_ex[1] = 0xFF;	 // Packet Header
	data_ex[2] = p_size; // Packet Size
	data_ex[3] = p_ID;	 // Servo ID
	data_ex[4] = cmd;	 // Command Ram Write
	data_ex[5] = ck1;	 // Checksum 1
	data_ex[6] = ck2;	 // Checksum 2

	send_data(data_ex, p_size);
}

void clear_error(uint8_t servo_ID)
{
	p_size = 0x0B;	 // 3.Packet size 7-58
	p_ID = servo_ID; // 4. Servo ID - 253=all servos
	cmd = HRAMWRITE; // 5. CMD
	data[0] = 0x30;	 // 8. Address
	data[1] = 0x02;	 // 9. Length
	data[2] = 0x00;	 // 10. Write error=0
	data[3] = 0x00;	 // 10. Write detail error=0

	string_length = 4; // lengthData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	   // Packet Header
	data_ex[1] = 0xFF;	   // Packet Header
	data_ex[2] = p_size;   // Packet Size
	data_ex[3] = p_ID;	   // Servo ID
	data_ex[4] = cmd;	   // Command Ram Write
	data_ex[5] = ck1;	   // Checksum 1
	data_ex[6] = ck2;	   // Checksum 2
	data_ex[7] = data[0];  // Address 52
	data_ex[8] = data[1];  // Length
	data_ex[9] = data[2];  // Value1
	data_ex[10] = data[3]; // Value2

	send_data(data_ex, p_size);
}

// ACK  - 0=No reply, 1=Only reply to READ CMD, 2=Always reply
void ACK(uint8_t val)
{
	p_size = 0x0A;	   // 3.Packet size 7-58
	p_ID = 0xFE;	   // 4. Servo ID
	cmd = HRAMWRITE;   // 5. CMD
	data[0] = 0x34;	   // 8. Address
	data[1] = 0x01;	   // 9. Lenght
	data[2] = val;	   // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
	string_length = 3; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	  // Packet Header
	data_ex[1] = 0xFF;	  // Packet Header
	data_ex[2] = p_size;  // Packet Size
	data_ex[3] = p_ID;	  // Servo ID
	data_ex[4] = cmd;	  // Command Ram Write
	data_ex[5] = ck1;	  // Checksum 1
	data_ex[6] = ck2;	  // Checksum 2
	data_ex[7] = data[0]; // Address 52
	data_ex[8] = data[1]; // Length
	data_ex[9] = data[2]; // Value

	send_data(data_ex, p_size);
}

void torque_on(uint8_t servo_ID)
{
	p_size = 0x0A;	   // 3.Packet size 7-58
	p_ID = servo_ID;   // 4. Servo ID
	cmd = HRAMWRITE;   // 5. CMD
	data[0] = 0x34;	   // 8. Address
	data[1] = 0x01;	   // 9. Lenght
	data[2] = 0x60;	   // 10. 0x60=Torque ON
	string_length = 3; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	  // Packet Header
	data_ex[1] = 0xFF;	  // Packet Header
	data_ex[2] = p_size;  // Packet Size
	data_ex[3] = p_ID;	  // Servo ID
	data_ex[4] = cmd;	  // Command Ram Write
	data_ex[5] = ck1;	  // Checksum 1
	data_ex[6] = ck2;	  // Checksum 2
	data_ex[7] = data[0]; // Address 52
	data_ex[8] = data[1]; // Length
	data_ex[9] = data[2]; // Torque ON

	send_data(data_ex, p_size);
}

void torque_off(uint8_t servo_ID)
{
	p_size = 0x0A;	   // 3.Packet size 7-58
	p_ID = servo_ID;   // 4. Servo ID
	cmd = HRAMWRITE;   // 5. CMD
	data[0] = 0x34;	   // 8. Address
	data[1] = 0x01;	   // 9. Lenght
	data[2] = 0x00;	   // 10. 0x00=Torque Free
	string_length = 3; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	  // Packet Header
	data_ex[1] = 0xFF;	  // Packet Header
	data_ex[2] = p_size;  // Packet Size
	data_ex[3] = p_ID;	  // Servo ID
	data_ex[4] = cmd;	  // Command Ram Write
	data_ex[5] = ck1;	  // Checksum 1
	data_ex[6] = ck2;	  // Checksum 2
	data_ex[7] = data[0]; // Address 52
	data_ex[8] = data[1]; // Length
	data_ex[9] = data[2]; // Torque Free

	send_data(data_ex, p_size);
}

// move one servo at goal position 0 - 1024
void move_one(uint8_t servo_ID, int goal, uint16_t p_time, uint8_t i_LED)
{
	if (goal > 1023 || goal < 0)
		return; // speed (goal) non correct
	if ((p_time < 0) || (p_time > 2856))
		return;

	// Position definition
	uint16_t pos_LSB = goal & 0X00FF;		 // MSB Pos
	uint16_t pos_MSB = (goal & 0XFF00) >> 8; // LSB Pos

	// led
	uint8_t i_blue = 0;
	uint8_t i_green = 0;
	uint8_t i_red = 0;
	switch (i_LED)
	{
	case 1:
		i_green = 1;
		break;
	case 2:
		i_blue = 1;
		break;
	case 3:
		i_red = 1;
		break;
	}
	uint8_t set_val = i_green * 4 + i_blue * 8 + i_red * 16; // assign led value

	x_time = (uint8_t)((float)p_time / 11.2); // 8. Execution time

	p_size = 0x0C; // 3.Packet size 7-58
	cmd = HSJOG;   // 5. CMD

	data[0] = pos_LSB;	// 8. speedLSB
	data[1] = pos_MSB;	// 9. speedMSB
	data[2] = set_val;	// 10. Mode=0;
	data[3] = servo_ID; // 11. ServoID

	p_ID = servo_ID ^ x_time;

	string_length = 4; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	p_ID = servo_ID;

	data_ex[0] = 0xFF;	 // Packet Header
	data_ex[1] = 0xFF;	 // Packet Header
	data_ex[2] = p_size; // Packet Size
	data_ex[3] = p_ID;	 // Servo ID
	data_ex[4] = cmd;	 // Command Ram Write
	data_ex[5] = ck1;	 // Checksum 1
	data_ex[6] = ck2;	 // Checksum 2
	data_ex[7] = x_time; // Execution time
	data_ex[8] = data[0];
	data_ex[9] = data[1];
	data_ex[10] = data[2];
	data_ex[11] = data[3];

	send_data(data_ex, p_size);
}

// move one servo to an angle between -160 and 160
void move_one_angle(uint8_t servo_ID, float angle, uint16_t p_time, uint8_t i_LED)
{
	if (angle > 160.0 || angle < -160.0)
		return;
	int position = (int)(angle / 0.325) + 512;
	move_one(servo_ID, position, p_time, i_LED);
}

// move all servo at the same time to a position: servo list building
void move_all(uint8_t servo_ID, int goal, uint8_t i_LED)
{
	if (goal > 1023 || goal < 0)
		return; // 0 <--> 1023 range

	uint8_t i_mode = 0; // mode=position
	uint8_t i_stop = 0; // stop=0

	// Position definition
	uint16_t pos_LSB = goal & 0X00FF;		 // MSB Pos
	uint16_t pos_MSB = (goal & 0XFF00) >> 8; // LSB Pos

	// led
	uint8_t i_blue = 0;
	uint8_t i_green = 0;
	uint8_t i_red = 0;
	switch (i_LED)
	{
	case 1:
		i_green = 1;
		break;
	case 2:
		i_blue = 1;
		break;
	case 3:
		i_red = 1;
		break;
	}

	uint8_t set_value = i_stop + i_mode * 2 + i_green * 4 + i_blue * 8 + i_red * 16; // assign led value

	add_data(pos_LSB, pos_MSB, set_value, servo_ID); // add servo data to list, pos mode
}

// move all servo at the same time to a position: servo list building
void move_all_angle(uint8_t servo_ID, float angle, uint8_t i_LED)
{
	if (angle > 160.0 || angle < -160.0)
		return; // out of the range
	int position = (int)(angle / 0.325) + 512;
	move_all(servo_ID, position, i_LED);
}

// move all servo at the same time with different speeds: servo list building
void move_speed_all(uint8_t servo_ID, int goal, uint8_t i_LED)
{
	if (goal > 1023 || goal < -1023)
		return; //-1023 <--> 1023 range

	uint8_t i_mode = 1; // mode=continous rotation
	uint8_t i_stop = 0; // Stop=0

	// Speed definition
	uint16_t goal_speed_sign;
	if (goal < 0)
	{
		goal_speed_sign = (-1) * goal;
		goal_speed_sign |= 0x4000; // bit n�14
	}
	else
	{
		goal_speed_sign = goal;
	}

	uint16_t goal_speed_LSB = goal_speed_sign & 0X00FF;		   // MSB speedGoal
	uint16_t goal_speed_MSB = (goal_speed_sign & 0xFF00) >> 8; // LSB speedGoal

	// led
	uint8_t i_blue = 0;
	uint8_t i_green = 0;
	uint8_t i_red = 0;
	switch (i_LED)
	{
	case 1:
		i_green = 1;
		break;
	case 2:
		i_blue = 1;
		break;
	case 3:
		i_red = 1;
		break;
	}

	uint8_t set_value = i_stop + i_mode * 2 + i_green * 4 + i_blue * 8 + i_red * 16; // assign led value

	add_data(goal_speed_LSB, goal_speed_MSB, set_value, servo_ID); // add servo data to list, speed mode
}

// move one servo with continous rotation
void move_speed_one(uint8_t servo_ID, int goal, uint16_t p_time, uint8_t i_LED)
{
	if (goal > 1023 || goal < -1023)
		return; // speed (goal) non correct
	if ((p_time < 0) || (p_time > 2856))
		return;

	int goal_speed_sign;
	if (goal < 0)
	{
		goal_speed_sign = (-1) * goal;
		goal_speed_sign |= 0x4000; // bit n�14
	}
	else
	{
		goal_speed_sign = goal;
	}
	uint16_t goal_speed_LSB = goal_speed_sign & 0X00FF;		   // MSB speedGoal
	uint16_t goal_speed_MSB = (goal_speed_sign & 0xFF00) >> 8; // LSB speedGoal

	// led
	uint8_t i_blue = 0;
	uint8_t i_green = 0;
	uint8_t i_red = 0;
	switch (i_LED)
	{
	case 1:
		i_green = 1;
		break;
	case 2:
		i_blue = 1;
		break;
	case 3:
		i_red = 1;
		break;
	}

	uint8_t set_value = 2 + i_green * 4 + i_blue * 8 + i_red * 16; // assign led value

	x_time = (uint8_t)((float)p_time / 11.2); // 8. Execution time

	p_size = 0x0C; // 3.Packet size 7-58
	cmd = HSJOG;   // 5. CMD

	data[0] = goal_speed_LSB; // 8. speedLSB
	data[1] = goal_speed_MSB; // 9. speedMSB
	data[2] = set_value;	  // 10. Mode=0;
	data[3] = servo_ID;		  // 11. ServoID

	p_ID = servo_ID ^ x_time;

	string_length = 4; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	p_ID = servo_ID;

	data_ex[0] = 0xFF;	 // Packet Header
	data_ex[1] = 0xFF;	 // Packet Header
	data_ex[2] = p_size; // Packet Size
	data_ex[3] = p_ID;	 // Servo ID
	data_ex[4] = cmd;	 // Command Ram Write
	data_ex[5] = ck1;	 // Checksum 1
	data_ex[6] = ck2;	 // Checksum 2
	data_ex[7] = x_time; // Execution time
	data_ex[8] = data[0];
	data_ex[9] = data[1];
	data_ex[10] = data[2];
	data_ex[11] = data[3];

	send_data(data_ex, p_size);
}

// move all servo with the same execution time
void action_all(uint16_t p_time)
{
	if ((p_time < 0) || (p_time > 2856))
		return;

	p_size = 0x08 + count;					  // 3.Packet size 7-58
	cmd = HSJOG;							  // 5. CMD SJOG Write n servo with same execution time
	x_time = (uint8_t)((float)p_time / 11.2); // 8. Execution time

	p_ID = 0xFE ^ x_time;
	ck1 = checksum1(move_data, count); // 6. Checksum1
	ck2 = checksum2(ck1);			   // 7. Checksum2

	p_ID = 0xFE;
	data_ex[0] = 0xFF;	 // Packet Header
	data_ex[1] = 0xFF;	 // Packet Header
	data_ex[2] = p_size; // Packet Size
	data_ex[3] = p_ID;	 // Servo ID
	data_ex[4] = cmd;	 // Command Ram Write
	data_ex[5] = ck1;	 // Checksum 1
	data_ex[6] = ck2;	 // Checksum 2
	data_ex[7] = x_time; // Execution time

	for (uint8_t i = 0; i < count; i++)
		data_ex[i + 8] = move_data[i]; // Variable servo data

	send_data(data_ex, p_size);

	count = 0; // reset counter
}

void set_LED(uint8_t servo_ID, uint8_t LED_val)
{
	p_size = 0x0A;	   // 3.Packet size 7-58
	p_ID = servo_ID;   // 4. Servo ID
	cmd = HRAMWRITE;   // 5. CMD
	data[0] = 0x35;	   // 8. Address 53
	data[1] = 0x01;	   // 9. Lenght
	data[2] = LED_val; // 10.LedValue
	string_length = 3; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	  // Packet Header
	data_ex[1] = 0xFF;	  // Packet Header
	data_ex[2] = p_size;  // Packet Size
	data_ex[3] = p_ID;	  // Servo ID
	data_ex[4] = cmd;	  // Command Ram Write
	data_ex[5] = ck1;	  // Checksum 1
	data_ex[6] = ck2;	  // Checksum 2
	data_ex[7] = data[0]; // Address
	data_ex[8] = data[1]; // Length
	data_ex[9] = data[2]; // Value

	send_data(data_ex, p_size);
}

uint8_t stat(uint8_t servo_ID)
{
	p_size = 0x07;	 // 3.Packet size
	p_ID = servo_ID; // 4.Servo ID - 0XFE=All servos
	cmd = HSTAT;	 // 5.CMD

	ck1 = (p_size ^ p_ID ^ cmd) & 0xFE;
	ck2 = (~(p_size ^ p_ID ^ cmd)) & 0xFE;

	data_ex[0] = 0xFF;	 // Packet Header
	data_ex[1] = 0xFF;	 // Packet Header
	data_ex[2] = p_size; // Packet Size
	data_ex[3] = p_ID;	 // Servo ID
	data_ex[4] = cmd;	 // Command Ram Write
	data_ex[5] = ck1;	 // Checksum 1
	data_ex[6] = ck2;	 // Checksum 2

	send_data(data_ex, p_size);
	HAL_Delay(2);
	read_data(9); // read 9 bytes from serial

	p_size = data_ex[2]; // 3.Packet size 7-58
	p_ID = data_ex[3];	 // 4. Servo ID
	cmd = data_ex[4];	 // 5. CMD
	data[0] = data_ex[7];
	data[1] = data_ex[8];
	string_length = 2;

	ck1 = (data_ex[2] ^ data_ex[3] ^ data_ex[4] ^ data_ex[7] ^ data_ex[8]) & 0xFE;
	ck2 = checksum2(ck1);

	if (ck1 != data_ex[5])
		return -1; // checksum verify
	if (ck2 != data_ex[6])
		return -2;

	return data_ex[7]; // return status
}

// model - 1=0101 - 2=0201
uint8_t model(void)
{
	p_size = 0x09;	   // 3.Packet size 7-58
	p_ID = 0xFE;	   // 4. Servo ID
	cmd = HEEPREAD;	   // 5. CMD
	data[0] = 0x00;	   // 8. Address
	data[1] = 0x01;	   // 9. Lenght
	string_length = 2; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1read_data
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	  // Packet Header
	data_ex[1] = 0xFF;	  // Packet Header
	data_ex[2] = p_size;  // Packet Size
	data_ex[3] = p_ID;	  // Servo ID
	data_ex[4] = cmd;	  // Command Ram Write
	data_ex[5] = ck1;	  // Checksum 1
	data_ex[6] = ck2;	  // Checksum 2
	data_ex[7] = data[0]; // Address
	data_ex[8] = data[1]; // Length

	send_data(data_ex, p_size);

	HAL_Delay(1);
	read_data(9);

	p_size = data_ex[2];  // 3.Packet size 7-58
	p_ID = data_ex[3];	  // 4. Servo ID
	cmd = data_ex[4];	  // 5. CMD
	data[0] = data_ex[7]; // 8. 1st byte
	string_length = 1;	  // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	if (ck1 != data_ex[5])
		return -1; // checksum verify
	if (ck2 != data_ex[6])
		return -2;

	return data_ex[7]; // return status
}

// setID - Need to restart the servo
void set_ID(uint8_t ID_old, uint8_t ID_new)
{
	p_size = 0x0A;	   // 3.Packet size 7-58
	p_ID = ID_old;	   // 4. Servo ID OLD - original servo ID
	cmd = HEEPWRITE;   // 5. CMD
	data[0] = 0x06;	   // 8. Address
	data[1] = 0x01;	   // 9. Lenght
	data[2] = ID_new;  // 10. ServoID NEW
	string_length = 3; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	  // Packet Header
	data_ex[1] = 0xFF;	  // Packet Header
	data_ex[2] = p_size;  // Packet Size
	data_ex[3] = p_ID;	  // Servo ID
	data_ex[4] = cmd;	  // Command Ram Write
	data_ex[5] = ck1;	  // Checksum 1
	data_ex[6] = ck2;	  // Checksum 2
	data_ex[7] = data[0]; // Address 52
	data_ex[8] = data[1]; // Length
	data_ex[9] = data[2]; // Value

	send_data(data_ex, p_size);
}

// get Position
uint16_t get_position(uint8_t servo_ID)
{
	uint16_t position = 0;

	p_size = 0x09;	 // 3.Packet size 7-58
	p_ID = servo_ID; // 4. Servo ID - 253=all servos
	cmd = HRAMREAD;	 // 5. CMD
	data[0] = 0x3A;	 // 8. Address
	data[1] = 0x02;	 // 9. Lenght

	string_length = 2; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	  // Packet Header
	data_ex[1] = 0xFF;	  // Packet Header
	data_ex[2] = p_size;  // Packet Size
	data_ex[3] = p_ID;	  // Servo ID
	data_ex[4] = cmd;	  // Command Ram Write
	data_ex[5] = ck1;	  // Checksum 1
	data_ex[6] = ck2;	  // Checksum 2
	data_ex[7] = data[0]; // Address
	data_ex[8] = data[1]; // Length

	send_data(data_ex, p_size);

	HAL_Delay(1);
	read_data(13);

	p_size = data_ex[2]; // 3.Packet size 7-58
	p_ID = data_ex[3];	 // 4. Servo ID
	cmd = data_ex[4];	 // 5. CMD
	data[0] = data_ex[7];
	data[1] = data_ex[8];
	data[2] = data_ex[9];
	data[3] = data_ex[10];
	data[4] = data_ex[11];
	data[5] = data_ex[12];
	string_length = 6;

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	if (ck1 != data_ex[5])
		return -1;
	if (ck2 != data_ex[6])
		return -1;

	position = ((data_ex[10] & 0x03) << 8) | data_ex[9];
	return position;
}

float get_angle(uint8_t servo_ID)
{
	int pos = (int)get_position(servo_ID);
	return (pos - 512) * 0.325;
}

// get the speed for one servo - values betweeb -1023 <--> 1023
uint16_t get_speed(uint8_t servo_ID)
{
	uint16_t speed = 0;

	p_size = 0x09;	 // 3.Packet size 7-58
	p_ID = servo_ID; // 4. Servo ID
	cmd = HRAMREAD;	 // 5. CMD
	data[0] = 0x40;	 // 8. Address
	data[1] = 0x02;	 // 9. Lenght

	string_length = 2; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	  // Packet Header
	data_ex[1] = 0xFF;	  // Packet Header
	data_ex[2] = p_size;  // Packet Size
	data_ex[3] = p_ID;	  // Servo ID
	data_ex[4] = cmd;	  // Command Ram Write
	data_ex[5] = ck1;	  // Checksum 1
	data_ex[6] = ck2;	  // Checksum 2
	data_ex[7] = data[0]; // Address
	data_ex[8] = data[1]; // Length

	send_data(data_ex, p_size);

	HAL_Delay(1);
	read_data(13);

	p_size = data_ex[2]; // 3.Packet size 7-58
	p_ID = data_ex[3];	 // 4. Servo ID
	cmd = data_ex[4];	 // 5. CMD
	data[0] = data_ex[7];
	data[1] = data_ex[8];
	data[2] = data_ex[9];
	data[3] = data_ex[10];
	data[4] = data_ex[11];
	data[5] = data_ex[12];
	string_length = 6;

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	if (ck1 != data_ex[5])
		return -1;
	if (ck2 != data_ex[6])
		return -1;

	speed = ((data_ex[10] & 0xFF) << 8) | data_ex[9];
	return speed;
}

// write registry in the RAM: one byte
void write_register_RAM(uint8_t servo_ID, uint8_t address, uint8_t write_byte)
{
	p_size = 0x0A;		  // 3.Packet size 7-58
	p_ID = servo_ID;	  // 4. Servo ID - 253=all servos
	cmd = HRAMWRITE;	  // 5. CMD
	data[0] = address;	  // 8. Address
	data[1] = 0x01;		  // 9. Lenght
	data[2] = write_byte; // 10. Write error=0

	string_length = 3; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	   // Packet Header
	data_ex[1] = 0xFF;	   // Packet Header
	data_ex[2] = p_size;   // Packet Size
	data_ex[3] = p_ID;	   // Servo ID
	data_ex[4] = cmd;	   // Command Ram Write
	data_ex[5] = ck1;	   // Checksum 1
	data_ex[6] = ck2;	   // Checksum 2
	data_ex[7] = data[0];  // Address 52
	data_ex[8] = data[1];  // Length
	data_ex[9] = data[2];  // Value1
	data_ex[10] = data[3]; // Value2

	send_data(data_ex, p_size);
}

// write registry in the EEP memory (ROM): one byte
void write_register_EEP(uint8_t servo_ID, uint8_t address, uint8_t write_byte)
{
	p_size = 0x0A;		  // 3.Packet size 7-58
	p_ID = servo_ID;	  // 4. Servo ID - 253=all servos
	cmd = HEEPWRITE;	  // 5. CMD
	data[0] = address;	  // 8. Address
	data[1] = 0x01;		  // 9. Lenght
	data[2] = write_byte; // 10. Write error=0

	string_length = 3; // lenghtData

	ck1 = checksum1(data, string_length); // 6. Checksum1
	ck2 = checksum2(ck1);				  // 7. Checksum2

	data_ex[0] = 0xFF;	   // Packet Header
	data_ex[1] = 0xFF;	   // Packet Header
	data_ex[2] = p_size;   // Packet Size
	data_ex[3] = p_ID;	   // Servo ID
	data_ex[4] = cmd;	   // Command Ram Write
	data_ex[5] = ck1;	   // Checksum 1
	data_ex[6] = ck2;	   // Checksum 2
	data_ex[7] = data[0];  // Address 52
	data_ex[8] = data[1];  // Length
	data_ex[9] = data[2];  // Value1
	data_ex[10] = data[3]; // Value2

	send_data(data_ex, p_size);
}

// add data to variable list servo for syncro execution
void add_data(uint16_t goal_LSB, uint16_t goal_MSB, uint8_t set, uint8_t servo_ID)
{
	move_data[count++] = goal_LSB;
	move_data[count++] = goal_MSB;
	move_data[count++] = set;
	move_data[count++] = servo_ID;
}

void send_data(uint8_t *buffer, uint8_t length)
{
	//		clearBuffer(); 		//clear the serialport buffer - try to do it!
	if (HAL_UART_Transmit(&huart4, buffer, length, TIME_OUT) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(1);
}

void read_data(uint8_t size)
{
	// int i = 0;
	// int beginsave=0;
	// int Time_Counter=0;
	// switch (port)
	// {
	// case SSerial:

	//     while((SwSerial.available() < size) & (Time_Counter < TIME_OUT)){
	//     		Time_Counter++;
	//     		delayMicroseconds(1000);  //wait 1 millisecond for 10 times
	// 	}

	// 	while (SwSerial.available() > 0){
	// 		byte inchar = (byte)SwSerial.read();
	// 		if ( (inchar == 0xFF) & ((byte)SwSerial.peek() == 0xFF) ){
	// 				beginsave=1;
	// 				i=0; 				 // if found new header, begin again
	// 		}
	// 		if (beginsave==1 && i<size) {
	// 			   dataEx[i] = inchar;
	// 			   i++;
	// 		}
	// 	}
	// 	SwSerial.flush();
	// 	break;

	// #if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
	// case HSerial1:
	// 	while((Serial1.available() < size) & (Time_Counter < TIME_OUT)){
	//     		Time_Counter++;
	//     		delayMicroseconds(1000);
	// 	}
	// 	while (Serial1.available() > 0){
	//   		byte inchar = (byte)Serial1.read();
	// 		//printHexByte(inchar);
	//     	if ( (inchar == 0xFF) & ((byte)Serial1.peek() == 0xFF) ){
	// 					beginsave=1;
	// 					i=0;
	//          }
	//         if (beginsave==1 && i<size) {
	//                    dataEx[i] = inchar;
	//                    i++;
	// 		}
	// 	}
	// 	break;

	// case HSerial2:
	//     while((Serial2.available() < size) & (Time_Counter < TIME_OUT)){
	//     		Time_Counter++;
	//     		delayMicroseconds(1000);
	// 	}

	// 	while (Serial2.available() > 0){
	// 		byte inchar = (byte)Serial2.read();
	// 		if ( (inchar == 0xFF) & ((byte)Serial2.peek() == 0xFF) ){
	// 				beginsave=1;
	// 				i=0;
	// 		}
	// 		if (beginsave==1 && i<size) {
	// 			   dataEx[i] = inchar;
	// 			   i++;
	// 		}
	// 	}
	// 	break;

	// case HSerial3:
	// 	while((Serial3.available() < size) & (Time_Counter < TIME_OUT)){
	// 		Time_Counter++;
	// 		delayMicroseconds(1000);
	// 	}

	// 	while (Serial3.available() > 0){
	// 		byte inchar = (byte)Serial3.read();
	// 		if ( (inchar == 0xFF) & ((byte)Serial3.peek() == 0xFF) ){
	// 				beginsave=1;
	// 				i=0;
	// 		}
	// 		if (beginsave==1 && i<size) {
	// 			   dataEx[i] = inchar;
	// 			   i++;
	// 		}
	// 	}
	// 	break;
	// #endif
	// }
}
