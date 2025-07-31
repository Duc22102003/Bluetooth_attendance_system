#include "lcd.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "unistd.h"
#include "string.h"
#include <stdio.h>

#define SLAVE_ADDRESS_LCD 0x4E >> 1 // change this according to ur setup

esp_err_t err;

#define I2C_NUM I2C_NUM_0

static const char *TAG = "LCD";

void lcd_send_cmd(char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	data_t[0] = data_u | 0x0C; // en=1, rs=0 1100
	data_t[1] = data_u | 0x08; // en=0, rs=0 1000
	data_t[2] = data_l | 0x0C; // en=1, rs=0
	data_t[3] = data_l | 0x08; // en=0, rs=0
	err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
	if (err != 0)
		ESP_LOGI(TAG, "Error in sending command");
}

void lcd_send_data(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D; // en=1, rs=1  1101
	data_t[1] = data_u | 0x09; // en=0, rs=1  1001
	data_t[2] = data_l | 0x0D; // en=1, rs=0
	data_t[3] = data_l | 0x09; // en=0, rs=0
	err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
	if (err != 0)
		ESP_LOGI(TAG, "Error in sending data");
}

// esp_err_t i2c_master_write_to_device(int i2c_num,uint8_t Address,uint8_t* data,int leng, int time){
// 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     ESP_ERROR_CHECK(i2c_master_start(cmd));
//     ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (Address << 1) | I2C_MASTER_WRITE, 1));
// 	int i = 0;
// 	for(i = 0 ; i < leng; i++){
//     ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[i], 1));
// 	}
//     ESP_ERROR_CHECK(i2c_master_stop(cmd));
// ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_num, cmd, time/portTICK_PERIOD_MS));
//     i2c_cmd_link_delete(cmd);
// 	return 0;
// }

void lcd_clear(void)
{
	lcd_send_cmd(0x01);
	usleep(5000);
}

void lcd_put_cur(int row, int col)
{
	switch (row)
	{
	case 0:
		col |= 0x80;
		break;
	case 1:
		col |= 0xC0;
		break;
	}

	lcd_send_cmd(col);
}

void lcd_init(void)
{
	// 4 bit initialisation
	usleep(50000); // wait for >40ms
	lcd_send_cmd(0x30);
	usleep(5000); // wait for >4.1ms
	lcd_send_cmd(0x30);
	usleep(200); // wait for >100us
	lcd_send_cmd(0x30);
	usleep(10000);
	lcd_send_cmd(0x20); // 4bit mode
	usleep(10000);

	// dislay initialisation
	lcd_send_cmd(0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	usleep(1000);
	lcd_send_cmd(0x08); // Display on/off control --> D=0,C=0, B=0  ---> display off
	usleep(1000);
	lcd_send_cmd(0x01); // clear display
	usleep(1000);
	usleep(1000);
	lcd_send_cmd(0x06); // Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	usleep(1000);
	lcd_send_cmd(0x0C); // Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
	usleep(1000);
}

void lcd_send_string(char *str)
{
	while (*str)
		lcd_send_data(*str++);
}

void lcd_go_left(int times)
{
	int i = 0;
	for (i = 0; i < times; i++)
	{
		lcd_send_cmd(0x1C);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void lcd_go_right(int times)
{
	int i = 0;
	for (i = 0; i < times; i++)
	{
		lcd_send_cmd(0x18);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}
// This function will split rec_buffer 2 part by "!".
/**
 * rec_buffer : "..............!Luu Dinh Tu, Nguyen Quang Toan,,"    
 * After that part 2 is "Luu Dinh Tu, Nguyen Quang Toan,,"
 */
void split_text(const char *input, char *part1, char *part2)
{
	char buffer[strlen(input) + 1];
	strcpy(buffer, input);

	char *token = strtok(buffer, "!");

	if (token != NULL)
	{
		strcpy(part1, token);
		token = strtok(NULL, "!");

		if (token != NULL)
		{
			strcpy(part2, token);
		}
		else
		{
			part2[0] = '\0';
		}
	}
	else
	{
		part1[0] = '\0';
		part2[0] = '\0';
	}
}
// This function will decode part2 
/**
 * After that names will be stord in array "names"
 * Finally, we can display each name sequencely on LCD
 */
void analyze_string(const char *input, char names[MAX_NAMES][MAX_NAME_LENGTH])
{

	int i = 0;
	char buffer[strlen(input) + 1];
	for (int j = 0; j < MAX_NAMES; j++)
	{
		names[j][0] = '\0';
	}
	strcpy(buffer, input);

	const char delim[] = ",";

	char *token = strtok(buffer, delim);

	while (token != NULL && i < MAX_NAMES)
	{
		strcpy(names[i], token);
		i++;
		token = strtok(NULL, delim);
	}
}

// int main() {
//     const char *input = "ádnksajndksajndkjasndasnkdsankjdsa!Luu Dinh Tu,Tran Duc Thang, ";
//     char part1[100];
//     char part2[100];
// 	char names[MAX_NAMES][MAX_NAME_LENGTH];

// 	split_text(input, part1, part2);

//     analyze_string(part2, names);
// 	for (int i = 0; i < MAX_NAMES; i++) {
// 		if (names[i] != NULL)
//         	printf("Name %d: %s\n", i + 1, names[i]);
//     }
//     const char *output = "ádnksajndksajndkjasndasnkdsankjdsa!Nguyen Quang Toan,Hehehe, ";
//     split_text(output, part1, part2);
// 	analyze_string(part2, names);
// 	for (int i = 0; i < MAX_NAMES; i++) {
// 		if (names[i] != NULL)
//         	printf("Name %d: %s\n", i + 1, names[i]);
//     }
//     return 0;
// }
