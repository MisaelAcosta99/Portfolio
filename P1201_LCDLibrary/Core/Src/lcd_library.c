/*
 * lcd_library.c
 *
 *  Created on: Mar 14, 2025
 *      Author: Misael Acosta
 */

#include "lcd_library.h"

void lcd_w_r_epin(LCD_HandleTypeDef *hlcd){
	HAL_GPIO_WritePin(hlcd->EPort, hlcd->EPin, PIN_ON);
	HAL_Delay(TIME_1);
	HAL_GPIO_WritePin(hlcd->EPort, hlcd->EPin, PIN_OFF);
}

void lcd_write_nibble_command(LCD_HandleTypeDef *hlcd, uint8_t command){

	HAL_GPIO_WritePin(hlcd->RsPort, hlcd->RsPin, PIN_OFF);

	HAL_GPIO_WritePin(hlcd->DB4Port, hlcd->DB4Pin,  (command >> 0) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB5Port, hlcd->DB5Pin, (command >> 1) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB6Port, hlcd->DB6Pin,  (command >> 2) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB7Port, hlcd->DB7Pin,  (command >> 3) & 0x01);
	lcd_w_r_epin(hlcd);
}

void lcd_write_command(LCD_HandleTypeDef *hlcd, uint8_t command){

	uint8_t firstNibble = (command >> NIBBLE) & MASK_NIBBLE;
	uint8_t SecondNibble = command & MASK_NIBBLE;
	uint8_t i = INIT_VALUE;

	HAL_GPIO_WritePin(hlcd->RsPort, hlcd->RsPin, PIN_OFF);

	HAL_GPIO_WritePin(hlcd->DB4Port, hlcd->DB4Pin, (firstNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB5Port, hlcd->DB5Pin, (firstNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB6Port, hlcd->DB6Pin, (firstNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB7Port, hlcd->DB7Pin, (firstNibble >> i++) & 0x01);
	lcd_w_r_epin(hlcd);

	i= INIT_VALUE;
	HAL_GPIO_WritePin(hlcd->DB4Port, hlcd->DB4Pin, (SecondNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB5Port, hlcd->DB5Pin, (SecondNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB6Port, hlcd->DB6Pin, (SecondNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB7Port, hlcd->DB7Pin, (SecondNibble >> i++) & 0x01);
	lcd_w_r_epin(hlcd);
}

void lcd_write_chr(LCD_HandleTypeDef *hlcd, char data){

	uint8_t firstNibble = (data >> NIBBLE) & MASK_NIBBLE;
	uint8_t SecondNibble = data & MASK_NIBBLE;
	uint8_t i = INIT_VALUE;

	HAL_GPIO_WritePin(hlcd->RsPort, hlcd->RsPin, PIN_ON);
	HAL_GPIO_WritePin(hlcd->DB4Port, hlcd->DB4Pin, (firstNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB5Port, hlcd->DB5Pin, (firstNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB6Port, hlcd->DB6Pin, (firstNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB7Port, hlcd->DB7Pin, (firstNibble >> i++) & 0x01);
	lcd_w_r_epin(hlcd);

	i = INIT_VALUE;
	HAL_GPIO_WritePin(hlcd->DB4Port, hlcd->DB4Pin, (SecondNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB5Port, hlcd->DB5Pin, (SecondNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB6Port, hlcd->DB6Pin, (SecondNibble >> i++) & 0x01);
	HAL_GPIO_WritePin(hlcd->DB7Port, hlcd->DB7Pin, (SecondNibble >> i++) & 0x01);
	lcd_w_r_epin(hlcd);
}

void lcd_write_data(LCD_HandleTypeDef *hlcd, char* data, uint8_t size_vector){
	uint8_t i = INIT_VALUE;

	for (i = INIT_VALUE; i < size_vector; i++ ){
		lcd_write_chr(hlcd,data[i]);
		HAL_Delay(TIME_2);
	}
}

void lcd_set_position(LCD_HandleTypeDef *hlcd, uint8_t row, uint8_t column){

	lcd_write_command(hlcd, ADDR_RETURN_HOME);

	if(row == ROW_0)
	{
		lcd_write_command(hlcd, ADDR_SET_DDRAM | column);
	}
	else if(row == ROW_1)
	{
		lcd_write_command(hlcd, ADDR_SET_DDRAM | INIT_1_ROW | column);
	}
	else if(row == ROW_2)
	{
		lcd_write_command(hlcd, ADDR_SET_DDRAM | INIT_2_ROW | column);
	}
	else
	{
		lcd_write_command(hlcd, ADDR_SET_DDRAM | INIT_3_ROW | column);
	}
}

void lcd_init(LCD_HandleTypeDef *hlcd){

	lcd_write_nibble_command(hlcd,0x02); /*Set 4 bit*/
	HAL_Delay(TIME_1);
	lcd_write_command(hlcd,ADDR_FUNCTION_SET | DL_4_BITS | N_2_LINES | F_DOTS_5X8); /*Function set: Set 4 bit*/
	HAL_Delay(TIME_1);
	lcd_write_command(hlcd,ADDR_DISPLAY_ON_OFF_CTRL | D_DISPLAY_ON | C_CURSOR_ON); /*Display on/off control: display on, cursor on*/
	lcd_write_command(hlcd,ADDR_ENTRY_MODE_SET | ID_INCREMENT); /*Entry mode set: Increment by 1, no shift*/
}
