/*
 * lcd_library.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Misael Acosta
 */

#ifndef INC_LCD_LIBRARY_H_
#define INC_LCD_LIBRARY_H_

#include "main.h"

#define PIN_OFF GPIO_PIN_RESET
#define PIN_ON GPIO_PIN_SET
#define ADDR_CLEAR_DISPLAY 			0x01
#define ADDR_RETURN_HOME			0x02
#define ADDR_ENTRY_MODE_SET			0x04
#define ID_INCREMENT				0x02
#define S_SHIFT_ACTIVATE			0x01
#define ADDR_DISPLAY_ON_OFF_CTRL    0x08
#define D_DISPLAY_ON				0x04
#define C_CURSOR_ON					0x02
#define C_BLINKIN_ON				0x01
#define ADDR_CURSOR_DISPLAY_SHIFT 	0x10
#define RL_SHIFT_RIGHT 				0x04
#define RL_SHIFT_LEFT 				0x00
#define SC_DISPLAY_SHIFT 			0x08
#define SC_DISPLAY_MOVE 			0x00
#define ADDR_FUNCTION_SET 			0x20
#define DL_8_BITS					0x01
#define DL_4_BITS					0x00
#define N_1_LINE					0x00
#define N_2_LINES					0x08
#define F_DOTS_5X10                 0x04
#define F_DOTS_5X8                  0x00
#define ADDR_SET_DDRAM				0x80
#define INIT_1_ROW					0x40
#define INIT_2_ROW					0x10
#define INIT_3_ROW					0x50

#define INIT_VALUE  	0
#define ROW_0 			0
#define ROW_1			1
#define ROW_2			2
#define TIME_1			5
#define TIME_2			10
#define MASK_NIBBLE		0x0F
#define NIBBLE 			4


typedef struct
{
  GPIO_TypeDef        *EPort;
  uint32_t            EPin;
  GPIO_TypeDef        *RsPort;
  uint32_t            RsPin;
  GPIO_TypeDef        *DB7Port;
  uint32_t            DB7Pin;
  GPIO_TypeDef        *DB6Port;
  uint32_t            DB6Pin;
  GPIO_TypeDef        *DB5Port;
  uint32_t            DB5Pin;
  GPIO_TypeDef        *DB4Port;
  uint32_t            DB4Pin;

} LCD_HandleTypeDef;

void lcd_write_nibble_command(LCD_HandleTypeDef *hlcd, uint8_t command);

void lcd_w_r_epin(LCD_HandleTypeDef *hlcd);

void lcd_write_command(LCD_HandleTypeDef *hlcd, uint8_t command);

void lcd_write_chr(LCD_HandleTypeDef *hlcd, char data);

void lcd_write_data(LCD_HandleTypeDef *hlcd, char* data, uint8_t size_vector);

void lcd_set_position(LCD_HandleTypeDef *hlcd, uint8_t row, uint8_t column);

void lcd_init(LCD_HandleTypeDef *hlcd);



#endif /* INC_LCD_LIBRARY_H_ */
