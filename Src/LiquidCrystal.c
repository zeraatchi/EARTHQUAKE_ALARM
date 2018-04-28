#include "LiquidCrystal.h"
#include <stdio.h>
#include "stm32f1xx_hal.h"

#define _data_pins_4  GPIO_PIN_4
#define _data_pins_5  GPIO_PIN_5
#define _data_pins_6  GPIO_PIN_6
#define _data_pins_7  GPIO_PIN_7

#define _data_pins_port_4  GPIOA
#define _data_pins_port_5  GPIOA
#define _data_pins_port_6  GPIOA
#define _data_pins_port_7  GPIOA

#define _enable_pin_port GPIOB
#define _enable_pin      GPIO_PIN_1

#define _rs_pin_port  GPIOB
#define _rs_pin       GPIO_PIN_0





// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1 
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

#define LOW GPIO_PIN_RESET 
#define HIGH GPIO_PIN_SET 




void lcd_begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
	_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;

  setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);  

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != LCD_5x8DOTS) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }



  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  HAL_Delay_Microseconds(50000); 
  // Now we pull both RS and R/W low to begin commands
	HAL_GPIO_WritePin(_rs_pin_port, _rs_pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(_enable_pin_port, _enable_pin,GPIO_PIN_RESET);

  
  //put the LCD into 4 bit or 8 bit mode
  if (! (_displayfunction & LCD_8BITMODE)) {
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    write4bits(0x03);
    HAL_Delay_Microseconds(4500); // wait min 4.1ms

    // second try
    write4bits(0x03);
    HAL_Delay_Microseconds(4500); // wait min 4.1ms
    
    // third go!
    write4bits(0x03); 
    HAL_Delay_Microseconds(150);

    // finally, set to 4-bit interface
    write4bits(0x02); 
  } else {
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    command(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay_Microseconds(4500);  // wait more than 4.1ms

    // second try
    command(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay_Microseconds(150);

    // third go
    command(LCD_FUNCTIONSET | _displayfunction);
  }

  // finally, set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);  

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);

}

void setRowOffsets(int row0, int row1, int row2, int row3)
{
  _row_offsets[0] = row0;
  _row_offsets[1] = row1;
  _row_offsets[2] = row2;
  _row_offsets[3] = row3;
}

/********** high level commands, for the user! */
void clear()
{
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  HAL_Delay_Microseconds(2000);  // this command takes a long time!
}

void home()
{
  command(LCD_RETURNHOME);  // set cursor position to zero
  HAL_Delay_Microseconds(2000);  // this command takes a long time!
}

void setCursor(uint8_t col, uint8_t row)
{
  const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
  if ( row >= max_lines ) {
    row = max_lines - 1;    // we count rows starting w/0
  }
  if ( row >= _numlines ) {
    row = _numlines - 1;    // we count rows starting w/0
  }
  
  command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

// Turn the display on/off (quickly)
void noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void cursor() {
  _displaycontrol |= LCD_CURSORON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void blink() {
  _displaycontrol |= LCD_BLINKON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void scrollDisplayLeft(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void scrollDisplayRight(void) {
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void leftToRight(void) {
  _displaymode |= LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void rightToLeft(void) {
  _displaymode &= ~LCD_ENTRYLEFT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void autoscroll(void) {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void noAutoscroll(void) {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    send(charmap[i],HIGH);
  }
}

/*********** mid level commands, for sending data/cmds */

void command(uint8_t value) {
  send(value, LOW);
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void send(uint8_t value, uint8_t mode) {
  HAL_GPIO_WritePin(_rs_pin_port,_rs_pin,mode);

  // if there is a RW pin indicated, set it low to Write

  
  if (_displayfunction & LCD_8BITMODE) {
  } else {
    write4bits(value>>4);
    write4bits(value);
  }
}

void pulseEnable(void) {
  HAL_GPIO_WritePin(_enable_pin_port, _enable_pin,GPIO_PIN_RESET);
  HAL_Delay_Microseconds(1);    
  HAL_GPIO_WritePin(_enable_pin_port, _enable_pin,GPIO_PIN_SET);
  HAL_Delay_Microseconds(1);    // enable pulse must be >450ns
  HAL_GPIO_WritePin(_enable_pin_port, _enable_pin,GPIO_PIN_RESET);
  HAL_Delay_Microseconds(100);   // commands need > 37us to settle
}

void write4bits(uint8_t value) {
	
	HAL_GPIO_WritePin(_data_pins_port_4,_data_pins_4,(value >> 0) & 0x01);
	HAL_GPIO_WritePin(_data_pins_port_5,_data_pins_5,(value >> 1) & 0x01);
	HAL_GPIO_WritePin(_data_pins_port_6,_data_pins_6,(value >> 2) & 0x01);
	HAL_GPIO_WritePin(_data_pins_port_7,_data_pins_7,(value >> 3) & 0x01);

  pulseEnable();
}
