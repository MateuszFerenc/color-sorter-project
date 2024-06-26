//#define __AVR_ATmega16__
#define F_CPU 14745600UL

// Include section start

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/cpufunc.h>
#include <avr/pgmspace.h>
#include <stdio.h>

// Include section end

// Preprocessor definitions start

#define PIN_clear(port, pin)                port &= ~( 1 << pin )
#define PIN_set(port, pin)                  port |= ( 1 << pin )
#define PIN_toggle(port, pin)               port ^= ( 1 << pin )
#define PIN_control(port, pin, state)       port = ( port & ~( 1 << pin ) ) | ( state << pin )

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define stable_version  0
#define beta_version    1

// sense stages ADC channels PORTA
#define metal_sense_adc 0
#define glass_sense_adc 1
#define color_sense_adc 2

#define DISP_STATE_NOP              0
#define DISP_STATE_WAIT             1
#define DISP_STATE_PUTHDATA         2
#define DISP_STATE_PUTLDATA         3
#define DISP_STATE_ENTOGGLE         4
#define DISP_STATE_CLEAR            5
#define DISP_STATE_HOME             6
#define DISP_STATE_ENWAIT           7
#define DISP_STATE_MCURSOR          8

#define PWM0_pin        PB3
#define PWM0_port       PORTB
#define PWM1_pin        PD5
#define PWM1_port       PORTD
#define PWM2_pin        PD7
#define PWM2_port       PORTD

#define PWM0            0
#define PWM1            1
#define PWM2            2

#define DISP_FRONTBUFFER    (unsigned char)   0
#define DISP_BACKBUFFER     (unsigned char)   80

#define RED_LED_pin         PB0
#define RED_LED_port        PORTB
#define GREEN_LED_pin       PB1
#define GREEN_LED_port      PORTB
#define BLUE_LED_pin        PB2
#define BLUE_LED_port       PORTB
#define WHITE_LED_pin       PA7
#define WHITE_LED_port      PORTA

#define BUZZER_pin          PD2
#define BUZZER_port         PORTD

#define MENU_STATE_MAIN         0
#define MENU_STATE_START        1
#define MENU_STATE_STOP         2
#define MENU_STATE_SELECT       3
#define MENU_STATE_SETTINGS     4
#define MENU_STATE_PWROFF       5

// Preprocessor definitions end

/* J1 - Metal stage
1 - metal sense         (PA0)
2 - object present 0    (PA4)
3 - stage 0 motor       (PB3)
*/

/* J2 - Glass stage
1 - glass sense         (PA1)
2 - object present 1    (PA5)
3 - stage 1 motor       (PD5)
*/

/* J3 - Color stage
1 - color sense         (PA2)
2 - object present 2    (PA6)
3 - stage 2 motor       (PD7)
4 - Red LED             (PB0)
5 - Green LED           (PB1)
6 - Blue RED            (PB2) 
7 - White LED           (PA7)
*/

/* J4 - LCD (HD44780 compatible)
1 - Gnd
2 - Vcc
3 - Contrast
4 - RS                  (PD3)
5 - Gnd
6 - E                   (PD6)
7 - Gnd
8 - Gnd
9 - Gnd
10 - Gnd
11 - D4                 (PC0)
12 - D5                 (PC1)
13 - D6                 (PC2)
14 - D7                 (PC3)
15 - LED +
16 - LED -
*/

/* J6 - UART
1 - Gnd
2 - RXD                 (PD0)
3 - TXD                 (PD1)
*/

/* J7 - Keypad
1 - R0                  (PC4)
2 - R1                  (PC5)
3 - R2                  (PC6)
4 - R3                  (PC7)
5 - C0                  (PB4)
6 - C1                  (PB5)
7 - C2                  (PB6)
8 - C3                  (PB7)
*/

/* J9 - Feed stage
1 - feed                (PD4)
2 - feed sense          (PA3)
*/

/*
Buzzer                  (PD2)
*/

// Functions definitions start

void USART_Transmit( char data );
unsigned char USART_Receive( void );
void USART_Flush( void );
void USART_text( char* text );
void wait_ms( uint16_t ms );
void wait_us( uint8_t us );
void lcd_command( uint8_t command );
void lcd_write_nibble( uint8_t data );
void lcd_init( void );
void put_data_to_lcd_buffer(unsigned char* data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer, uint8_t from_flash);
void put_line_to_lcd_buffer(unsigned char* text, uint8_t buffer, uint8_t row, uint8_t from_flash);
void disp_clear_buffer(uint8_t buffer);
uint8_t disp_swap_buffers(void);

// Functions definitions endl

//  Variables start
uint16_t metal_sense_buffer[10] = { 0 }, glass_sense_buffer[10] = { 0 }, color_sense_buffer[10] = { 0 }, metal_sense_value = 0, glass_sense_value = 0, color_sense_value = 0;
uint16_t system_counter = 0, sec_counter = 0;
uint8_t sec = 0, min = 0, hour = 0;
uint8_t adc_hold = 0, adc_read_count = 0;
uint8_t lcdColumns, lcdRows, currentCol, currentRow, lcdRowStart[4];

uint8_t compare_PWM0, compare_PWM1, compare_PWM2;
volatile uint8_t compbuff_PWM0, compbuff_PWM1, compbuff_PWM2;
uint8_t actual_num_key = '-', last_num_key = '-', actual_func_key = '-', last_func_key = '-';

unsigned char disp_linear_buff[160];

// one properties mem. cell, but I think two will be better, i.e. now need of writing without making buffer dirty
uint8_t disp_buffers_dirty = 0;        // buffer "dirty" bits, one means buffer updated and ready to display
// bits: 7-4: disp_linear_buff[79:159] = 7 - 4th line .. 4 - 1st line, 3-0: disp_linear_buff[0:79] = 3 - 4th line .. 0 - 1st line

uint8_t disp_state = DISP_STATE_NOP, disp_last_state = DISP_STATE_NOP, disp_operation = DISP_STATE_NOP;

uint8_t val_pwm0 = 0;

uint8_t disp_delay = 0, disp_temp_data = 0, disp_column_counter = 0, disp_active_buffer = DISP_FRONTBUFFER;
unsigned char *disp_buffer_pointer = NULL;

uint8_t menu_state = MENU_STATE_MAIN;
uint8_t selected_object = 0;            // bit 7 - blink state, bits 6:0 - selection

uint8_t blink_position = 0;             // bits 7 - blink state, 6:5 - row, 4:0 - column
uint8_t blink_conf = 0;                 // bits 7:4 - period [0 - off, 1-15 - val * 80 ms], 3:0 - length [1-16 characters]
unsigned char blink_buffer[16];
//  Variables end

//  Constans start
const unsigned char keypad_num0_keys[5] PROGMEM = "-12-3";
const unsigned char keypad_num1_keys[5] PROGMEM = "-45-6";
const unsigned char keypad_num2_keys[5] PROGMEM = "-78-9";
const unsigned char keypad_num3_keys[5] PROGMEM = "-*0-#";
const unsigned char keypad_func_keys[5] PROGMEM = "-ABCD";

const unsigned char menu0_line0_start[5] PROGMEM = "START";
const unsigned char menu0_line0_stop[4] PROGMEM = "STOP";
const unsigned char menu0_line1[14] PROGMEM = "select program";
const unsigned char menu0_line2[8] PROGMEM = "settings";
const unsigned char menu0_line3[9] PROGMEM = "power off";
//  Constans end