#define __AVR_ATmega16__
#define F_CPU 14745600UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/cpufunc.h>
#include <avr/pgmspace.h>
#include <stdio.h>

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

#define     DISP_STATE_NOP              0
#define     DISP_STATE_WAIT             1
#define     DISP_STATE_PUTHDATA         2
#define     DISP_STATE_PUTLDATA         3
#define     DISP_STATE_ENTOGGLE         4
#define     DISP_STATE_CLEAR            5
#define     DISP_STATE_HOME             6
#define     DISP_STATE_ENWAIT           7


void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
void USART_Flush( void );
void USART_text(unsigned char* text);
void wait_ms(uint16_t ms);
void wait_us(uint8_t us);
void lcd_text(unsigned char* text);
void lcd_data(uint8_t data);
void lcd_command(uint8_t command);
void lcd_write_nibble(uint8_t data);
void lcd_move_cursor(uint8_t row, uint8_t col);
void lcd_init(void);

#define PWM0_pin        PB3
#define PWM0_port       PORTB
#define PWM1_pin        PD5
#define PWM1_port       PORTD
#define PWM2_pin        PD7
#define PWM2_port       PORTD

#define PWM0            0
#define PWM1            1
#define PWM2            2