#define __AVR_ATmega16__
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 14745600UL
#include <util/delay.h>
#include <avr/cpufunc.h>
#include <string.h>
#include <stdio.h>

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

char selected_adc_channel = metal_sense_adc;

uint16_t metal_sense_buffer[10] = { 0 }, glass_sense_buffer[10] = { 0 }, color_sense_buffer[10] = { 0 }, metal_sense_value = 0, glass_sense_value = 0, color_sense_value = 0;
uint8_t adc_read_count = 0, int_cnt = 0;
uint16_t second_counter = 0;
uint8_t sec, min, hour, adc_hold = 0;

void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
void USART_Flush( void );
void USART_text(unsigned char* text);
void wait_ms(uint16_t ms);
void wait_us(uint8_t us);


ISR(TIMER0_OVF_vect){
    if ( ADCSRA & ( 1 << ADIF ) ) {         // ADC conversion complete flag
        if ( adc_hold ) {
            if ( adc_hold > 3 ) {       // wait with starting new conversion for ca. 6 ms after changing mux
                ADCSRA |= ( 1 << ADSC ) | ( 1 << ADIF );
                adc_hold = 0;
            } else
                adc_hold++;
        } else {
            if ( selected_adc_channel == metal_sense_adc ){
                selected_adc_channel = glass_sense_adc;
                metal_sense_value -= metal_sense_buffer[adc_read_count];
                metal_sense_buffer[adc_read_count] = ADC;
                metal_sense_value += metal_sense_buffer[adc_read_count];
            } else if ( selected_adc_channel == glass_sense_adc ){
                selected_adc_channel = color_sense_adc;
                glass_sense_value -= glass_sense_buffer[adc_read_count];
                glass_sense_buffer[adc_read_count] = ADC;
                glass_sense_value += glass_sense_buffer[adc_read_count];
            } else {
                selected_adc_channel = metal_sense_adc;
                color_sense_value -= color_sense_buffer[adc_read_count];
                color_sense_buffer[adc_read_count] = ADC;
                color_sense_value += color_sense_buffer[adc_read_count];
                adc_read_count++;
                if ( adc_read_count > 9){
                    adc_read_count = 0;
                }                
            }
            ADMUX = ( 1 << REFS0) | ( selected_adc_channel & 0x0F );        // switch ADC mux to other channel
            adc_hold = 1;       // hold conversion for stable readings
        }
    }
    if ( int_cnt > 200 ){
        int_cnt = 0;
        unsigned char message[15];

        USART_text("\e[2J");
        USART_text("\e[H");

        sprintf(message, "R = 0x%x\n\r", RED_value / 10);
        USART_text(message);        

        // sprintf(message, "G = 0x%x\n\r", GREEN_value / 10);
        // USART_text(message);

        // sprintf(message, "B = 0x%x\n\r", BLUE_value / 10);
        // USART_text(message);

        // unsigned char time[24];

        // sprintf(time, "\nRuntime: %d:%d:%d", hour, min, sec);
        // USART_text(time);
    }
    if ( second_counter == 488 ){
        second_counter = 0;
        sec++;
        if ( sec > 59 ) {
            sec = 0;
            min++;
            if ( min > 59 ) {
                min = 0;
                hour++;
                if ( hour > 23 )
                    hour = 0;
            }
        }
    }
    int_cnt++;
    second_counter++;
}

ISR(BADISR_vect){}




void lcd_data(uint8_t data){
    PORTD |= ( 1 << PD3 );
    lcd_write_nibble(data >> 4);
    lcd_write_nibble(data);
}

void lcd_command(uint8_t command){
    PORTD &= 0xFF & ~( 1 << PD3 );
    lcd_write_nibble(command >> 4);
    lcd_write_nibble(command);
}

void lcd_write_nibble(uint8_t data){
    PORTC = ( data >> 4 ) & 0x0F;

    PORTD &= 0xFF & ~( 1 << PD6 );
    PORTD |= ( 1 << PD6 );
    PORTD &= 0xFF & ~( 1 << PD6 );

    wait_us(200);
}

void lcd_init(void){
    for ( uint8_t enable_4b_mode = 0; enable_4b_mode < 3; enable_4b_mode++){
        lcd_write_nibble(0x03);
        wait_ms(5);
    }

    
}

void USART_Init( unsigned int ubrr){
    UBRRH = (unsigned char)(ubrr>>8);
    UBRRL = (unsigned char)ubrr;
    UCSRA = (1<<U2X);
    UCSRB = (1<<RXEN)|(1<<TXEN);
    UCSRC = (1<<USBS)|(3<<UCSZ0);
}

void USART_Transmit( unsigned char data ){
    while ( !( UCSRA & (1<<UDRE)) );
    UDR = data;
}

unsigned char USART_Receive( void ){
    while ( !(UCSRA & (1<<RXC)) );
    return UDR;
}

void USART_Flush( void ){
    unsigned char dummy;
    while ( UCSRA & (1<<RXC) ) dummy = UDR;
}

void USART_text(unsigned char* text){
    while(*text){
        USART_Transmit((unsigned char) *text++);
    }
}

void wait_ms(uint16_t ms){
    while(ms--)
        _delay_ms(1);
}

void wait_us(uint8_t us){
    while(us--)
        _delay_us(1);
}

void setup(void){
    cli();
    USART_Init(191);        // UART - 9600 Baudrate
    DDRA = ( 1 << PA7 );
    DDRB = ( 1 << PB0 ) | ( 1 << PB1 ) | ( 1 << PB2) | ( 1 << PB3 );
    DDRC = ( 1<< PC0 ) | ( 1  << PC1 ) | ( 1 << PC2 ) | ( 1 << PC3);
    DDRD = 0xF6;

    PORTA = 0x70;
    PORTB = 0xF0;
    PORTC = 0x00;
    PORTD = 0x00;

    DDRB |= ( 1 << PB4 );
    PORTB |= ( 1 << PB4 );
    DDRC = ( 0 << PC2 ) | ( 0 << PC3 ) | ( 0 << PC3 );

    ADMUX = ( 1 << REFS0) | (selected_adc_channel & 0x1F);
    ADCSRA = ( 1 << ADEN ) | ( 1 << ADSC ) | (1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 );   // Adc single shot mode, clk/128

    // Software PWM timer (50Hz)
    // 50 Hz with 8bit resolution => update every 20ms times 256 => update every 0.078125 ms
    TCCR0 = ( 1 << WGM01 ) | ( 1 << CS01 );                     // Timer0 in CTC mode, clk/8 = 12800Hz
    OCR0 = 71

    // System timer
    TCCR2 = ( 1 << WGM21 ) | ( 1 << CS21 ) | ( 1 << CS20);     // Timer2 in CTC mode, clk/64
    OCR2 = 115                                  // Interrupt every ~0.5ms

    // Enable timers interrupts
    TIMSK = ( 1 << OCIE0 ) | ( 1 << OCIE2 );

    // Disable analog comparator
    ACSR = (1 << ACD);
    sei();
}

int main(void){
    setup();
    // start first ADC conversion

    for(;;){
        _delay_ms(500);
        //USART_text("2137\n");
        PORTB ^= ( 1 << PB4 );
    }
}