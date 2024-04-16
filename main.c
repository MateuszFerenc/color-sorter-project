#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/cpufunc.h>
#include <string.h>
#include <stdio.h>

// color sense ADC channels
#define RED_adc 2
#define GREEN_adc 3
#define BLUE_adc 4

char selected_adc_channel = RED_adc;

uint16_t RED_buffer[10] = { 0 }, GREEN_buffer[10] = { 0 }, BLUE_buffer[10] = { 0 }, RED_value, GREEN_value, BLUE_value;
uint8_t adc_read_count = 0, int_cnt = 0;
uint16_t second_counter = 0;
uint8_t sec, min, hour;

void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
void USART_Flush( void );
void USART_text(unsigned char* text);


ISR(TIMER0_OVF_vect){
    if ( ADCSRA & ( 1 << ADIF ) ) {
        if ( selected_adc_channel == RED_adc ){
            selected_adc_channel = GREEN_adc;
            RED_value -= RED_buffer[adc_read_count];
            RED_buffer[adc_read_count] = ADC;
            RED_value += ADC;
            RED_value /= 10;
        } else if ( selected_adc_channel == GREEN_adc ){
            selected_adc_channel = BLUE_adc;
            GREEN_value += GREEN_buffer[adc_read_count];
            GREEN_buffer[adc_read_count] = ADC;
            GREEN_value += ADC;
            GREEN_value /= 10;
        } else {
            selected_adc_channel = RED_adc;
            BLUE_value = BLUE_buffer[adc_read_count];
            BLUE_buffer[adc_read_count] = ADC;
            BLUE_value += ADC;
            BLUE_value /= 10;
            adc_read_count++;
            if ( adc_read_count > 9){
                adc_read_count = 0;
            }                
        }
        ADMUX = ( 1 << REFS0) | ( selected_adc_channel & 0x0F );
        ADCSRA |= ( 1 << ADSC ) | ( 1 << ADIF );
    }
    if ( int_cnt > 200 ){
        int_cnt = 0;
        unsigned char message[15];

        USART_text("\e[2J");
        USART_text("\e[H");

        sprintf(message, "R = 0x%x\n\r", RED_value);
        USART_text(message);

        sprintf(message, "G = 0x%x\n\r", GREEN_value);
        USART_text(message);

        sprintf(message, "B = 0x%x\n\r", BLUE_value);
        USART_text(message);

        unsigned char time[24];

        sprintf(time, "sec: %d\n\r", second_counter);
        USART_text(time);

        sprintf(time, "\nRuntime: %d:%d:%d", hour, min, sec);
        USART_text(time);
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

void USART_Init( unsigned int ubrr){
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0A = (1<<U2X0);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit( unsigned char data ){
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = data;
}

unsigned char USART_Receive( void ){
    while ( !(UCSR0A & (1<<RXC0)) );
    return UDR0;
}

void USART_Flush( void ){
    unsigned char dummy;
    while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}

void USART_text(unsigned char* text){
    while(*text){
        USART_Transmit((unsigned char) *text++);
    }
}

void setup(void){
    USART_Init(13);
    DDRB |= ( 1 << PB4 );
    PORTB |= ( 1 << PB4 );
    DDRC = ( 1 << PC2 ) | ( 1 << PC3 ) | ( 1 << PC3 );

    DIDR0 = ( 1 << ADC4D ) | ( 1 << ADC3D ) | ( 1 << ADC2D);

    ADMUX = ( 1 << REFS0) | (selected_adc_channel & 0x0F);
    ADCSRA = ( 1 << ADEN ) | ( 1 << ADSC ) | ( 1 << ADPS2 ); //| ( 1 << ADPS0 );

    TCCR0B = ( 1 << CS01 );     // Setup Timer0 to overflow mode, interrupt on every 2.048 ms
    TIMSK0 = ( 1 << TOIE0 );
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

/*
typedef struct IO_B {
	unsigned int LCD_RS:1;				//PB0
	unsigned int LCD_EN:1;				//PB1
	unsigned int B2:1;		            //PB2
	unsigned int B3:1;			        //PB3
	unsigned int B4:1;		            //PB4
	unsigned int B5:1;			        //PB5
	unsigned int B6:1;			        //PB6
	unsigned int B7:1;			        //PB7
} IO_B;

#define _PORTB	(*( volatile IO_B*)&PORTB)

typedef struct IO_D {
	unsigned int D0:1;				//PD0
	unsigned int D1:1;				//PD1
	unsigned int LCD_D4:1;		    //PD2
	unsigned int LCD_D5:1;			//PD3
	unsigned int LCD_D6:1;		    //PD4
	unsigned int LCD_D7:1;			//PD5
	unsigned int D6:1;			    //PD6
	unsigned int D7:1;			    //PD7
} IO_D;

#define _PORTD	(*( volatile IO_D*)&PORTD)
*/