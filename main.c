#include "definitions.h"


// System interrupt (every 0.625us)
ISR(TIMER2_COMP_vect){
    static uint8_t selected_adc_channel = metal_sense_adc;
    static uint8_t selected_digit = 0;
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
    //if ( system_counter % 256 == 0 && system_counter != 0 ){
    //    USART_text("Interrupt! (256 * 0.625ms = 160ms)\n\r");
    //}

    if (system_counter % 16 == 0 && system_counter != 0){
        if ( actual_func_key == 'D' && last_func_key == 'D' ){
            selected_digit = 1;
        }

        if ( selected_digit > 0 ){
            if ( actual_num_key != last_num_key && actual_num_key != '-' ){
                val_pwm0 = val_pwm0 * 10 + ( actual_num_key - '0' );
                selected_digit++;
            }
            if ( selected_digit > 4 ){
                val_pwm0 = 0;
                selected_digit = 1;
            }
        }

        if ( actual_func_key == 'A' && last_func_key == 'A' && selected_digit > 0 ){
            compbuff_PWM0 = val_pwm0;
            val_pwm0 = 0;
            selected_digit = 0;
        }
    }
    // correction available
    // 2^16 = 65536 when 2^16 / 3200 = 20.48, which means that every counter reload lacks 1664 ticks
    // so when we correct modulo after missing cycle, we can achieve stable clock
    // like one run -> mod 1600 => reload counter (missing 1664 cycles) -> first mod 1664 then back to mod 1600, and repeat
    if ( sec_counter == 3200){
        sec_counter = 0;
        sec++;
        if ( ( sec & 0x0F ) > 9 ){
            sec += 0x10;
            sec &= 0xF0;
            if ( ( sec >> 4 ) > 5 ){
                sec = 0;
                min++;
                if ( ( min & 0x0F ) > 9 ){
                    min += 0x10;
                    min &= 0xF0;
                    if ( ( min >> 4 ) > 5 ){
                        min = 0;
                        hour++;
                        if ( ( hour & 0x0F ) > 9 ){
                            hour += 0x10;
                            hour &= 0xF0;
                            if ( ( ( hour >> 4 ) == 2 ) && ( ( hour & 0x0F ) > 4 ) ){
                                hour = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    system_counter++;
    sec_counter++;
}


// SoftPWM and Keypad scan interrupt (every 150us)
ISR(TIMER0_COMP_vect){
    static uint8_t pwm_counter = 0xFF;
    static uint8_t key_scan_row = 0;
    static uint8_t key_col_state[4] = { 0 };
    static uint8_t pwm_mask = 0;
    
    PIN_control(PWM0_port, PWM0_pin, (pwm_mask & 1));
    PIN_control(PWM1_port, PWM1_pin, (pwm_mask & 2) >> 1);
    PIN_control(PWM2_port, PWM2_pin, (pwm_mask & 4) >> 2);

    if ( ++pwm_counter == 0 ){
        compare_PWM0 = compbuff_PWM0;
        compare_PWM1 = compbuff_PWM1;
        compare_PWM2 = compbuff_PWM2;

        pwm_mask = ( 1 << PWM2 ) | ( 1 << PWM1 ) | ( 1 << PWM0 );
    }

    if ( pwm_counter == compare_PWM0)   pwm_mask &= ~( 1 << PWM0 );
    if ( pwm_counter == compare_PWM1)   pwm_mask &= ~( 1 << PWM1 );
    if ( pwm_counter == compare_PWM2)   pwm_mask &= ~( 1 << PWM2 );

    if ( pwm_counter % 10 == 0 && pwm_counter != 0){       // scan next row every ~1.5ms (refresh rate: 322Hz)
        key_col_state[key_scan_row] = ~(PINB >> 4) & 0x0F;
        PORTC |= 0xF0;
        PORTC ^= (1 << (4 + key_scan_row++));
        if ( key_scan_row > 3 ) {
            key_scan_row = 0;
            last_func_key = actual_func_key;
            last_num_key = actual_num_key;
            actual_func_key = pgm_read_byte(&keypad_func_keys[ ( key_col_state[1] >> 3 ) | ( key_col_state[2] >> 3 ) * 2 | ( key_col_state[3] >> 3 ) * 3 | ( key_col_state[0] >> 3 ) * 4 ]);
            if ( (key_col_state[1] & 7) && last_num_key == '-' )
                actual_num_key = pgm_read_byte(&keypad_num0_keys[key_col_state[1] & 7 ]);
            if ( (key_col_state[2] & 7) && last_num_key == '-' )
                actual_num_key = pgm_read_byte(&keypad_num1_keys[key_col_state[2] & 7 ]);
            if ( (key_col_state[3] & 7) && last_num_key == '-' )
                actual_num_key = pgm_read_byte(&keypad_num2_keys[key_col_state[3] & 7 ]);
            if ( (key_col_state[0] & 7) && last_num_key == '-' )
                actual_num_key = pgm_read_byte(&keypad_num3_keys[key_col_state[0] & 7 ]);
            if ( ( (key_col_state[0] & 7) + (key_col_state[1] & 7) + (key_col_state[2] & 7) + (key_col_state[3] & 7) ) == 0 )
                actual_num_key = '-';
        }
    }

    if ( disp_state == DISP_STATE_NOP ){
        if ( ( disp_buffers_dirty || disp_column_counter ) && disp_operation == DISP_STATE_NOP ){
            if ( disp_column_counter == 0 ){
                unsigned char temp_buff_addr = ( disp_active_buffer / 20 );
                for (unsigned char buff_idx = 0; buff_idx < 4; buff_idx++ ){
                    if ( ( disp_buffers_dirty >> buff_idx ) & 1 ){
                        disp_buffer_pointer = (unsigned char *)disp_linear_buff + (unsigned char)(buff_idx * 20);
                        disp_column_counter = 20;
                        disp_buffers_dirty &= ~( 1 << buff_idx );
                        disp_state = DISP_STATE_MCURSOR;
                        currentCol = 0;
                        currentRow = buff_idx;
                        disp_temp_data = 0x80 | lcdRowStart[currentRow];
                        break;
                    }
                }
            } else {
                PIN_set(PORTD, PD3);
                disp_state = DISP_STATE_PUTHDATA;
                disp_temp_data = *(disp_buffer_pointer++);
                disp_column_counter--;
            }
        } else {
            disp_state = disp_operation;
        }
    }

    if ( disp_state == DISP_STATE_MCURSOR ){
        disp_state = DISP_STATE_PUTHDATA;
        PIN_clear(PORTD, PD3);
        disp_temp_data = 0x80 | ( lcdRowStart[currentRow] + currentCol);      // lcd home command
    }

    if ( disp_state == DISP_STATE_HOME ){
        disp_state = DISP_STATE_PUTHDATA;
        currentCol = 0;
        currentRow = 0;
        PIN_clear(PORTD, PD3);
        disp_temp_data = 0x02;      // lcd home command
    }

    if ( disp_state == DISP_STATE_CLEAR ){
        disp_state = DISP_STATE_PUTHDATA;
        PIN_clear(PORTD, PD3);
        disp_temp_data = 0x01;      // lcd clear command
    }

    if ( disp_state == DISP_STATE_PUTHDATA || disp_state == DISP_STATE_PUTLDATA ){
        disp_last_state = disp_state;

        PORTC &= 0xF0;
        PORTC |= ( disp_state == DISP_STATE_PUTHDATA ) ? ( disp_temp_data >> 4 ) : ( disp_temp_data & 0x0F );

        disp_state = DISP_STATE_WAIT;
        disp_delay = 1;
    }

    if ( disp_state == DISP_STATE_WAIT ){
        if ( --disp_delay == 0 ){
            disp_state = DISP_STATE_ENTOGGLE;
        }
    }

    if ( disp_state == DISP_STATE_ENWAIT ){
        if ( --disp_delay == 0 ){
            if ( disp_last_state == DISP_STATE_PUTHDATA )
                disp_state = DISP_STATE_PUTLDATA;
            else {
                disp_state = DISP_STATE_NOP;
                disp_operation = DISP_STATE_NOP;
            }
        }
    }

    if ( disp_state == DISP_STATE_ENTOGGLE ){
        disp_state = DISP_STATE_ENWAIT;

        PIN_clear(PORTD, PD6);
        PIN_set(PORTD, PD6);
        PIN_clear(PORTD, PD6);

        disp_delay = 1;
    }
}

ISR(BADISR_vect){}

void put_line_to_lcd_buffer(char* text, uint8_t buffer, uint8_t row){
    put_data_to_lcd_buffer(text, 20, 0, row, buffer);
}

void put_data_to_lcd_buffer(char* data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer){
    uint8_t temp = 1 << ( ( buffer / 20 ) + row );
    for (unsigned char offset = (unsigned char)(buffer + (row * 20) + col); length > 0 ; length--, offset++ ){
        if ( *data < 32 )
            continue;
        *( disp_linear_buff + offset ) = *data++;
    }
    disp_buffers_dirty |= temp;
}

uint8_t disp_swap_buffers(void){
    if ( disp_active_buffer == DISP_FRONTBUFFER ){
        disp_active_buffer = DISP_BACKBUFFER;
        disp_buffers_dirty = 0xF0;
    } else {
        disp_active_buffer = DISP_FRONTBUFFER;
        disp_buffers_dirty = 0x0F;
    }
    return disp_active_buffer;
}

void disp_clear_buffer(uint8_t buffer){
    for (unsigned char offset = (unsigned char)buffer; offset < (80 + buffer); offset++)
        *( disp_linear_buff + offset ) = ' ';
}

// all display methods in FSM in interrupt forbids use of "async" use (i.e. data waiting in interrupt, but data changed in main thread => conflict and data loss)

// will be done in interrupt
void lcd_command(uint8_t command){
    //while ( disp_operation != DISP_STATE_NOP );
    PIN_clear(PORTD, PD3);
    lcd_write_nibble(command >> 4);
    lcd_write_nibble(command);
}

// will be done in interrupt
void lcd_write_nibble(uint8_t data){
    PORTC &= 0xF0;
    PORTC |= (data & 0x0F);
    wait_us(200);

    PIN_clear(PORTD, PD6);
    PIN_set(PORTD, PD6);
    PIN_clear(PORTD, PD6);

    wait_us(200);
}

// only initialization will be async, but display FSM should be forced NOP when this method is being executed
void lcd_init(void){
    for ( uint8_t enable_4b_mode = 0; enable_4b_mode < 3; enable_4b_mode++){
        lcd_write_nibble(0x03);
        wait_ms(5);
    }
    lcd_write_nibble(0x02);
    wait_ms(1);
    
    lcd_command(0x28);
    
    lcd_command(0x01);     // lcd clear
    wait_ms(2);

    lcd_command(0x0C);
    lcd_command(0x06);
    
    lcdColumns = 20;
    lcdRows = 4;
    lcdRowStart[0] = 0x00;
    lcdRowStart[1] = 0x40;
    lcdRowStart[2] = lcdColumns;
    lcdRowStart[3] = 0x50 + lcdRows;
    
    lcd_command(0x02);      // lcd home
    wait_ms(2);

    PIN_set(PORTD, PD3);
    lcd_write_nibble('O' >> 4);
    lcd_write_nibble('O');
    lcd_write_nibble('K' >> 4);
    lcd_write_nibble('K');
    wait_ms(80);
}

void USART_Init( unsigned int ubrr){
    UBRRH = (unsigned char)(ubrr>>8);
    UBRRL = (unsigned char)ubrr;
    UCSRB = (1<<RXEN)|(1<<TXEN);
    UCSRC = (1<<USBS)|(3<<UCSZ0);
}

void USART_Transmit( char data ){
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

void USART_text( char* text ){
    while(*text){
        USART_Transmit((unsigned char) *text++);
    }
}

void wait_ms( uint16_t ms ){
    while(ms--)
        _delay_ms(1);
}

void wait_us( uint8_t us ){
    while(us--)
        _delay_us(1);
}

void setup(void){
    cli();
    //USART_Init(96);        // UART - 9600 Baudrate
    DDRA = 0x80;
    DDRB = 0x0F;
    DDRC = 0xFF;
    DDRD = 0xFC;

    PORTA = 0x70;
    PORTB = 0xF0;
    PORTC = 0xF0;
    PORTD = 0x04;

    ADMUX = ( 1 << REFS0) | (metal_sense_adc & 0x1F);
    ADCSRA = ( 1 << ADEN ) | ( 1 << ADSC ) | (1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 );   // Adc single shot mode, clk/128

    // Software PWM timer (50Hz)
    // 50 Hz with 8bit resolution => update every 20ms and 256 samples => update every 155 us
    TCCR0 = ( 1 << WGM01 ) | ( 1 << CS01 );                     // Timer0 in CTC mode, clk/8 = 6444.8Hz
    OCR0 = 142;

    // System timer
    TCCR2 = ( 1 << WGM21 ) | ( 1 << CS21 ) | ( 1 << CS20);     // Timer2 in CTC mode, clk/32
    OCR2 = 143;                                  // Interrupt every 625us

    // Enable timers interrupts
    TIMSK = ( 1 << OCIE0 ) | ( 1 << OCIE2 );

    // Disable analog comparator
    ACSR = (1 << ACD);

    compare_PWM0 = 0x00;
    compbuff_PWM0 = 0x00;
    compare_PWM1 = 0x30;
    compbuff_PWM1 = 0x30;
    compare_PWM2 = 0xAA;
    compbuff_PWM2 = 0xAA;

    lcd_init();
    sei();
}

int main(void){
    setup();

    disp_clear_buffer(DISP_FRONTBUFFER);

    put_data_to_lcd_buffer("Sorter v" TOSTRING(stable_version) "." TOSTRING(beta_version), 11, 0, 0, DISP_FRONTBUFFER);
    put_data_to_lcd_buffer(__DATE__, 11, 1, 0, DISP_FRONTBUFFER);

    wait_ms(1000);

    disp_operation = DISP_STATE_CLEAR;
    disp_clear_buffer(DISP_FRONTBUFFER);

    //for test purposes of display buffer
    // wait_ms(1000);
    // for (unsigned char offset = 0; offset < 10 ; offset++ ){
    //     *( disp_linear_buff + offset ) = '0' + offset;
    // }

    // for (unsigned char offset = 0; offset < 10 ; offset++ ){
    //     *( disp_linear_buff + offset + 20) = 'a' + offset;
    // }

    // for (unsigned char offset = 0; offset < 10 ; offset++ ){
    //     *( disp_linear_buff + offset + 40 ) = 'A' + offset;
    // }

    // for (unsigned char offset = 0; offset < 10 ; offset++ ){
    //     *( disp_linear_buff + offset + 60) = ' ' + offset;
    // }

    // disp_buffers_dirty = 15;

    
    for(;;){
        wait_ms(500);

        unsigned char val[10];
        sprintf(val, "%c%c", actual_num_key, actual_func_key);
        put_data_to_lcd_buffer(val, 2, 0, 0, DISP_FRONTBUFFER);

        //put_data_to_lcd_buffer((hour >> 4) + '0' + (hour & 0x0F) + '0' + ':' + (min >> 4) + '0' + (min & 0x0F) + '0' + ':' + (sec >> 4) + '0' + (sec & 0x0F) + '0', 8, 1, 0, DISP_FRONTBUFFER);        
        sprintf(val, "%02x:%02x:%02x", hour, min, sec);
        put_data_to_lcd_buffer(val, 8, 1, 0, DISP_FRONTBUFFER);
        
        
        sprintf(val, "val: %3d", val_pwm0);
        put_data_to_lcd_buffer(val, 9, 2, 0, DISP_FRONTBUFFER);

        sprintf(val, "PWM0: %3d", compbuff_PWM0);
        put_data_to_lcd_buffer(val, 10, 3, 0, DISP_FRONTBUFFER);
    }
}
