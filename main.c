#include "definitions.h"


// System interrupt (every 0.625us)
ISR(TIMER2_COMP_vect){
    static uint8_t selected_adc_channel = metal_sense_adc;

    if ( ADCSRA & ( 1 << ADIF ) ) {         // ADC conversion complete flag
        if ( adc_hold ) {
            if ( adc_hold > 3 ) {       // wait with starting new conversion for ca. 6 ms after changing mux
                ADCSRA |= ( 1 << ADSC ) | ( 1 << ADIF );
                adc_hold = 0;
            } else
                adc_hold++;
        } else {
            if ( sorting_state != 0 ){
                if ( selected_adc_channel == metal_sense_adc ){
                    selected_adc_channel = color_sense_adc;
                    metal_sense_value -= metal_sense_buffer[adc_read_count];
                    metal_sense_buffer[adc_read_count] = ADC;
                    metal_sense_value += metal_sense_buffer[adc_read_count];
                } else
                if ( selected_adc_channel == color_sense_adc ){
                    selected_adc_channel = metal_sense_adc;
                    color_sense_value -= color_sense_buffer[adc_read_count];
                    color_sense_buffer[adc_read_count] = ADC;
                    color_sense_value += color_sense_buffer[adc_read_count];
                    adc_read_count++;
                    if ( adc_read_count > 9){
                        adc_read_count = 0;
                    }  
                }
            }
            ADMUX = ( 1 << REFS0) | ( selected_adc_channel & 0x0F );        // switch ADC mux to other channel
            adc_hold = 1;       // hold conversion for stable readings
        }
    }
    //if ( system_counter % 256 == 0 && system_counter != 0 ){
    //    USART_text("Interrupt! (256 * 0.625ms = 160ms)\n\r");
    //}

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

    if ( blink_conf & 0xF0 ){
        if ( system_counter % (2 << ( 7 + ( blink_conf >> 4)) ) == 0 && system_counter != 0 ){
            blink_position ^= 0x80;
            if (blink_position & 0x80)
                put_data_to_lcd_buffer(&blink_buffer, blink_conf & 0x0F, (blink_position >> 5) & 3, blink_position & 0x0F, disp_active_buffer, 0);
            else
                put_one_char(255, blink_conf & 0x0F, (blink_position >> 5) & 3, blink_position & 0x0F, disp_active_buffer);
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

            key_code = (( key_col_state[1] & 8 ) * 4 ) | (( key_col_state[2] & 8 ) * 8) | (( key_col_state[3] & 8 ) * 12) | (( key_col_state[0] & 8 ) * 16);
            if ( key_col_state[1] & 7 )
                key_code |= (key_col_state[1] & 7);
            if ( key_col_state[2] & 7 )
                key_code |= (key_col_state[2] & 7) + 7;
            if ( key_col_state[3] & 7 )
                key_code |= (key_col_state[3] & 7) + 14;
            if ( key_col_state[0] & 7 )
                key_code |= (key_col_state[0] & 7) + 21;
            if ( ( (key_col_state[0] & 7) + (key_col_state[1] & 7) + (key_col_state[2] & 7) + (key_col_state[3] & 7) ) == 0 )
                key_code &= 0xF0;
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

unsigned char get_keypad_character( void ){
    if ( key_code == 0 )
        return 0;

    uint8_t temp = key_code & 0x1F;

    if ( temp < 8 ){
        if ( temp == 1 )
            return '1';
        if ( temp == 2 )
            return '2';
        if ( temp == 4 )
            return '3';
    }

    if ( temp < 15 ){
        if ( temp == 8 )
            return '4';
        if ( temp == 9 )
            return '5';
        if ( temp == 11 )
            return '6';
    }

    if ( temp < 22 ){
        if ( temp == 15 )
            return '7';
        if ( temp == 16 )
            return '8';
        if ( temp == 18 )
            return '9';
    }

    if ( temp < 25 ){
        if ( temp == 22 )
            return '*';
        if ( temp == 23 )
            return '0';
        if ( temp == 25 )
            return '#';
    }

    return '-';
}

void put_line_to_lcd_buffer(unsigned char* text, uint8_t buffer, uint8_t row, uint8_t from_flash){
    put_data_to_lcd_buffer(text, 20, 0, row, buffer, from_flash);
}

void put_data_to_lcd_buffer(unsigned char* data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer, uint8_t from_flash){
    unsigned char position = (unsigned char)(buffer + (row * 20) + col);
    for (unsigned char offset; length > 0 ; length--, offset++ ){
        if ( from_flash )
            *( disp_linear_buff + position ) = pgm_read_byte(data + offset);
        else {
            if ( *data < 32)
                continue;
            *( disp_linear_buff + position ) = *(data + offset);
        }
        position++;
    }
    disp_buffers_dirty |= 1 << ( ( buffer / 20 ) + row );
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
        *( disp_linear_buff + offset ) = (unsigned char)' ';
    wait_ms(1);
}

void put_one_char(unsigned char character, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer){
    unsigned char temp[20];
    if ( col + length < 20 ){
        for ( uint8_t chr = 0; chr < length; chr++)
            temp[chr] = character;
        put_data_to_lcd_buffer(&temp, length, row, col, buffer, 0);
    }
}


void lcd_command(uint8_t command){
    PIN_clear(PORTD, PD3);
    lcd_write_nibble(command >> 4);
    lcd_write_nibble(command);
}

void lcd_write_nibble(uint8_t data){
    PORTC &= 0xF0;
    PORTC |= (data & 0x0F);
    wait_us(200);

    PIN_clear(PORTD, PD6);
    PIN_set(PORTD, PD6);
    PIN_clear(PORTD, PD6);

    wait_us(200);
}

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
    wait_ms(120);
}

uint8_t blink_init(uint8_t row, uint8_t col, uint8_t length, uint8_t period){
    if ( col + length > 20)
        return 255;

    unsigned char offset = (unsigned char)(disp_active_buffer + (row * 20) + col);
    for ( unsigned char character; character < length; character++){
        *( blink_buffer + character ) = *( disp_linear_buff + offset + character );
    }

    blink_position = 0x80 | ( row & 3 ) << 5 | ( col & 15 );
    blink_conf = ( period & 15 ) << 4 | ( length & 15);

    return 0;
}

void blink_stop( void ){
    while ( (blink_position & 0x80) == 0 && (blink_conf & 0xF0))
        _NOP();
    blink_conf = 0;
}

uint8_t read_keypad( void ){
    uint8_t temp = key_code;
    key_code = 0;
    return temp;
}

void USART_Init( unsigned int ubrr ){
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
    DDRA = 0x82;
    DDRB = 0x0F;
    DDRC = 0xFF;
    DDRD = 0xFC;

    PORTA = 0x70;
    PORTB = 0xFF;
    PORTC = 0xF0;
    PORTD = 0x04;

    ADMUX = ( 1 << REFS0) | (metal_sense_adc & 0x1F);
    ADCSRA = ( 1 << ADEN ) | ( 1 << ADSC ) | (1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 );   // Adc single shot mode, clk/128

    // Software PWM timer (50Hz)
    // 50 Hz with 8bit resolution => update every 20ms and 256 samples => update every 155 us
    TCCR0 = ( 1 << WGM01 ) | ( 1 << CS01 );                     // Timer0 in CTC mode, clk/8 = 6444.8Hz
    OCR0 = 50;

    // System timer
    TCCR2 = ( 1 << WGM21 ) | ( 1 << CS21 ) | ( 1 << CS20);     // Timer2 in CTC mode, clk/32
    OCR2 = 143;                                  // Interrupt every 625us

    // Enable timers interrupts
    TIMSK = ( 1 << OCIE0 ) | ( 1 << OCIE2 );

    // Disable analog comparator
    ACSR = (1 << ACD);

    compare_PWM0 = 0;
    compare_PWM1 = 0;
    compare_PWM2 = 0;

    lcd_init();
    sei();
}

int main(void){
    setup();
    PIN_set(PORTA, PA1);        // Turn on the backlight

    disp_clear_buffer(DISP_FRONTBUFFER);

    put_data_to_lcd_buffer("Sorter v" TOSTRING(stable_version) "." TOSTRING(beta_version), 11, 0, 0, DISP_FRONTBUFFER, 0);
    put_data_to_lcd_buffer(__DATE__, 11, 1, 0, DISP_FRONTBUFFER, 0);
    
    // Quick color sense LEDs test
    PIN_clear(RED_LED_port, RED_LED_pin);
    PIN_clear(GREEN_LED_port, GREEN_LED_pin);
    PIN_clear(BLUE_LED_port, BLUE_LED_pin);
    PIN_clear(WHITE_LED_port, WHITE_LED_pin);

    // Quick servos test
    compbuff_PWM0 = 16;     // leftmost position
    compbuff_PWM1 = 0;
    compbuff_PWM2 = 16;

    wait_ms(300);

    compbuff_PWM0 = 92;     // rightmost position
    compbuff_PWM1 = 0;
    compbuff_PWM2 = 92;

    wait_ms(300);

    // Move to default position
    compbuff_PWM0 = 53;
    compbuff_PWM1 = 0;
    compbuff_PWM2 = 56;


    // Turn off the LEDs
    PIN_set(RED_LED_port, RED_LED_pin);
    PIN_set(GREEN_LED_port, GREEN_LED_pin);
    PIN_set(BLUE_LED_port, BLUE_LED_pin);
    PIN_set(WHITE_LED_port, WHITE_LED_pin);

    uint8_t actual_character = 0, last_character = 0;
    uint8_t selected_digit = 0;
    
    uint8_t keypad_press_wait = 4, keypad_hold_wait = 3;    

    uint8_t stage1_state = STAGE_STATE_WAIT;
    //uint8_t stage2_state = STAGE_STATE_WAIT;
    uint8_t stage3_state = STAGE_STATE_WAIT;


    uint8_t stage1_in_wait = 50, stage1_out_wait = 50, stage1_servo = 0;
    //uint8_t stage2_in_wait = 2, stage2_out_wait = 2, stage2_servo = 0;
    uint8_t stage3_in_wait = 50, stage3_out_wait = 50, stage3_servo = 0;

    for(;;){
        wait_ms(5);

        actual_character = read_keypad();

        //unsigned char val[10];

        if (--keypad_press_wait > 0 && keypad_hold_wait == 3){
            if ( actual_character != last_character ){
                keypad_press_wait = 4;
                last_character = actual_character;
            }
            actual_character = 0;
        } else {
            keypad_press_wait = 4;
            if (--keypad_hold_wait > 0 ){
                if ( actual_character == last_character ){
                    keypad_hold_wait = 2;
                }
                actual_character = 0;
            } else
                keypad_hold_wait = 3;
        }

        if ( menu_state == MENU_STATE_MAIN ){
            disp_operation = DISP_STATE_CLEAR;
            disp_clear_buffer(DISP_FRONTBUFFER);

            if ( sorting_state == 0 ){
                put_data_to_lcd_buffer(&menu0_line0_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
                blink_init(0, 7, 5, 3);     // period = 640ms
            } else {
                put_data_to_lcd_buffer(&menu0_line0_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);
                blink_init(0, 7, 4, 3);     // period = 640ms
            }
            
            put_data_to_lcd_buffer(&menu0_line1, 14, 1, 3, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&menu0_line2, 9, 2, 6, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&menu0_line3, 9, 3, 6, DISP_FRONTBUFFER, 1);
            
            menu_state = MENU_STATE_START;
        }

        if ( menu_state == MENU_STATE_START ){
            last_menu_state = MENU_STATE_START;
            if ( actual_character == 48 ){
                blink_stop();
                blink_init(1, 3, 14, 3);
                menu_state = MENU_STATE_SELECT;
            } else 
            if ( actual_character == 41 ){
                blink_stop();
                menu_state = MENU_STATE_START_ACTIVE;
                put_one_char(' ', 5, 0, 7, DISP_FRONTBUFFER);
                put_data_to_lcd_buffer(&menu0_line0_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);
                blink_init(0, 7, 4, 3);
                sorting_state = 1;
            } else
            if (  actual_character == 34 ){
                blink_stop();
                blink_init(3, 6, 9, 3);
                menu_state = MENU_STATE_PWROFF;
            }
            //if ( menu_state != MENU_STATE_START && menu_state != MENU_STATE_START_ACTIVE )
            //    put_data_to_lcd_buffer(&menu0_line0_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
        } else
        if ( menu_state == MENU_STATE_SELECT ){
            last_menu_state = MENU_STATE_SELECT;
            if ( actual_character == 48 ){
                blink_stop();
                blink_init(2, 6, 9, 3);
                menu_state = MENU_STATE_CONFIG;
            } else 
            if ( actual_character == 41 ){
                blink_stop();
                menu_state = MENU_STATE_MAIN;
                //menu_state = MENU_STATE_SELECT_ACTIVE;
                //disp_operation = DISP_STATE_CLEAR;
                //disp_clear_buffer(DISP_FRONTBUFFER);
            } else
            if (  actual_character == 34 ){
                blink_stop();
                blink_init(0, 7, 5, 3);
                menu_state = MENU_STATE_START;
            }
            //if ( menu_state != MENU_STATE_SELECT && menu_state != MENU_STATE_SELECT_ACTIVE )
            //    put_data_to_lcd_buffer(&menu0_line1, 14, 1, 3, DISP_FRONTBUFFER, 1);
        } else
        if ( menu_state == MENU_STATE_CONFIG ){
            last_menu_state = MENU_STATE_CONFIG;
            if ( actual_character == 48 ){
                blink_stop();
                blink_init(3, 6, 9, 3);
                menu_state = MENU_STATE_PWROFF;
            } else 
            if ( actual_character == 41 ){
                blink_stop();
                menu_state = MENU_STATE_CONFIG_ACTIVE;
                disp_operation = DISP_STATE_CLEAR;
                disp_clear_buffer(DISP_FRONTBUFFER);

                put_data_to_lcd_buffer(&menu2_line0_config, 10, 0, 0, DISP_FRONTBUFFER, 1);
                put_data_to_lcd_buffer(&menu2_line0, 7, 0, 12, DISP_FRONTBUFFER, 1);
                put_data_to_lcd_buffer(&menu2_line1_exit, 4, 1, 4, DISP_FRONTBUFFER, 1);
                put_data_to_lcd_buffer(&menu2_line1, 6, 1, 10, DISP_FRONTBUFFER, 1);
                put_data_to_lcd_buffer(&menu2_line2, 7, 2, 0, DISP_FRONTBUFFER, 1);

                blink_init(0, 12, 7, 3);
            } else
            if (  actual_character == 34 ){
                blink_stop();
                blink_init(1, 3, 14, 3);
                menu_state = MENU_STATE_SELECT;
            }
            //if ( menu_state != MENU_STATE_CONFIG && menu_state != MENU_STATE_CONFIG_ACTIVE )
            //    put_data_to_lcd_buffer(&menu0_line2, 9, 2, 6, DISP_FRONTBUFFER, 1);
        } else
        if ( menu_state == MENU_STATE_PWROFF ){
            last_menu_state = MENU_STATE_PWROFF;
            if ( actual_character == 48 ){
                blink_stop();
                blink_init(0, 7, 5, 3);
                menu_state = MENU_STATE_START;
            } else 
            if ( actual_character == 41 ){
                blink_stop();
                menu_state = MENU_STATE_PWROFF_ACTIVE;
                disp_operation = DISP_STATE_CLEAR;
                disp_clear_buffer(DISP_FRONTBUFFER);
                PIN_clear(PORTA, PA1);
            } else
            if (  actual_character == 34 ){
                blink_stop();
                blink_init(2, 6, 9, 3);
                menu_state = MENU_STATE_CONFIG;
            }
            //if ( menu_state != MENU_STATE_PWROFF && menu_state != MENU_STATE_PWROFF_ACTIVE )
            //    put_data_to_lcd_buffer(&menu0_line3, 9, 3, 6, DISP_FRONTBUFFER, 1);
        } else
        if ( menu_state == MENU_STATE_START_ACTIVE ){
            last_menu_state = MENU_STATE_START_ACTIVE;
            if ( actual_character == 48 ){
                blink_stop();
                blink_init(1, 3, 14, 3);
                menu_state = MENU_STATE_SELECT;
            } else 
            if ( actual_character == 41 ){
                blink_stop();
                menu_state = MENU_STATE_START;
                put_data_to_lcd_buffer(&menu0_line0_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
                blink_init(0, 7, 5, 3);
                sorting_state = 0;
            } else
            if (  actual_character == 34 ){
                blink_stop();
                blink_init(3, 6, 9, 3);
                menu_state = MENU_STATE_PWROFF;
            }
            //if ( menu_state != MENU_STATE_START_ACTIVE )
            //    put_data_to_lcd_buffer(&menu0_line0_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);       
        } else
        if ( menu_state == MENU_STATE_SELECT_ACTIVE ){
            last_menu_state = MENU_STATE_SELECT_ACTIVE;
            // if ( actual_character == 48 ){
            //     blink_stop();
            //     blink_init(1, 3, 14, 3);
            //     menu_state = MENU_STATE_SELECT;
            // } else 
            // if ( actual_character == 41 ){
            //     blink_stop();
            //     menu_state = MENU_STATE_START;
            //     put_data_to_lcd_buffer(&menu0_line0_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
            //     blink_init(0, 5, 4, 3);
            // } else
            // if (  actual_character == 34 ){
            //     blink_stop();
            //     blink_init(3, 6, 9, 3);
            //     menu_state = MENU_STATE_PWROFF;
            // }
            // if ( menu_state != MENU_STATE_START_ACTIVE )
            //     put_data_to_lcd_buffer(&menu0_line0_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);       
        } else
        if ( menu_state == MENU_STATE_CONFIG_ACTIVE || menu_state == MENU_STATE_CFG_PROGRAM || menu_state == MENU_STATE_CFG_SYSTEM || menu_state == MENU_STATE_CFG_EXIT ){
            last_menu_state = menu_state;
            
            if ( sorting_state & 0xF0 )
                put_data_to_lcd_buffer(&menu2_line2_err, 3, 2, 8, DISP_FRONTBUFFER, 1);
            else
                put_data_to_lcd_buffer(&menu2_line2_ok, 2, 2, 8, DISP_FRONTBUFFER, 1);

            put_one_char(',', 1, 2, 11, DISP_FRONTBUFFER);

            if ( sorting_state & 0x0F )
                put_data_to_lcd_buffer(&menu2_line2_running, 7, 2, 13, DISP_FRONTBUFFER, 1);
            else
                put_data_to_lcd_buffer(&menu2_line2_stopped, 7, 2, 13, DISP_FRONTBUFFER, 1);

            put_data_to_lcd_buffer(&menu2_line3, 8, 3, 0, DISP_FRONTBUFFER, 1);

            put_one_char('0' + (hour >> 4), 1, 3, 10, DISP_FRONTBUFFER);
            put_one_char('0' + (hour & 0x0F), 1, 3, 11, DISP_FRONTBUFFER);
            put_one_char(':', 1, 3, 12, DISP_FRONTBUFFER);
            put_one_char('0' + (min >> 4), 1, 3, 13, DISP_FRONTBUFFER);
            put_one_char('0' + (min & 0x0F), 1, 3, 14, DISP_FRONTBUFFER);
            put_one_char(':', 1, 3, 15, DISP_FRONTBUFFER);
            put_one_char('0' + (sec >> 4), 1, 3, 16, DISP_FRONTBUFFER);
            put_one_char('0' + (sec & 0x0F), 1, 3, 17, DISP_FRONTBUFFER);

            // put_data_to_lcd_buffer("TEST RUN", 8, 0, 0, DISP_FRONTBUFFER, 0);
            // sprintf(val, "metal = %05d", metal_sense_value / 10);
            // put_data_to_lcd_buffer(val, 13, 1, 0, DISP_FRONTBUFFER, 0);
            // sprintf(val, "color = %05d", color_sense_value / 10);
            // put_data_to_lcd_buffer(val, 13, 2, 0, DISP_FRONTBUFFER, 0);

            if ( menu_state == MENU_STATE_CONFIG_ACTIVE || menu_state == MENU_STATE_CFG_PROGRAM ){
                if ( actual_character == 34 ){
                    blink_stop();
                    blink_init(1, 4, 4, 3);
                    menu_state = MENU_STATE_CFG_EXIT;
                } else 
                if ( actual_character == 41 ){
                    blink_stop();
                    menu_state = MENU_STATE_CONFIG_ACTIVE;
                } else
                if (  actual_character == 48 ){
                    blink_stop();
                    blink_init(1, 10, 6, 3);
                    menu_state = MENU_STATE_CFG_SYSTEM;
                }
            } else 
            if ( menu_state == MENU_STATE_CFG_SYSTEM ) {
                if ( actual_character == 34 ){
                    blink_stop();
                    blink_init(0, 12, 7, 3);
                    menu_state = MENU_STATE_CFG_PROGRAM;
                } else 
                if ( actual_character == 41 ){
                    blink_stop();
                    menu_state = MENU_STATE_CONFIG_ACTIVE;
                } else
                if (  actual_character == 48 ){
                    blink_stop();
                    blink_init(1, 4, 4, 3);
                    menu_state = MENU_STATE_CFG_EXIT;
                }
            } else 
            if ( menu_state == MENU_STATE_CFG_EXIT ) {
                if ( actual_character == 34 ){
                    blink_stop();
                    blink_init(1, 10, 6, 3);
                    menu_state = MENU_STATE_CFG_SYSTEM;
                } else 
                if ( actual_character == 41 ){
                    blink_stop();
                    menu_state = MENU_STATE_MAIN;
                } else
                if (  actual_character == 48 ){
                    blink_stop();
                    blink_init(0, 12, 7, 3);
                    menu_state = MENU_STATE_CFG_PROGRAM;
                }
            }

            

            // if ( actual_character == 41 ){
            //      blink_stop();
            //      menu_state = MENU_STATE_MAIN;
            //      put_data_to_lcd_buffer(&menu0_line0_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
            //      blink_init(0, 5, 4, 3);
            // }
        }

        if ( sorting_state != 0 ){          
            if ( stage1_state == STAGE_STATE_WAIT ){
                if ( PIN_is_low(PINA, PA4) )
                    stage1_state = STAGE_STATE_IN;
            } else
            if ( stage1_state == STAGE_STATE_IN ){
                if ( --stage1_in_wait == 0 ){
                    stage1_in_wait = 50;
                    stage1_state = STAGE_STATE_MEASURE;
                }
            } else
            if ( stage1_state == STAGE_STATE_MEASURE ){
                compbuff_PWM0 = 65;
                stage1_state = STAGE_STATE_OUT;
            } else
            if ( stage1_state == STAGE_STATE_OUT ) {
                if ( stage3_state == STAGE_STATE_WAIT )
                    if ( --stage1_out_wait == 0 ){
                        stage1_out_wait = 50;
                        stage1_state = STAGE_STATE_WAIT;
                        compbuff_PWM0 = 53;
                    }
            }

            if ( stage3_state == STAGE_STATE_WAIT ){
                if ( PIN_is_low(PINA, PA6) )
                    stage3_state = STAGE_STATE_IN;
            } else
            if ( stage3_state == STAGE_STATE_IN ){
                if ( --stage3_in_wait == 0 ){
                    stage3_in_wait = 50;
                    stage3_state = STAGE_STATE_MEASURE;
                }
            } else
            if ( stage3_state == STAGE_STATE_MEASURE ){
                compbuff_PWM2 = 68;
                stage3_state = STAGE_STATE_OUT;
            } else
            if ( stage3_state == STAGE_STATE_OUT ) {
                if ( --stage3_out_wait == 0 ){
                    stage3_out_wait = 50;
                    stage3_state = STAGE_STATE_WAIT;
                    compbuff_PWM2 = 56;
                }
            }
        }
    }
}
