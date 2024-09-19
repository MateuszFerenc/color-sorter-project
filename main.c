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
        if ( system_counter % (2 << ( 7 + ( blink_conf >> 4)) ) == 0){
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
    static uint8_t pwm_counter = 0xFF, pwm_mask = 0;
    static uint8_t key_scan_row = 0;
    static uint8_t key_col_state[4] = { 0 };
    static uint8_t disp_delay = 0, disp_temp_data = 0, disp_column_counter = 0, disp_state = DISP_STATE_NOP, disp_last_state = DISP_STATE_NOP;

    
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

    if ( pwm_counter % 15 == 0 && pwm_counter != 0){       // OUTDATED: scan next row every ~1.5ms (refresh rate: 322Hz)
        key_col_state[key_scan_row] = ~(PINB >> 4) & 0x0F;
        PORTC |= 0xF0;
        PORTC ^= (1 << (4 + key_scan_row++));
        if ( key_scan_row > 3 ) {
            key_scan_row = 0;

            // TODO optimize
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
                unsigned buff_switch_offset = ( disp_active_buffer == DISP_BACKBUFFER )? 4 : 0;
                for (unsigned char buff_idx = buff_switch_offset; buff_idx < ( 4 + buff_switch_offset ); buff_idx++ ){
                    if ( ( disp_buffers_dirty >> buff_idx ) & 1 ){
                        disp_buffer_pointer = (unsigned char *)disp_linear_buff + (unsigned char)(buff_idx * 20);
                        disp_column_counter = 20;
                        disp_buffers_dirty &= ~( 1 << buff_idx );
                        disp_state = DISP_STATE_MCURSOR;
                        currentCol = 0;
                        currentRow = buff_idx % 4;
                        disp_temp_data = 0x80 | lcdRowStart[currentRow];
                        break;
                    }
                }
            } else {
                PIN_set(PORTD, PD3);
                disp_state = DISP_STATE_PUTHDATA;
                disp_temp_data = *((uint8_t *)disp_buffer_pointer++);
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
        disp_delay = 2;
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

        disp_delay = 2;
    }
}

ISR ( BADISR_vect ){}

unsigned char get_keypad_character( void ){
    if ( key_code == 0 )
        return 0;

    uint8_t temp = key_code & 0x1F;

    if (temp < 26)
        return pgm_read_byte(key_map[temp]);

    return '-';
}

void put_data_to_lcd_buffer( void * data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer, uint8_t from_flash){
    unsigned char position = (unsigned char)(buffer + (row * 20) + col), temp;
    for (unsigned char offset = 0; length > 0 ; length--, offset++ ){
        if ( from_flash )
            temp = pgm_read_byte(data + offset);
        else
            temp = *((unsigned char *)data + offset);
        if ( temp < 32)
            break;
        *( disp_linear_buff + position ) = temp;
        position++;
    }
    disp_buffers_dirty |= 1 << ( ( buffer / 20 ) + row );
}

uint8_t disp_swap_buffers( void ){
    if ( disp_active_buffer == DISP_FRONTBUFFER ){
        disp_active_buffer = DISP_BACKBUFFER;
        disp_buffers_dirty = 0xF0;
    } else {
        disp_active_buffer = DISP_FRONTBUFFER;
        disp_buffers_dirty = 0x0F;
    }
    return disp_active_buffer;
}

static uint8_t disp_active_buffer_get( void ){
    return disp_active_buffer;
}

void disp_clear_buffer(uint8_t buffer){
    for (unsigned char offset = (unsigned char)buffer; offset < (80 + buffer); offset++)
        *( disp_linear_buff + offset ) = (unsigned char)' ';
    if ( buffer == DISP_FRONTBUFFER )
        disp_buffers_dirty = 0x0F;
    else
        disp_buffers_dirty = 0xF0;
}

void put_one_char(unsigned char character, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer){
    unsigned char temp[20];
    if ( col + length > 20 )
        length -= col;
    for ( uint8_t chr = 0; chr < length; chr++)
        temp[chr] = character;
    put_data_to_lcd_buffer(&temp, length, row, col, buffer, 0);
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

static void lcd_init(void){
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
    
    lcdRowStart[0] = 0x00;
    lcdRowStart[1] = 0x40;
    lcdRowStart[2] = 20;            // Number of columns
    lcdRowStart[3] = 0x50 + 4;        // plus number of rows
    
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
    for ( unsigned char character = 0; character < length; character++){
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

static void fake_shutdown( void ){
    wdt_disable();
    disp_clear_buffer(DISP_FRONTBUFFER);
    put_data_to_lcd_buffer(&text_goodbye, 10, 2, 5, DISP_FRONTBUFFER, 1);
    wait_ms(150);
    blink_init(2, 5, 10, 2);
    wait_ms(150);
    compbuff_PWM0 = 53; // Move all servos to default position
    compbuff_PWM1 = 0;
    compbuff_PWM2 = 56;
    wait_ms(350);
    PIN_clear(PORTA, PA1);
    ADMUX = 0;          // Disable ADC
    ADCSRA = 0;
    wait_ms(200);
    blink_stop();
    disp_clear_buffer(DISP_FRONTBUFFER);
    wait_ms(50);
    cli();
    TCCR0 = 0;          // Disable timers in last resort
    TCCR2 = 0;
    
    wdt_enable(WDTO_15MS);    

    PORTC &= 0x0F;
    uint8_t button_hold = 200;

    for(;;) {
        wait_ms(5);
        wdt_reset();

        if ( ~(PINB >> 4) & 0x0F ){
            if (--button_hold == 0){
                for(;;);
            }
        } else {
            if ( button_hold < 20 )
                button_hold += 2;
            else
                button_hold = 200;
        }
    }
}

void print_settings_options( unsigned char buffer ){
    put_data_to_lcd_buffer("C.", 2, 3, 2, buffer, 0);
    put_data_to_lcd_buffer(&text_save, 4, 3, 4, buffer, 1);

    put_data_to_lcd_buffer("D.", 2, 3, 10, buffer, 0);
    put_data_to_lcd_buffer(&text_exit, 4, 3, 12, buffer, 1);
}

void check_eeprom_variables( void ) {
    if ( EEPROM_FSM_state == EEP_VAR_RAM2ROM ){
        if ( eeprom_is_ready() ){
            if ( EEPROM_variable_count < EEPROM_VARIABLES_COUNT ){
                eeprom_update_byte( pgm_read_word(&eeprom_variables_pointer_array[EEPROM_variable_count]), (uint8_t) *(uint8_t *) pgm_read_word(&setpoint_variables_pointer_array[EEPROM_variable_count]));
                EEPROM_variable_count++;
            } else {
                EEPROM_FSM_state = EEP_VAR_NOP;
                EEPROM_variable_count = 0;
            }
        }
    } else
    if ( EEPROM_FSM_state == EEP_VAR_ROM2RAM ){
        for (EEPROM_variable_count = 0; EEPROM_variable_count < EEPROM_VARIABLES_COUNT ; EEPROM_variable_count++){
            * (uint8_t *) pgm_read_word(&setpoint_variables_pointer_array[EEPROM_variable_count]) = eeprom_read_byte( pgm_read_word(&eeprom_variables_pointer_array[EEPROM_variable_count]));
        }
        EEPROM_FSM_state = EEP_VAR_NOP;
        EEPROM_variable_count = 0;
    }
}


void display_parameters( uint8_t max_amount, uint8_t offset, uint8_t max_offset, void * names_ptr, void * values_ptr, uint8_t names_len, uint8_t values_len, uint8_t values_pos, uint8_t values_src, uint8_t values_word_select){
    uint16_t temp_val;
    unsigned char val[5];

    if ( offset > ( max_amount - max_offset ) )
        offset = max_amount - max_offset;

    for (uint8_t i = 0; i < max_offset; i++){
        temp_val = 0;

        // clear line
        put_one_char(' ', 20, i, 0, disp_active_buffer);
        
        // Display parameter name
        put_data_to_lcd_buffer(&names_ptr[(i + offset) * names_len], names_len, i, 0, disp_active_buffer, 1);

        // Display ':' separator
        put_one_char(':', 1, i, names_len, disp_active_buffer);

        // Get 16-bit value from array in specified manner
        if ( (values_src & MASK_LOAD_PARAMETERS_MODE ) == __LOAD_PARAMETERS_DIRECT ){
            if ( values_src == LOAD_PARAMETERS_SRC_RAM_DIRECT )
                temp_val = ( values_word_select )? *((uint16_t *) values_ptr + i + offset ) : *((uint8_t *) values_ptr + i + offset );
            else
            if ( values_src == LOAD_PARAMETERS_SRC_ROM_DIRECT )
                temp_val = ( values_word_select )? pgm_read_word(values_ptr + (( i + offset ) << 1 )) : pgm_read_byte( (uint16_t) values_ptr + (( i + offset ) << 1 ));
            else
            if ( values_src == LOAD_PARAMETERS_SRC_EEP_DIRECT )
                temp_val = ( values_word_select )? eeprom_read_word(values_ptr + i + offset) : eeprom_read_byte(values_ptr + i + offset);
        } else
        if ( (values_src & MASK_LOAD_PARAMETERS_MODE ) == __LOAD_PARAMETERS_VIA_RAM ) {
            if ( values_src == LOAD_PARAMETERS_SRC_RAM_VIA_RAM_TABLE )
                temp_val = ( values_word_select )? (uint16_t) *(uint16_t *) &values_ptr[ i + offset ] : (uint8_t) *(uint16_t *) &values_ptr[ i + offset ];
            else
            if ( values_src == LOAD_PARAMETERS_SRC_ROM_VIA_RAM_TABLE )
                temp_val = ( values_word_select )? pgm_read_word( &values_ptr[ ( i + offset ) << 1 ] ) : pgm_read_byte( (uint16_t) &values_ptr[ ( i + offset ) << 1 ] );
            else
            if ( values_src == LOAD_PARAMETERS_SRC_EEP_VIA_RAM_TABLE )
                temp_val = ( values_word_select )? eeprom_read_word( &values_ptr[ i + offset ] ) : eeprom_read_byte( &values_ptr[ i + offset ] );
        } else
        if ( (values_src & MASK_LOAD_PARAMETERS_MODE ) == __LOAD_PARAMETERS_VIA_ROM ){
            if ( values_src == LOAD_PARAMETERS_SRC_RAM_VIA_ROM_TABLE )
                temp_val = ( values_word_select )? (uint16_t) *(uint16_t *) pgm_read_word( (uint16_t) &values_ptr[ ( i + offset ) << 1 ] ) : (uint8_t) *(uint8_t *) pgm_read_word( (uint16_t) &values_ptr[ ( i + offset ) << 1 ] );
            else
            if ( values_src == LOAD_PARAMETERS_SRC_ROM_VIA_ROM_TABLE )
                temp_val = ( values_word_select )? pgm_read_word( pgm_read_word( (uint16_t) &values_ptr[ ( i + offset ) << 1 ]) ) : pgm_read_byte( pgm_read_word( (uint16_t) &values_ptr[ ( i + offset ) << 1 ]) );
            else
            if ( values_src == LOAD_PARAMETERS_SRC_EEP_VIA_ROM_TABLE )
                temp_val = ( values_word_select )? eeprom_read_word( pgm_read_word( (uint16_t) &values_ptr[ i + offset ] ) ) : eeprom_read_byte( pgm_read_word( (uint16_t) &values_ptr[ i + offset ] ) );
        }

        // Convert value to ASCII
        utoa(temp_val, val, 10);

        // Display given value
        put_data_to_lcd_buffer(val, values_len, i, values_pos, disp_active_buffer, 0);
    }
}

void display_prompt(uint8_t type, void * text, uint8_t text_length, uint8_t buffer){
    blink_stop();
    disp_clear_buffer(buffer);
    put_one_char(255, 16, 0, 2, buffer);
    put_one_char(255, 16, 1, 2, buffer);
    put_one_char(255, 16, 2, 2, buffer);
    put_one_char(255, 16, 3, 2, buffer);

    if ( type == 1 )
        put_data_to_lcd_buffer(&text_warning, 7, 0, 6, buffer, 1);
    else
    if ( type == 2 )
        put_data_to_lcd_buffer(&text_err, 3, 0, 8, buffer, 1);
    else
        put_data_to_lcd_buffer(&text_info, 4, 0, 8, buffer, 1);

    put_data_to_lcd_buffer(text, text_length, 2, ((20 - text_length) / 2), buffer, 1);

    if ( disp_active_buffer_get() != buffer ){
        disp_swap_buffers();
    }
}

static void setup( void ){
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

    // Values are OUTDATED, frequency increased due to need of servo signal resolution increase
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

    // Timed sequence to disable JTAG interface
    MCUCSR = 0x80;
    MCUCSR = 0x80;

    compare_PWM0 = 0;
    compare_PWM1 = 0;
    compare_PWM2 = 0;

    lcd_init();
    sei();
}

int main( void ){
    wdt_disable();

    // Local variables start

    uint8_t actual_character = 0, last_character = 0;
    uint8_t keypad_press_wait = 4, keypad_hold_wait = 3;    

    // Stage State for FSM
    uint8_t stage1_state = STAGE_STATE_WAIT;
    //uint8_t stage2_state = STAGE_STATE_WAIT;
    uint8_t stage3_state = STAGE_STATE_WAIT;

    uint8_t stage1_config = 0;     // bits 1:0 [00 - accept all, 01 - accept greater, 10 - accept less, 11 - reject all]
    uint16_t stage1_level = 0;

    //uint8_t stage2_config = 0;
    //uint16_t stage2_level = 0;

    uint8_t stage3_config = 0;     // bits 1:0 [00 - accept all, 01 - logic AND between selections, 10 - logic OR between selections, 11 - reject all], bits pairs 3:2, 5:4, 6:7 rely to Red, Green, Blue [00 & 11 - ignore, 01 - accept greater, 10 - accept less]
    
    
    uint16_t stage3_red_level = 0, stage3_green_level = 0, stage3_blue_level = 0;

    uint16_t red_value = 0, green_value = 0, blue_value = 0;

    // Local Variables end

    setup();
    PIN_set(PORTA, PA1);        // Turn on the backlight

    disp_clear_buffer(DISP_FRONTBUFFER);

    // Display version and compilation date
    put_data_to_lcd_buffer(&sorter_version, 11, 0, 0, DISP_FRONTBUFFER, 1);
    put_data_to_lcd_buffer(&compilation_date, 11, 1, 0, DISP_FRONTBUFFER, 1);
    put_data_to_lcd_buffer(&dev0_name, 14, 3, 0, DISP_FRONTBUFFER, 1);
    
    // Quick color sense LEDs test
    PIN_clear(RED_LED_port, RED_LED_pin);
    PIN_clear(GREEN_LED_port, GREEN_LED_pin);
    PIN_clear(BLUE_LED_port, BLUE_LED_pin);
    PIN_clear(WHITE_LED_port, WHITE_LED_pin);

    // Quick servos test
    compbuff_PWM0 = 43;     // rightmost position
    compbuff_PWM1 = 0;
    compbuff_PWM2 = 46;

    wait_ms(300);

    compbuff_PWM0 = 65;     // leftmost position
    compbuff_PWM1 = 0;
    compbuff_PWM2 = 68;

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


    unsigned char val[20];

    EEPROM_FSM_state = EEP_VAR_ROM2RAM;          // Change EEPROM FSM state to load from EEPROM to RAM, i.e. initialize set-point values

    wdt_enable(WDTO_1S);

    for(;;){
        wdt_reset();

        check_eeprom_variables();

        wait_ms(5);

        actual_character = read_keypad();

        if (--keypad_press_wait > 0 && keypad_hold_wait == 3){      // Wait desired time to avoid miss clicks on keypad
            if ( actual_character != last_character ){
                keypad_press_wait = 4;
                last_character = actual_character;
            }
            actual_character = 0;
        } else {        // When proper key is registered, wait until released 
            keypad_press_wait = 4;
            if (--keypad_hold_wait > 0 ){
                if ( actual_character == last_character ){
                    keypad_hold_wait = 2;
                }
                actual_character = 0;
            } else
                keypad_hold_wait = 3;
        }

        if ( menu_state == MENU_STATE_DRIVE ){
            if ( --prompt_time == 0 ){
                disp_swap_buffers();
                menu_state = MENU_STATE_DRAW_MAIN;      // TODO goto last FSM state
            }
        } else
        if ( menu_state == MENU_STATE_DRAW_MAIN ){      // Draw main menu (0) screen
            disp_clear_buffer(DISP_FRONTBUFFER);

            if ( sorting_state == 0 ){
                put_data_to_lcd_buffer(&text_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
                blink_init(0, 7, 5, 3);     // period = 640ms
                menu_state = MENU_STATE_START;
            } else {
                put_data_to_lcd_buffer(&text_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);
                blink_init(0, 7, 4, 3);     // period = 640ms
                menu_state = MENU_STATE_S_ACTIVE;
            }
            
            put_data_to_lcd_buffer(&text_select_program, 14, 1, 3, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_configure, 9, 2, 6, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_power_off, 9, 3, 6, DISP_FRONTBUFFER, 1);
            
            
        } else
        if ( menu_state == MENU_STATE_START ){          // Handle "START" selection
            if ( actual_character == KEYPAD_ALT_DOWN ){
                blink_stop();
                blink_init(1, 3, 14, 3);
                menu_state = MENU_STATE_SELECT;
            } else 
            if ( actual_character == KEYPAD_ALT_SELECT ){
                if ( active_program ){
                    blink_stop();
                    menu_state = MENU_STATE_S_ACTIVE;
                    put_one_char(' ', 5, 0, 7, DISP_FRONTBUFFER);
                    put_data_to_lcd_buffer(&text_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);
                    blink_init(0, 7, 4, 3);
                    sorting_state = 1;
                } else {
                    display_prompt(1, &text_no_program, 11, DISP_BACKBUFFER);
                    menu_state = MENU_STATE_DRIVE;
                    prompt_time = 100;
                }
            } else
            if ( actual_character == KEYPAD_ALT_UP ){
                blink_stop();
                blink_init(3, 6, 9, 3);
                menu_state = MENU_STATE_PWROFF;
            }
        } else
        if ( menu_state == MENU_STATE_SELECT ){         // Handle "select program" election
            if ( actual_character == KEYPAD_ALT_DOWN ){
                blink_stop();
                blink_init(2, 6, 9, 3);
                menu_state = MENU_STATE_CONFIG;
            } else 
            if ( actual_character == KEYPAD_ALT_SELECT ){
                blink_stop();
                disp_clear_buffer(DISP_FRONTBUFFER);
                put_data_to_lcd_buffer(&text_select_hint, 20, 3, 0, DISP_FRONTBUFFER, 1);
                menu_state = MENU_STATE_SEL_ACTIVE;
                value_selected = 0;
            } else
            if ( actual_character == KEYPAD_ALT_UP ){
                blink_stop();
                blink_init(0, 7, 5, 3);
                menu_state = MENU_STATE_START;
            }
        } else
        if ( menu_state == MENU_STATE_CONFIG ){         // Handle "configure" selection
            if ( actual_character == KEYPAD_ALT_DOWN ){
                blink_stop();
                blink_init(3, 6, 9, 3);
                menu_state = MENU_STATE_PWROFF;
            } else 
            if ( actual_character == KEYPAD_ALT_SELECT ){
                menu_state = MENU_STATE_DRAW_CONFIG;
            } else
            if ( actual_character == KEYPAD_ALT_UP ){
                blink_stop();
                blink_init(1, 3, 14, 3);
                menu_state = MENU_STATE_SELECT;
            }
        } else
        if ( menu_state == MENU_STATE_PWROFF ){         // Handle "power off" selection
            if ( actual_character == KEYPAD_ALT_DOWN ){
                blink_stop();
                blink_init(0, 7, 5, 3);
                menu_state = MENU_STATE_START;
            } else 
            if ( actual_character == KEYPAD_ALT_SELECT ){
                blink_stop();
                fake_shutdown();
            } else
            if ( actual_character == KEYPAD_ALT_UP ){
                blink_stop();
                blink_init(2, 6, 9, 3);
                menu_state = MENU_STATE_CONFIG;
            }
        } else
        if ( menu_state == MENU_STATE_S_ACTIVE ){       // Handle toggle from START to STOP
            if ( actual_character == KEYPAD_ALT_DOWN ){
                blink_stop();
                blink_init(1, 3, 14, 3);
                menu_state = MENU_STATE_SELECT;
            } else 
            if ( actual_character == KEYPAD_ALT_SELECT ){
                blink_stop();
                menu_state = MENU_STATE_START;
                put_data_to_lcd_buffer(&text_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
                blink_init(0, 7, 5, 3);
                sorting_state = 0;
            } else
            if ( actual_character == KEYPAD_ALT_UP ){
                blink_stop();
                blink_init(3, 6, 9, 3);
                menu_state = MENU_STATE_PWROFF;
            }    
        } else
        if ( menu_state == MENU_STATE_SEL_ACTIVE ){     // Handle active "select program"
            if ( ~value_selected & 0x80 ){
                // Clear currently displayed line
                put_one_char(' ', 20, 0, 0, DISP_FRONTBUFFER);

                // Put program id
                utoa(value_selected, val, 10);
                put_data_to_lcd_buffer(val, 3, 0, 0, DISP_FRONTBUFFER, 0);

                // Separator
                put_one_char(':', 1, 0, 3, DISP_FRONTBUFFER);

                // Program name
                put_data_to_lcd_buffer(&program_content_array[1 + value_selected * 18  ], 7, 0, 5, DISP_FRONTBUFFER, 1); //+ ( value_selected )? 8 : 0

                //sprintf(val, "stg1 cfg: 0x%02x", pgm_read_byte(&program_content_array[ (9 + value_selected * 18) ]));
                //put_data_to_lcd_buffer(val, 14, 1, 0, DISP_FRONTBUFFER, 0);
                value_selected |= 0x80;
            }

            if ( actual_character == KEYPAD_ALT_SELECT ){
                active_program = value_selected & 0x7F;
                if ( active_program ){
                    uint16_t offset = active_program * 18; //+ 8
                    stage1_config = pgm_read_byte(&program_content_array[ (8 + offset) ]);
                    stage1_level = pgm_read_byte(&program_content_array[ (10 + offset) ]) << 8;
                    stage1_level |= pgm_read_byte(&program_content_array[ (11 + offset) ]);
                    stage3_config = pgm_read_byte(&program_content_array[ (9 + offset) ]);
                    stage3_red_level = pgm_read_byte(&program_content_array[ (12 + offset) ]) << 8;
                    stage3_red_level |= pgm_read_byte(&program_content_array[ (13 + offset) ]);
                    stage3_green_level = pgm_read_byte(&program_content_array[ (14 + offset) ]) << 8;
                    stage3_green_level |= pgm_read_byte(&program_content_array[ (15 + offset) ]);
                    stage3_blue_level = pgm_read_byte(&program_content_array[ (16 + offset) ]) << 8;
                    stage3_blue_level |= pgm_read_byte(&program_content_array[ (17 + offset) ]);
                }

                display_prompt(0, &text_program_loaded, 14, DISP_BACKBUFFER);
                menu_state = MENU_STATE_DRIVE;
                prompt_time = 100;
            } else
            if ( actual_character == KEYPAD_ALT_UP ){
                value_selected &= 0x7F;
                if ( value_selected == 0 ){
                    value_selected = 10;
                } else {
                    value_selected--;
                }
            } else
            if ( actual_character == KEYPAD_ALT_DOWN ){
                value_selected &= 0x7F;
                if ( ++value_selected > 10 ){
                    value_selected = 0;
                }
            } else
            if ( actual_character == KEYPAD_KEY_D ){
                menu_state = MENU_STATE_DRAW_MAIN;
                blink_stop();
            }
        } else
        if ( menu_state == MENU_STATE_DRAW_CONFIG ) {       // Draw "configure" screen
            blink_stop();
            menu_state = MENU_STATE_C_ACTIVE;
            disp_clear_buffer(DISP_FRONTBUFFER);

            put_data_to_lcd_buffer(&text_configure0, 10, 0, 0, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_program, 7, 0, 12, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_exit, 4, 1, 4, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_system, 6, 1, 10, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_status0, 7, 2, 0, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_runtime0, 8, 3, 0, DISP_FRONTBUFFER, 1);

            blink_init(0, 12, 7, 3);
        } else
        if ( menu_state == MENU_STATE_C_ACTIVE || menu_state == MENU_STATE_C_A_PROGRAM || menu_state == MENU_STATE_C_A_SYSTEM || 
            menu_state == MENU_STATE_C_A_EXIT ){               // Handle active "configure"

            // Display state of the machine
            if ( sorting_state & 0xF0 )
                put_data_to_lcd_buffer(&text_err, 3, 2, 8, DISP_FRONTBUFFER, 1);
            else
                put_data_to_lcd_buffer(&text_ok, 2, 2, 8, DISP_FRONTBUFFER, 1);

            put_one_char(',', 1, 2, 11, DISP_FRONTBUFFER);

            // Display state of sorting
            if ( sorting_state & 0x0F )
                put_data_to_lcd_buffer(&text_running, 7, 2, 13, DISP_FRONTBUFFER, 1);
            else
                put_data_to_lcd_buffer(&text_stopped, 7, 2, 13, DISP_FRONTBUFFER, 1);

            // Display runtime XD
            put_one_char('0' + (hour >> 4), 1, 3, 10, DISP_FRONTBUFFER);
            put_one_char('0' + (hour & 0x0F), 1, 3, 11, DISP_FRONTBUFFER);
            put_one_char(':', 1, 3, 12, DISP_FRONTBUFFER);
            put_one_char('0' + (min >> 4), 1, 3, 13, DISP_FRONTBUFFER);
            put_one_char('0' + (min & 0x0F), 1, 3, 14, DISP_FRONTBUFFER);
            put_one_char(':', 1, 3, 15, DISP_FRONTBUFFER);
            put_one_char('0' + (sec >> 4), 1, 3, 16, DISP_FRONTBUFFER);
            put_one_char('0' + (sec & 0x0F), 1, 3, 17, DISP_FRONTBUFFER);

            if ( menu_state == MENU_STATE_C_ACTIVE || menu_state == MENU_STATE_C_A_PROGRAM ){       // Handle "configure" -> "program" selection
                if ( actual_character == KEYPAD_ALT_UP ){
                    blink_stop();
                    blink_init(1, 4, 4, 3);
                    menu_state = MENU_STATE_C_A_EXIT;
                } else 
                if ( actual_character == KEYPAD_ALT_SELECT ){
                    menu_state = MENU_STATE_DRAW_C_PROG;
                } else
                if ( actual_character == KEYPAD_ALT_DOWN ){
                    blink_stop();
                    blink_init(1, 10, 6, 3);
                    menu_state = MENU_STATE_C_A_SYSTEM;
                }
            } else 
            if ( menu_state == MENU_STATE_C_A_SYSTEM ) {            // Handle "configure" -> "system" selection
                if ( actual_character == KEYPAD_ALT_UP ){
                    blink_stop();
                    blink_init(0, 12, 7, 3);
                    menu_state = MENU_STATE_C_A_PROGRAM;
                } else 
                if ( actual_character == KEYPAD_ALT_SELECT ){
                    blink_stop();
                    menu_state = MENU_STATE_C_A_SYSTEM_ACTIVE;
                    disp_clear_buffer(DISP_FRONTBUFFER);

                    parameter_disp_config = 0;
                    value_selected = 0;
                    print_settings_options(DISP_FRONTBUFFER);
                } else
                if ( actual_character == KEYPAD_ALT_DOWN ){
                    blink_stop();
                    blink_init(1, 4, 4, 3);
                    menu_state = MENU_STATE_C_A_EXIT;
                }
            } else 
            if ( menu_state == MENU_STATE_C_A_EXIT ) {              // Handle "configure" -> "exit" selection
                if ( actual_character == KEYPAD_ALT_UP ){
                    blink_stop();
                    blink_init(1, 10, 6, 3);
                    menu_state = MENU_STATE_C_A_SYSTEM;
                } else 
                if ( actual_character == KEYPAD_ALT_SELECT ){
                    blink_stop();
                    menu_state = MENU_STATE_DRAW_MAIN;
                } else
                if ( actual_character == KEYPAD_ALT_DOWN ){
                    blink_stop();
                    blink_init(0, 12, 7, 3);
                    menu_state = MENU_STATE_C_A_PROGRAM;
                }
            }
        } else 
        if ( menu_state == MENU_STATE_DRAW_C_PROG ) {               // Draw "configure" -> "program" screen
            blink_stop();
            menu_state = MENU_STATE_C_A_PROGRAM_ACTIVE;
            disp_clear_buffer(DISP_FRONTBUFFER);

            put_data_to_lcd_buffer(&text_parameters0, 11, 0, 0, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_view, 4, 1, 8, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_configure, 9, 2, 4, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_save, 4, 3, 4, DISP_FRONTBUFFER, 1);
            put_data_to_lcd_buffer(&text_exit, 4, 3, 12, DISP_FRONTBUFFER, 1);

            blink_init(1, 8, 4, 3);
        } else
        if ( menu_state == MENU_STATE_C_A_PROGRAM_ACTIVE || menu_state == MENU_STATE_C_A_PRG_A_view || menu_state == MENU_STATE_C_A_PRG_A_config ||
            menu_state == MENU_STATE_C_A_PRG_A_save || menu_state == MENU_STATE_C_A_PRG_A_exit ){          // Handle "configure" -> "program" selection
            if ( menu_state == MENU_STATE_C_A_PROGRAM_ACTIVE || menu_state == MENU_STATE_C_A_PRG_A_view ){      // Handle "configure" -> "program" - > "view" selection
                if ( actual_character == KEYPAD_ALT_UP ){
                    blink_stop();
                    blink_init(3, 12, 4, 3);
                    menu_state = MENU_STATE_C_A_PRG_A_exit;
                } else 
                if ( actual_character == KEYPAD_ALT_SELECT ){
                    blink_stop();
                    menu_state = MENU_STATE_C_A_PRG_A_view_A;
                    disp_clear_buffer(DISP_FRONTBUFFER);
                } else
                if ( actual_character == KEYPAD_ALT_DOWN ){
                    blink_stop();
                    blink_init(2, 4, 9, 3);
                    menu_state = MENU_STATE_C_A_PRG_A_config;
                }
            } else 
            if ( menu_state == MENU_STATE_C_A_PRG_A_config ) {          // Handle "configure" -> "program" - > "configure" selection
                if ( actual_character == KEYPAD_ALT_UP ){
                    blink_stop();
                    blink_init(1, 8, 4, 3);
                    menu_state = MENU_STATE_C_A_PRG_A_view;
                } else 
                if ( actual_character == KEYPAD_ALT_SELECT ){
                    blink_stop();
                    menu_state = MENU_STATE_C_A_PRG_A_config_A;
                    disp_clear_buffer(DISP_FRONTBUFFER);

                    sprintf(val, "%04x %04x %04x %04x", stage1_level, stage3_red_level, stage3_green_level, stage3_blue_level);
                    put_data_to_lcd_buffer(val, 19, 0, 0, DISP_FRONTBUFFER, 0);

                    val[0] = 0;
                    if ( stage1_config == 1 )
                        val[0] = 2;
                    else
                    if ( stage1_config == 2 )
                        val[0] = 3;
                    else
                    if ( stage1_config == 3 )
                        val[0] = 1;
                    put_data_to_lcd_buffer(&program_stage_config_display_names[ val[0] * 10 ], 10, 1, 0, DISP_FRONTBUFFER, 1);

                    val[0] = 0;
                    if ( (stage3_config & 0x03) == 1 )
                        val[0] = 4;
                    else
                    if ( (stage3_config & 0x03) == 2 )
                        val[0] = 5;
                    else
                    if ( (stage3_config & 0x03) == 3 )
                        val[0] = 1;
                    put_data_to_lcd_buffer(&program_stage_config_display_names[ val[0] * 10 ], 10, 2, 0, DISP_FRONTBUFFER, 1);

                    val[0] = 6;
                    if ( (stage3_config & 0x0C) == 4 )
                        val[0] = 2;
                    else
                    if ( (stage3_config & 0x0C) == 8 )
                        val[0] = 3;
                    put_data_to_lcd_buffer(&program_stage_config_display_names[ val[0] * 10 ], 8, 2, 12, DISP_FRONTBUFFER, 1);

                    val[0] = 6;
                    if ( (stage3_config & 0x30) == 16 )
                        val[0] = 2;
                    else
                    if ( (stage3_config & 0x30) == 32 )
                        val[0] = 3;
                    put_data_to_lcd_buffer(&program_stage_config_display_names[ val[0] * 10 ], 10, 3, 0, DISP_FRONTBUFFER, 1);

                    val[0] = 6;
                    if ( (stage3_config & 0xC0) == 64 )
                        val[0] = 2;
                    else
                    if ( (stage3_config & 0xC0) == 128 )
                        val[0] = 3;
                    put_data_to_lcd_buffer(&program_stage_config_display_names[ val[0] * 10 ], 8, 3, 12, DISP_FRONTBUFFER, 1);
                    //print_settings_options(DISP_FRONTBUFFER);
                } else
                if ( actual_character == KEYPAD_ALT_DOWN ){
                    blink_stop();
                    blink_init(3, 4, 4, 3);
                    menu_state = MENU_STATE_C_A_PRG_A_save;
                }
            } else 
            if ( menu_state == MENU_STATE_C_A_PRG_A_save ) {            // Handle "configure" -> "program" - > "save" selection
                if ( actual_character == KEYPAD_ALT_UP ){
                    blink_stop();
                    blink_init(2, 4, 9, 3);
                    menu_state = MENU_STATE_C_A_PRG_A_config;
                } else 
                if ( actual_character == KEYPAD_ALT_SELECT ){
                    blink_stop();
                    menu_state = MENU_STATE_DRAW_MAIN;
                } else
                if ( actual_character == KEYPAD_ALT_DOWN ){
                    blink_stop();
                    blink_init(3, 12, 4, 3);
                    menu_state = MENU_STATE_C_A_PRG_A_exit;
                }
            } else 
            if ( menu_state == MENU_STATE_C_A_PRG_A_exit ) {            // Handle "configure" -> "program" - > "exit" selection
                if ( actual_character == KEYPAD_ALT_UP ){
                    blink_stop();
                    blink_init(3, 4, 4, 3);
                    menu_state = MENU_STATE_C_A_PRG_A_save;
                } else 
                if ( actual_character == KEYPAD_ALT_SELECT ){
                    blink_stop();
                    menu_state = MENU_STATE_DRAW_CONFIG;
                } else
                if ( actual_character == KEYPAD_ALT_DOWN ){
                    blink_stop();
                    blink_init(1, 8, 4, 1);
                    menu_state = MENU_STATE_C_A_PRG_A_view;
                }
            }
        } else
        if ( menu_state == MENU_STATE_C_A_PRG_A_view_A ){               // Draw "configure" -> "program" -> "view" screen
            sprintf(val, "metal: %05u", metal_sense_value / 10);
            put_data_to_lcd_buffer(val, 12, 0, 0, DISP_FRONTBUFFER, 0);
            sprintf(val, "red: %05u", red_value);
            put_data_to_lcd_buffer(val, 10, 1, 0, DISP_FRONTBUFFER, 0);
            sprintf(val, "green: %05u", green_value);
            put_data_to_lcd_buffer(val, 12, 2, 0, DISP_FRONTBUFFER, 0);
            sprintf(val, "blue: %05u", blue_value);
            put_data_to_lcd_buffer(val, 11, 3, 0, DISP_FRONTBUFFER, 0);

            if ( actual_character ){
                menu_state = MENU_STATE_DRAW_C_PROG;
            }
        } else
        if ( menu_state == MENU_STATE_C_A_PRG_A_config_A ){     // Handle active "configure" -> "program" - > "view" selection
            if ( actual_character == KEYPAD_ALT_SELECT ){
                
            } else
            if ( actual_character == KEYPAD_ALT_UP ){

            } else
            if ( actual_character == KEYPAD_ALT_DOWN ){

            } else
            if ( actual_character == KEYPAD_KEY_C ){

            } else
            if ( actual_character == KEYPAD_KEY_D ){
                menu_state = MENU_STATE_DRAW_C_PROG;
            }
        } else
        if ( menu_state == MENU_STATE_C_A_SYSTEM_ACTIVE ){      // Handle active "configure" -> "system" selection
            if ( ~parameter_disp_config & 0x80 ){
                blink_stop();
                display_parameters(EEPROM_VARIABLES_COUNT, parameter_disp_config, 3,
                &system_parameters_display_names, &setpoint_variables_pointer_array, 13, 3, 15, LOAD_PARAMETERS_SRC_RAM_VIA_ROM_TABLE, BYTE_LOAD_DISPLAY_PARAMETERS);
                parameter_disp_config |= 0x80;
            }

            if ( ~value_selected & 0x80 ){
                blink_stop();
                blink_init(value_selected, 15, 3, 3);
                value_selected |= 0x80;
            }

            if ( actual_character == KEYPAD_ALT_SELECT ){
                
            } else
            if ( actual_character == KEYPAD_ALT_UP ){
                value_selected &= 0x7F;
                if ( ( value_selected & 0x7F ) == 0 ){
                    parameter_disp_config &= 0x7F;
                    if ( parameter_disp_config == 0 ){
                        parameter_disp_config = EEPROM_VARIABLES_COUNT - 3;
                        value_selected = 2;
                    } else
                        parameter_disp_config--; 
                } else {
                    value_selected--;
                }                 
            } else
            if ( actual_character == KEYPAD_ALT_DOWN ){
                value_selected &= 0x7F;
                if ( ( value_selected & 0x7F ) == 2 ){
                    parameter_disp_config &= 0x7F;
                    if ( ++parameter_disp_config > (EEPROM_VARIABLES_COUNT - 3) ){
                        value_selected = 0;
                        parameter_disp_config = 0;
                    } 
                } else {
                    value_selected++;
                }  
            } else
            if ( actual_character == KEYPAD_KEY_C ){

            } else
            if ( actual_character == KEYPAD_KEY_D ){
                menu_state = MENU_STATE_DRAW_CONFIG;
                blink_stop();
            }
        }

        if ( sorting_state != 0 ){     
            if ( stage1_state == STAGE_STATE_WAIT ){
                if ( PIN_is_low(PINA, PA4) ) {
                    stage1_in_wait = S_stage1_in_wait;
                    stage1_state = STAGE_STATE_IN;
                }
            } else
            if ( stage1_state == STAGE_STATE_IN ){
                if ( --stage1_in_wait == 0 ){
                    stage1_measure_hold = S_stage1_measure_hold;
                    stage1_state = STAGE_STATE_MEASURE;
                }
            } else
            if ( stage1_state == STAGE_STATE_MEASURE ){
                if ( stage1_measure_hold == 20 ){
                    adc_read_count = 0;
                }
                if ( --stage1_measure_hold == 0 ){
                    stage1_out_wait = S_stage1_out_wait;
                    stage1_state = STAGE_STATE_OUT;
                }
            } else
            if ( stage1_state == STAGE_STATE_OUT ){
                if ( stage3_state == STAGE_STATE_WAIT ) {
                    if ( (stage1_config & 0x03) == 0)
                        compbuff_PWM0 = S_stage1_servo_accept;
                    else if ( (stage1_config & 0x03) == 1)
                        compbuff_PWM0 = (metal_sense_value > stage1_level) ? S_stage1_servo_accept : S_stage1_servo_reject;
                    else if ( (stage1_config & 0x03) == 2)
                        compbuff_PWM0 = (metal_sense_value < stage1_level) ? S_stage1_servo_accept : S_stage1_servo_reject;
                    else
                        compbuff_PWM0 = S_stage1_servo_reject;

                    stage1_state = STAGE_STATE_DEFAULT;
                }
            } else
            if ( stage1_state == STAGE_STATE_DEFAULT ){
                if ( --stage1_out_wait == 0 ){
                    compbuff_PWM0 = S_stage1_servo_default;   
                    stage1_state = STAGE_STATE_WAIT; 
                }
            } 

            if ( stage3_state == STAGE_STATE_WAIT ){
                if ( PIN_is_low(PINA, PA6) ) {
                    stage3_in_wait = S_stage3_in_wait;
                    stage3_state = STAGE_STATE_IN;
                }
            } else
            if ( stage3_state == STAGE_STATE_IN ){
                if ( --stage3_in_wait == 0 ){
                    stage3_color_switch_hold = S_stage3_color_switch_hold;
                    stage3_measure_hold = S_stage3_measure_hold;
                    stage3_state = STAGE_STATE_MEASURE;
                    PIN_clear(RED_LED_port, RED_LED_pin);
                    stage3_config &= 0xF3;
                    stage3_config |= 4;
                    adc_read_count = 0;
                }
            } else
            if ( stage3_state == STAGE_STATE_MEASURE ){
                if ( --stage3_color_switch_hold == 0 ){
                    if ( (stage3_config & 0x0C) == 4 ){
                        stage3_config &= 0xF3;
                        stage3_config |= 8;
                        PIN_set(RED_LED_port, RED_LED_pin);
                        PIN_clear(GREEN_LED_port, GREEN_LED_pin);
                        adc_read_count = 0;
                        red_value = color_sense_value / 10;
                    } else 
                    if ( (stage3_config & 0x0C) == 8 ){
                        stage3_config &= 0xF3;
                        stage3_config |= 12;
                        PIN_set(GREEN_LED_port, GREEN_LED_pin);
                        PIN_clear(BLUE_LED_port, BLUE_LED_pin);
                        adc_read_count = 0;
                        green_value = color_sense_value / 10;
                    } else
                    if ( (stage3_config & 0x0C) == 12 ){
                        stage3_config &= 0xF3;
                        stage3_config |= 4;
                        PIN_set(BLUE_LED_port, BLUE_LED_pin);
                        PIN_clear(RED_LED_port, RED_LED_pin);
                        adc_read_count = 0;
                        blue_value = color_sense_value / 10;
                    }
                    stage3_color_switch_hold = S_stage3_color_switch_hold;
                }

                if ( --stage3_measure_hold == 0 ){
                    stage3_out_wait = S_stage3_out_wait;
                    stage3_state = STAGE_STATE_OUT;
                    PIN_set(RED_LED_port, RED_LED_pin);
                    PIN_set(GREEN_LED_port, GREEN_LED_pin);
                    PIN_set(BLUE_LED_port, BLUE_LED_pin);
                    PIN_set(WHITE_LED_port, WHITE_LED_pin);
                }
            } else
            if ( stage3_state == STAGE_STATE_OUT ){
                uint8_t temp = S_stage3_servo_accept, stage_base = stage3_config & 0x03, stage_red = (stage3_config & 0x0C) >> 2, stage_green = (stage3_config & 0x30) >> 4, stage_blue = (stage3_config & 0xC0) >> 6;

                if (stage_base == 1){
                    if ((stage_red == 1 && red_value < stage3_red_level) || 
                        (stage_red == 2 && red_value > stage3_red_level) || 
                        (stage_green == 1 && green_value < stage3_green_level) || 
                        (stage_green == 2 && green_value > stage3_green_level) || 
                        (stage_blue == 1 && blue_value < stage3_blue_level) || 
                        (stage_blue == 2 && blue_value > stage3_blue_level))
                        temp = S_stage3_servo_reject;
                } else
                if (stage_base == 2){
                    temp = S_stage3_servo_reject;
                    if ((stage_red == 1 && red_value > stage3_red_level) || 
                        (stage_red == 2 && red_value < stage3_red_level) || 
                        (stage_green == 1 && green_value > stage3_green_level) || 
                        (stage_green == 2 && green_value < stage3_green_level) || 
                        (stage_blue == 1 && blue_value > stage3_blue_level) || 
                        (stage_blue == 2 && blue_value < stage3_blue_level))
                        temp = S_stage3_servo_accept;
                } else 
                if (stage_base == 3)
                    temp = S_stage3_servo_reject;
                compbuff_PWM2 = temp;
                stage3_state = STAGE_STATE_DEFAULT;
                
            } else
            if ( stage3_state == STAGE_STATE_DEFAULT ){
                if ( --stage3_out_wait == 0 ){
                    compbuff_PWM2 = S_stage3_servo_default;   
                    stage3_state = STAGE_STATE_WAIT; 
                }
            }
        }
    }
}
