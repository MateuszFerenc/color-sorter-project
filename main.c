#include "definitions.h"

uint16_t metal_sense_buffer[10] = { 0 }, glass_sense_buffer[10] = { 0 }, color_sense_buffer[10] = { 0 }, metal_sense_value = 0, glass_sense_value = 0, color_sense_value = 0;
uint16_t system_counter = 0, sec_counter = 0;
uint8_t sec = 0, min = 0, hour = 0;
uint8_t adc_hold = 0, adc_read_count = 0;
uint8_t lcdColumns, lcdRows, currentCol, currentRow, lcdRowStart[4];

uint8_t compare_PWM0, compare_PWM1, compare_PWM2;
volatile uint8_t compbuff_PWM0, compbuff_PWM1, compbuff_PWM2;
uint8_t actual_num_key = '-', last_num_key = '-', actual_func_key = '-', last_func_key = '-';

unsigned char disp_0lane_0buff[20] = "";
unsigned char disp_1lane_0buff[20] = "";
unsigned char disp_2lane_0buff[20] = "";
unsigned char disp_3lane_0buff[20] = "";

unsigned char disp_0lane_1buff[20] = "";
unsigned char disp_1lane_1buff[20] = "";
unsigned char disp_2lane_1buff[20] = "";
unsigned char disp_3lane_1buff[20] = "";

// one properties mem. cell, but I think two will be better, i.e. now need of writing without making buffer dirty
uint8_t disp_buffers_properties = 0;        // buffer "dirty" bits, one means buffer updated and ready to display [ bits: 7-4 => dispX_1buff, 3-0 => dispX_0buff ]

uint8_t disp_state = DISP_STATE_NOP, disp_last_state = DISP_STATE_NOP;

uint8_t val_pwm0 = 0;

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
    if ( system_counter % 256 == 0 && system_counter != 0 ){
        USART_text("Interrupt! (256 * 0.625ms = 160ms)\n\r");
    }

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


const unsigned char keypad_num0_keys[] PROGMEM = "-12-3";
const unsigned char keypad_num1_keys[] PROGMEM = "-45-6";
const unsigned char keypad_num2_keys[] PROGMEM = "-78-9";
const unsigned char keypad_num3_keys[] PROGMEM = "-*0-#";
const unsigned char keypad_func_keys[] PROGMEM = "-ABCD";

uint8_t disp_delay = 0, disp_temp_data = 0;

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
            //actual_num_key = pgm_read_byte(&keypad_num_keys[ ( key_col_state[1] & 7 ) | ( key_col_state[2] & 7 ) * 2 | ( key_col_state[3] & 7 ) * 3 | ( key_col_state[0] & 7 ) * 4 ]);
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

    //if ( disp_state == DISP_STATE_NOP ){}

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
        disp_state = DISP_STATE_WAIT;

        PORTC &= 0xF0;
        PORTC |= ( disp_state == DISP_STATE_PUTHDATA ) ? ( disp_temp_data >> 4 ) : disp_temp_data & 0x0F;

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
                disp_state = DISP_STATE_PUTLDATA
            else
                disp_state = DISP_STATE_NOP;
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

// all display methods in FSM in interrupt forbids use of "async" use (i.e. data waiting in interrupt, but data changed in main thread => conflict and data loss)
// will be done in interrupt
void lcd_text(unsigned char* text){
    while(*text){
        lcd_data((unsigned char) *text++);
        currentCol++;
        if ( currentCol > lcdColumns ){
            currentCol = 0;
            currentRow++;
            if ( currentRow > lcdRows ){
                currentRow = 0;
            }
            lcd_move_cursor(currentCol, currentRow);
        }
    }
}

// will be done in interrupt
void lcd_data(uint8_t data){
    PIN_set(PORTD, PD3);
    lcd_write_nibble(data >> 4);
    lcd_write_nibble(data);
}

// will be done in interrupt
void lcd_command(uint8_t command){
    PIN_clear(PORTD, PD3);
    lcd_write_nibble(command >> 4);
    lcd_write_nibble(command);
}

// will be done in interrupt
void lcd_write_nibble(uint8_t data){
    PORTC &= 0xF0;
    PORTC |= (data & 0x0F);
    wait_us(50);

    PIN_clear(PORTD, PD6);
    PIN_set(PORTD, PD6);
    PIN_clear(PORTD, PD6);

    wait_us(50);
}

// will be done in interrupt
void lcd_move_cursor(uint8_t row, uint8_t col){
    lcd_command(0b10000000 | ( lcdRowStart[row] + col ));
    currentCol = col;
    currentRow = row;
}

// will be done in interrupt
void lcd_clear(void){
    lcd_command(0x01);
    wait_ms(1);
}

// will be done in interrupt
void lcd_home(void){
    currentCol = 0;
    currentRow = 0;
    lcd_command(0x02);
    wait_ms(1);
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
    
    lcd_clear();
    lcd_command(0x0C);
    lcd_command(0x06);
    
    lcdColumns = 20;
    lcdRows = 4;
    lcdRowStart[0] = 0x00;
    lcdRowStart[1] = 0x40;
    lcdRowStart[2] = lcdColumns;
    lcdRowStart[3] = 0x50 + lcdRows;
    
    lcd_home();
}

void USART_Init( unsigned int ubrr){
    UBRRH = (unsigned char)(ubrr>>8);
    UBRRL = (unsigned char)ubrr;
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
    //USART_Init(96);        // UART - 9600 Baudrate
    DDRA = ( 1 << PA7 );
    DDRB = ( 1 << PB0 ) | ( 1 << PB1 ) | ( 1 << PB2) | ( 1 << PB3 );
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
    lcd_text("Sorter v" TOSTRING(stable_version) "." TOSTRING(beta_version));
    lcd_move_cursor(1, 0);
    lcd_text(__DATE__);
    wait_ms(1000);
    lcd_clear();

    sei();
}

int main(void){
    setup();
    
    for(;;){
        wait_ms(100);

        lcd_move_cursor(3, 10);
        lcd_data((hour >> 4) + '0');
        lcd_data((hour & 0x0F) + '0');
        lcd_data(':');
        lcd_data((min >> 4) + '0');
        lcd_data((min & 0x0F) + '0');
        lcd_data(':');
        lcd_data((sec >> 4) + '0');
        lcd_move_cursor(3, 17);
        lcd_text(" ");
        lcd_move_cursor(3, 17);
        lcd_data((sec & 0x0F) + '0');

        lcd_move_cursor(0, 10);
        lcd_data(actual_num_key);
        lcd_data(actual_func_key);

        

        unsigned char val[10];
        sprintf(val, "val: %3d", val_pwm0);
        lcd_move_cursor(1, 10);
        lcd_text(val);
        sprintf(val, "PWM0: %3d", compbuff_PWM0);
        lcd_move_cursor(2, 10);
        lcd_text(val);
    }
}
