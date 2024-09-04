diff --git a/main.c b/main.c
index c9878df..ac77eb9 100644
--- a/main.c
+++ b/main.c
@@ -88,10 +88,10 @@ ISR(TIMER2_COMP_vect){
 
 // SoftPWM and Keypad scan interrupt (every 150us)
 ISR(TIMER0_COMP_vect){
-    static uint8_t pwm_counter = 0xFF;
+    static uint8_t pwm_counter = 0xFF, pwm_mask = 0;
     static uint8_t key_scan_row = 0;
     static uint8_t key_col_state[4] = { 0 };
-    static uint8_t pwm_mask = 0;
+    static uint8_t disp_delay = 0, disp_temp_data = 0, disp_column_counter = 0, disp_state = DISP_STATE_NOP, disp_last_state = DISP_STATE_NOP;
 
     
     PIN_control(PWM0_port, PWM0_pin, (pwm_mask & 1));
@@ -117,6 +117,7 @@ ISR(TIMER0_COMP_vect){
         if ( key_scan_row > 3 ) {
             key_scan_row = 0;
 
+            // TODO optimize
             key_code = (( key_col_state[1] & 8 ) * 4 ) | (( key_col_state[2] & 8 ) * 8) | (( key_col_state[3] & 8 ) * 12) | (( key_col_state[0] & 8 ) * 16);
             if ( key_col_state[1] & 7 )
                 key_code |= (key_col_state[1] & 7);
@@ -134,7 +135,6 @@ ISR(TIMER0_COMP_vect){
     if ( disp_state == DISP_STATE_NOP ){
         if ( ( disp_buffers_dirty || disp_column_counter ) && disp_operation == DISP_STATE_NOP ){
             if ( disp_column_counter == 0 ){
-                unsigned char temp_buff_addr = ( disp_active_buffer / 20 );
                 for (unsigned char buff_idx = 0; buff_idx < 4; buff_idx++ ){
                     if ( ( disp_buffers_dirty >> buff_idx ) & 1 ){
                         disp_buffer_pointer = (unsigned char *)disp_linear_buff + (unsigned char)(buff_idx * 20);
@@ -150,7 +150,7 @@ ISR(TIMER0_COMP_vect){
             } else {
                 PIN_set(PORTD, PD3);
                 disp_state = DISP_STATE_PUTHDATA;
-                disp_temp_data = *(disp_buffer_pointer++);
+                disp_temp_data = *((uint8_t *)disp_buffer_pointer++);
                 disp_column_counter--;
             }
         } else {
@@ -185,7 +185,7 @@ ISR(TIMER0_COMP_vect){
         PORTC |= ( disp_state == DISP_STATE_PUTHDATA ) ? ( disp_temp_data >> 4 ) : ( disp_temp_data & 0x0F );
 
         disp_state = DISP_STATE_WAIT;
-        disp_delay = 1;
+        disp_delay = 2;
     }
 
     if ( disp_state == DISP_STATE_WAIT ){
@@ -212,11 +212,15 @@ ISR(TIMER0_COMP_vect){
         PIN_set(PORTD, PD6);
         PIN_clear(PORTD, PD6);
 
-        disp_delay = 1;
+        disp_delay = 2;
     }
 }
 
-ISR(BADISR_vect){}
+ISR ( EE_RDY_vect ){
+
+}
+
+ISR ( BADISR_vect ){}
 
 unsigned char get_keypad_character( void ){
     if ( key_code == 0 )
@@ -224,52 +228,15 @@ unsigned char get_keypad_character( void ){
 
     uint8_t temp = key_code & 0x1F;
 
-    if ( temp < 8 ){
-        if ( temp == 1 )
-            return '1';
-        if ( temp == 2 )
-            return '2';
-        if ( temp == 4 )
-            return '3';
-    }
-
-    if ( temp < 15 ){
-        if ( temp == 8 )
-            return '4';
-        if ( temp == 9 )
-            return '5';
-        if ( temp == 11 )
-            return '6';
-    }
-
-    if ( temp < 22 ){
-        if ( temp == 15 )
-            return '7';
-        if ( temp == 16 )
-            return '8';
-        if ( temp == 18 )
-            return '9';
-    }
-
-    if ( temp < 25 ){
-        if ( temp == 22 )
-            return '*';
-        if ( temp == 23 )
-            return '0';
-        if ( temp == 25 )
-            return '#';
-    }
+    if (temp < 26)
+        return pgm_read_byte(key_map[temp]);
 
     return '-';
 }
 
-void put_line_to_lcd_buffer(unsigned char* text, uint8_t buffer, uint8_t row, uint8_t from_flash){
-    put_data_to_lcd_buffer(text, 20, 0, row, buffer, from_flash);
-}
-
 void put_data_to_lcd_buffer(unsigned char* data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer, uint8_t from_flash){
     unsigned char position = (unsigned char)(buffer + (row * 20) + col);
-    for (unsigned char offset; length > 0 ; length--, offset++ ){
+    for (unsigned char offset = 0; length > 0 ; length--, offset++ ){
         if ( from_flash )
             *( disp_linear_buff + position ) = pgm_read_byte(data + offset);
         else {
@@ -296,7 +263,10 @@ uint8_t disp_swap_buffers(void){
 void disp_clear_buffer(uint8_t buffer){
     for (unsigned char offset = (unsigned char)buffer; offset < (80 + buffer); offset++)
         *( disp_linear_buff + offset ) = (unsigned char)' ';
-    wait_ms(1);
+    if ( buffer == DISP_FRONTBUFFER )
+        disp_buffers_dirty = 0x0F;
+    else
+        disp_buffers_dirty = 0xF0;
 }
 
 void put_one_char(unsigned char character, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer){
@@ -343,12 +313,10 @@ void lcd_init(void){
     lcd_command(0x0C);
     lcd_command(0x06);
     
-    lcdColumns = 20;
-    lcdRows = 4;
     lcdRowStart[0] = 0x00;
     lcdRowStart[1] = 0x40;
-    lcdRowStart[2] = lcdColumns;
-    lcdRowStart[3] = 0x50 + lcdRows;
+    lcdRowStart[2] = 20;            // Number of columns
+    lcdRowStart[3] = 0x50 + 4;        // plus number of rows
     
     lcd_command(0x02);      // lcd home
     wait_ms(2);
@@ -366,7 +334,7 @@ uint8_t blink_init(uint8_t row, uint8_t col, uint8_t length, uint8_t period){
         return 255;
 
     unsigned char offset = (unsigned char)(disp_active_buffer + (row * 20) + col);
-    for ( unsigned char character; character < length; character++){
+    for ( unsigned char character = 0; character < length; character++){
         *( blink_buffer + character ) = *( disp_linear_buff + offset + character );
     }
 
@@ -427,27 +395,26 @@ void wait_us( uint8_t us ){
 }
 
 void fake_shutdown( void ){
-    disp_operation = DISP_STATE_CLEAR;
     disp_clear_buffer(DISP_FRONTBUFFER);
-    wait_ms(300);
     put_data_to_lcd_buffer(&text_goodbye, 10, 2, 5, DISP_FRONTBUFFER, 1);
     wait_ms(150);
     blink_init(2, 5, 10, 2);
     wait_ms(150);
-    compbuff_PWM0 = 53;
+    compbuff_PWM0 = 53; // Move all servos to default position
     compbuff_PWM1 = 0;
     compbuff_PWM2 = 56;
     wait_ms(350);
     PIN_clear(PORTA, PA1);
-    ADMUX = 0;
+    ADMUX = 0;          // Disable ADC
     ADCSRA = 0;
     wait_ms(200);
     blink_stop();
-    disp_operation = DISP_STATE_CLEAR;
+    disp_clear_buffer(DISP_FRONTBUFFER);
     wait_ms(50);
     cli();
-    TCCR0 = 0;
+    TCCR0 = 0;          // Disable timers in last resort
     TCCR2 = 0;
+    //SFIOR |= 0x04;      // Disable all Pull-ups
     
 
     PORTC &= 0x0F;
@@ -456,7 +423,7 @@ void fake_shutdown( void ){
     for(;;) {
         wait_ms(5);
 
-        if ( ~(PINB >> 4) & 0x0F ){
+        if ( !(PINB >> 4) & 0x0F ){
             if (--button_hold == 0){
                 wdt_enable(WDTO_15MS);
                 for(;;);
@@ -464,11 +431,30 @@ void fake_shutdown( void ){
         } else {
             if ( button_hold < 40 )
                 button_hold += 10;
+            else
+                button_hold = 200;
         }
     }
 }
 
-void setup(void){
+void print_settings_options( unsigned char buffer ){
+    put_data_to_lcd_buffer("1.", 2, 3, 2, buffer, 0);
+    put_data_to_lcd_buffer(&text_save, 4, 3, 4, buffer, 1);
+
+    put_data_to_lcd_buffer("2.", 2, 3, 10, buffer, 0);
+    put_data_to_lcd_buffer(&text_exit, 4, 3, 12, buffer, 1);
+}
+
+void check_eeprom_variables( void ) {
+    if ( EEPROM_variables_config == 0x0F ){
+        //eeprom_write_byte();
+    } else
+    if ( EEPROM_variables_config == 0xF0 ){
+
+    }
+}
+
+void setup( void ){
     cli();
     //USART_Init(96);        // UART - 9600 Baudrate
     DDRA = 0x82;
@@ -499,6 +485,10 @@ void setup(void){
     // Disable analog comparator
     ACSR = (1 << ACD);
 
+    // Timed sequence to disable JTAG interface
+    MCUCSR = 0x80;
+    MCUCSR = 0x80;
+
     compare_PWM0 = 0;
     compare_PWM1 = 0;
     compare_PWM2 = 0;
@@ -507,14 +497,41 @@ void setup(void){
     sei();
 }
 
-int main(void){
+int main( void ){
+    // Local variables start
+
+    uint8_t actual_character = 0, last_character = 0;
+
+    uint8_t keypad_press_wait = 4, keypad_hold_wait = 3;    
+
+    // Stage State for FSM
+    uint8_t stage1_state = STAGE_STATE_WAIT;
+    //uint8_t stage2_state = STAGE_STATE_WAIT;
+    uint8_t stage3_state = STAGE_STATE_WAIT;
+
+    uint8_t stage_1_config = 0;     // bits 1:0 [00 - accept all, 01 - accept greater, 10 - accept less, 11 - reject all]
+    uint16_t stage1_level = 0;
+
+    //uint8_t stage2_config = 0;
+    //uint16_t stage2_level = 0;
+
+    uint8_t stage3_config = 0;     // bits 1:0 [00 - accept all, 01 - accept greater, 10 - accept less, 11 - reject all]
+    // 3:2 [ 0 - no color, 01 - red, 10 - green, 11 - blue ]
+    
+    uint16_t stage3_level = 0;
+
+    uint16_t red_value = 0, green_value = 0, blue_value = 0;
+
+    // Local Variables end
+
+
     setup();
     PIN_set(PORTA, PA1);        // Turn on the backlight
 
     disp_clear_buffer(DISP_FRONTBUFFER);
 
-    put_data_to_lcd_buffer("Sorter v" TOSTRING(stable_version) "." TOSTRING(beta_version), 11, 0, 0, DISP_FRONTBUFFER, 0);
-    put_data_to_lcd_buffer(__DATE__, 11, 1, 0, DISP_FRONTBUFFER, 0);
+    put_data_to_lcd_buffer(&sorter_version, 11, 0, 0, DISP_FRONTBUFFER, 1);
+    put_data_to_lcd_buffer(&compilation_date, 11, 1, 0, DISP_FRONTBUFFER, 1);
     
     // Quick color sense LEDs test
     PIN_clear(RED_LED_port, RED_LED_pin);
@@ -523,15 +540,15 @@ int main(void){
     PIN_clear(WHITE_LED_port, WHITE_LED_pin);
 
     // Quick servos test
-    compbuff_PWM0 = 16;     // leftmost position
+    compbuff_PWM0 = 43;     // rightmost position
     compbuff_PWM1 = 0;
-    compbuff_PWM2 = 16;
+    compbuff_PWM2 = 46;
 
     wait_ms(300);
 
-    compbuff_PWM0 = 92;     // rightmost position
+    compbuff_PWM0 = 65;     // leftmost position
     compbuff_PWM1 = 0;
-    compbuff_PWM2 = 92;
+    compbuff_PWM2 = 68;
 
     wait_ms(300);
 
@@ -547,43 +564,22 @@ int main(void){
     PIN_set(BLUE_LED_port, BLUE_LED_pin);
     PIN_set(WHITE_LED_port, WHITE_LED_pin);
 
-    uint8_t actual_character = 0, last_character = 0;
-    uint8_t selected_digit = 0;
-    
-    uint8_t keypad_press_wait = 4, keypad_hold_wait = 3;    
-
-    uint8_t stage1_state = STAGE_STATE_WAIT;
-    //uint8_t stage2_state = STAGE_STATE_WAIT;
-    uint8_t stage3_state = STAGE_STATE_WAIT;
-
-
-    uint8_t stage1_in_wait = 50, stage1_measure_hold = 20, stage1_out_wait = 50, stage1_servo = 0;
-    //uint8_t stage2_in_wait = 2, stage2_measure_hold = 100, stage2_out_wait = 2, stage2_servo = 0;
-    uint8_t stage3_in_wait = 50, stage3_measure_hold = 20, stage3_out_wait = 50, stage3_servo = 0;
-
-    uint8_t stage_1_config = 0;
-    uint16_t stage1_level = 0;
-
-    //uint8_t stage2_config = 0;
-    //uint16_t stage2_level = 0;
-
-    uint8_t stage3_config = 0;
-    uint16_t stage3_level = 0;
+    unsigned char val[20];
 
     for(;;){
+        check_eeprom_variables();
+
         wait_ms(5);
 
         actual_character = read_keypad();
 
-        //unsigned char val[10];
-
-        if (--keypad_press_wait > 0 && keypad_hold_wait == 3){
+        if (--keypad_press_wait > 0 && keypad_hold_wait == 3){      // Wait desired time to avoid miss clicks on keypad
             if ( actual_character != last_character ){
                 keypad_press_wait = 4;
                 last_character = actual_character;
             }
             actual_character = 0;
-        } else {
+        } else {        // When proper key is registered, wait until released 
             keypad_press_wait = 4;
             if (--keypad_hold_wait > 0 ){
                 if ( actual_character == last_character ){
@@ -594,27 +590,24 @@ int main(void){
                 keypad_hold_wait = 3;
         }
 
-        if ( menu_state == MENU_STATE_MAIN ){
-            disp_operation = DISP_STATE_CLEAR;
+        if ( menu_state == MENU_STATE_DRAW_MAIN ){      // Draw main menu (0) screen
             disp_clear_buffer(DISP_FRONTBUFFER);
 
             if ( sorting_state == 0 ){
-                put_data_to_lcd_buffer(&menu0_line0_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
+                put_data_to_lcd_buffer(&text_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
                 blink_init(0, 7, 5, 3);     // period = 640ms
             } else {
-                put_data_to_lcd_buffer(&menu0_line0_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);
+                put_data_to_lcd_buffer(&text_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);
                 blink_init(0, 7, 4, 3);     // period = 640ms
             }
             
-            put_data_to_lcd_buffer(&menu0_line1, 14, 1, 3, DISP_FRONTBUFFER, 1);
-            put_data_to_lcd_buffer(&menu0_line2, 9, 2, 6, DISP_FRONTBUFFER, 1);
-            put_data_to_lcd_buffer(&menu0_line3, 9, 3, 6, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_select_program, 14, 1, 3, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_configure, 9, 2, 6, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_power_off, 9, 3, 6, DISP_FRONTBUFFER, 1);
             
             menu_state = MENU_STATE_START;
-        }
-
-        if ( menu_state == MENU_STATE_START ){
-            last_menu_state = MENU_STATE_START;
+        } else
+        if ( menu_state == MENU_STATE_START ){          // Handle "START" selection
             if ( actual_character == 48 ){
                 blink_stop();
                 blink_init(1, 3, 14, 3);
@@ -622,9 +615,9 @@ int main(void){
             } else 
             if ( actual_character == 41 ){
                 blink_stop();
-                menu_state = MENU_STATE_START_ACTIVE;
+                menu_state = MENU_STATE_S_ACTIVE;
                 put_one_char(' ', 5, 0, 7, DISP_FRONTBUFFER);
-                put_data_to_lcd_buffer(&menu0_line0_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);
+                put_data_to_lcd_buffer(&text_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);
                 blink_init(0, 7, 4, 3);
                 sorting_state = 1;
             } else
@@ -634,8 +627,7 @@ int main(void){
                 menu_state = MENU_STATE_PWROFF;
             }
         } else
-        if ( menu_state == MENU_STATE_SELECT ){
-            last_menu_state = MENU_STATE_SELECT;
+        if ( menu_state == MENU_STATE_SELECT ){         // Handle "select program" election
             if ( actual_character == 48 ){
                 blink_stop();
                 blink_init(2, 6, 9, 3);
@@ -643,10 +635,9 @@ int main(void){
             } else 
             if ( actual_character == 41 ){
                 blink_stop();
-                menu_state = MENU_STATE_MAIN;
-                //menu_state = MENU_STATE_SELECT_ACTIVE;
-                //disp_operation = DISP_STATE_CLEAR;
-                //disp_clear_buffer(DISP_FRONTBUFFER);
+                disp_clear_buffer(DISP_FRONTBUFFER);
+                put_data_to_lcd_buffer(&text_select_hint, 20, 3, 0, DISP_FRONTBUFFER, 1);
+                menu_state = MENU_STATE_SEL_ACTIVE;
             } else
             if (  actual_character == 34 ){
                 blink_stop();
@@ -654,26 +645,14 @@ int main(void){
                 menu_state = MENU_STATE_START;
             }
         } else
-        if ( menu_state == MENU_STATE_CONFIG ){
-            last_menu_state = MENU_STATE_CONFIG;
+        if ( menu_state == MENU_STATE_CONFIG ){         // Handle "configure" selection
             if ( actual_character == 48 ){
                 blink_stop();
                 blink_init(3, 6, 9, 3);
                 menu_state = MENU_STATE_PWROFF;
             } else 
             if ( actual_character == 41 ){
-                blink_stop();
-                menu_state = MENU_STATE_CONFIG_ACTIVE;
-                disp_operation = DISP_STATE_CLEAR;
-                disp_clear_buffer(DISP_FRONTBUFFER);
-
-                put_data_to_lcd_buffer(&menu2_line0_config, 10, 0, 0, DISP_FRONTBUFFER, 1);
-                put_data_to_lcd_buffer(&menu2_line0, 7, 0, 12, DISP_FRONTBUFFER, 1);
-                put_data_to_lcd_buffer(&menu2_line1_exit, 4, 1, 4, DISP_FRONTBUFFER, 1);
-                put_data_to_lcd_buffer(&menu2_line1, 6, 1, 10, DISP_FRONTBUFFER, 1);
-                put_data_to_lcd_buffer(&menu2_line2, 7, 2, 0, DISP_FRONTBUFFER, 1);
-
-                blink_init(0, 12, 7, 3);
+                menu_state = MENU_STATE_DRAW_CONFIG;
             } else
             if (  actual_character == 34 ){
                 blink_stop();
@@ -681,8 +660,7 @@ int main(void){
                 menu_state = MENU_STATE_SELECT;
             }
         } else
-        if ( menu_state == MENU_STATE_PWROFF ){
-            last_menu_state = MENU_STATE_PWROFF;
+        if ( menu_state == MENU_STATE_PWROFF ){         // Handle "power off" selection
             if ( actual_character == 48 ){
                 blink_stop();
                 blink_init(0, 7, 5, 3);
@@ -698,8 +676,7 @@ int main(void){
                 menu_state = MENU_STATE_CONFIG;
             }
         } else
-        if ( menu_state == MENU_STATE_START_ACTIVE ){
-            last_menu_state = MENU_STATE_START_ACTIVE;
+        if ( menu_state == MENU_STATE_S_ACTIVE ){       // Handle toggle from START to STOP
             if ( actual_character == 48 ){
                 blink_stop();
                 blink_init(1, 3, 14, 3);
@@ -708,7 +685,7 @@ int main(void){
             if ( actual_character == 41 ){
                 blink_stop();
                 menu_state = MENU_STATE_START;
-                put_data_to_lcd_buffer(&menu0_line0_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
+                put_data_to_lcd_buffer(&text_start, 5, 0, 7, DISP_FRONTBUFFER, 1);
                 blink_init(0, 7, 5, 3);
                 sorting_state = 0;
             } else
@@ -718,8 +695,10 @@ int main(void){
                 menu_state = MENU_STATE_PWROFF;
             }    
         } else
-        if ( menu_state == MENU_STATE_SELECT_ACTIVE ){
-            last_menu_state = MENU_STATE_SELECT_ACTIVE;
+        if ( menu_state == MENU_STATE_SEL_ACTIVE ){     // Handle active "select program"
+            if ( actual_character == 41 ){
+                menu_state = MENU_STATE_DRAW_MAIN;
+            }
             // if ( actual_character == 48 ){
             //     blink_stop();
             //     blink_init(1, 3, 14, 3);
@@ -736,26 +715,41 @@ int main(void){
             //     blink_init(3, 6, 9, 3);
             //     menu_state = MENU_STATE_PWROFF;
             // }
-            // if ( menu_state != MENU_STATE_START_ACTIVE )
+            // if ( menu_state != MENU_STATE_S_ACTIVE )
             //     put_data_to_lcd_buffer(&menu0_line0_stop, 4, 0, 7, DISP_FRONTBUFFER, 1);       
         } else
-        if ( menu_state == MENU_STATE_CONFIG_ACTIVE || menu_state == MENU_STATE_CFG_PROGRAM || menu_state == MENU_STATE_CFG_SYSTEM || menu_state == MENU_STATE_CFG_EXIT ){
-            last_menu_state = menu_state;
-            
+        if ( menu_state == MENU_STATE_DRAW_CONFIG ) {       // Draw "configure" screen
+            blink_stop();
+            menu_state = MENU_STATE_C_ACTIVE;
+            disp_clear_buffer(DISP_FRONTBUFFER);
+
+            put_data_to_lcd_buffer(&text_configure0, 10, 0, 0, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_program, 7, 0, 12, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_exit, 4, 1, 4, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_system, 6, 1, 10, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_status0, 7, 2, 0, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_runtime0, 8, 3, 0, DISP_FRONTBUFFER, 1);
+
+            blink_init(0, 12, 7, 3);
+        } else
+        if ( menu_state == MENU_STATE_C_ACTIVE || menu_state == MENU_STATE_C_A_PROGRAM || menu_state == MENU_STATE_C_A_SYSTEM || 
+            menu_state == MENU_STATE_C_A_EXIT ){               // Handle active "configure"
+
+            // Display state of the machine
             if ( sorting_state & 0xF0 )
-                put_data_to_lcd_buffer(&menu2_line2_err, 3, 2, 8, DISP_FRONTBUFFER, 1);
+                put_data_to_lcd_buffer(&text_err, 3, 2, 8, DISP_FRONTBUFFER, 1);
             else
-                put_data_to_lcd_buffer(&menu2_line2_ok, 2, 2, 8, DISP_FRONTBUFFER, 1);
+                put_data_to_lcd_buffer(&text_ok, 2, 2, 8, DISP_FRONTBUFFER, 1);
 
             put_one_char(',', 1, 2, 11, DISP_FRONTBUFFER);
 
+            // Display state of sorting
             if ( sorting_state & 0x0F )
-                put_data_to_lcd_buffer(&menu2_line2_running, 7, 2, 13, DISP_FRONTBUFFER, 1);
+                put_data_to_lcd_buffer(&text_running, 7, 2, 13, DISP_FRONTBUFFER, 1);
             else
-                put_data_to_lcd_buffer(&menu2_line2_stopped, 7, 2, 13, DISP_FRONTBUFFER, 1);
-
-            put_data_to_lcd_buffer(&menu2_line3, 8, 3, 0, DISP_FRONTBUFFER, 1);
+                put_data_to_lcd_buffer(&text_stopped, 7, 2, 13, DISP_FRONTBUFFER, 1);
 
+            // Display runtime XD
             put_one_char('0' + (hour >> 4), 1, 3, 10, DISP_FRONTBUFFER);
             put_one_char('0' + (hour & 0x0F), 1, 3, 11, DISP_FRONTBUFFER);
             put_one_char(':', 1, 3, 12, DISP_FRONTBUFFER);
@@ -765,59 +759,165 @@ int main(void){
             put_one_char('0' + (sec >> 4), 1, 3, 16, DISP_FRONTBUFFER);
             put_one_char('0' + (sec & 0x0F), 1, 3, 17, DISP_FRONTBUFFER);
 
-            // put_data_to_lcd_buffer("TEST RUN", 8, 0, 0, DISP_FRONTBUFFER, 0);
-            // sprintf(val, "metal = %05d", metal_sense_value / 10);
-            // put_data_to_lcd_buffer(val, 13, 1, 0, DISP_FRONTBUFFER, 0);
-            // sprintf(val, "color = %05d", color_sense_value / 10);
-            // put_data_to_lcd_buffer(val, 13, 2, 0, DISP_FRONTBUFFER, 0);
-
-            if ( menu_state == MENU_STATE_CONFIG_ACTIVE || menu_state == MENU_STATE_CFG_PROGRAM ){
+            if ( menu_state == MENU_STATE_C_ACTIVE || menu_state == MENU_STATE_C_A_PROGRAM ){       // Handle "configure" -> "program" selection
                 if ( actual_character == 34 ){
                     blink_stop();
                     blink_init(1, 4, 4, 3);
-                    menu_state = MENU_STATE_CFG_EXIT;
+                    menu_state = MENU_STATE_C_A_EXIT;
                 } else 
                 if ( actual_character == 41 ){
-                    blink_stop();
-                    menu_state = MENU_STATE_CONFIG_ACTIVE;
+                    menu_state = MENU_STATE_DRAW_C_PROG;
                 } else
                 if (  actual_character == 48 ){
                     blink_stop();
                     blink_init(1, 10, 6, 3);
-                    menu_state = MENU_STATE_CFG_SYSTEM;
+                    menu_state = MENU_STATE_C_A_SYSTEM;
                 }
             } else 
-            if ( menu_state == MENU_STATE_CFG_SYSTEM ) {
+            if ( menu_state == MENU_STATE_C_A_SYSTEM ) {            // Handle "configure" -> "system" selection
                 if ( actual_character == 34 ){
                     blink_stop();
                     blink_init(0, 12, 7, 3);
-                    menu_state = MENU_STATE_CFG_PROGRAM;
+                    menu_state = MENU_STATE_C_A_PROGRAM;
                 } else 
                 if ( actual_character == 41 ){
                     blink_stop();
-                    menu_state = MENU_STATE_CONFIG_ACTIVE;
+                    menu_state = MENU_STATE_C_A_SYSTEM_ACTIVE;
+                    disp_clear_buffer(DISP_FRONTBUFFER);
+
+                    print_settings_options(DISP_FRONTBUFFER);
                 } else
                 if (  actual_character == 48 ){
                     blink_stop();
                     blink_init(1, 4, 4, 3);
-                    menu_state = MENU_STATE_CFG_EXIT;
+                    menu_state = MENU_STATE_C_A_EXIT;
                 }
             } else 
-            if ( menu_state == MENU_STATE_CFG_EXIT ) {
+            if ( menu_state == MENU_STATE_C_A_EXIT ) {              // Handle "configure" -> "exit" selection
                 if ( actual_character == 34 ){
                     blink_stop();
                     blink_init(1, 10, 6, 3);
-                    menu_state = MENU_STATE_CFG_SYSTEM;
+                    menu_state = MENU_STATE_C_A_SYSTEM;
                 } else 
                 if ( actual_character == 41 ){
                     blink_stop();
-                    menu_state = MENU_STATE_MAIN;
+                    menu_state = MENU_STATE_DRAW_MAIN;
                 } else
                 if (  actual_character == 48 ){
                     blink_stop();
                     blink_init(0, 12, 7, 3);
-                    menu_state = MENU_STATE_CFG_PROGRAM;
+                    menu_state = MENU_STATE_C_A_PROGRAM;
+                }
+            }
+        } else 
+        if ( menu_state == MENU_STATE_DRAW_C_PROG ) {               // Draw "configure" -> "program" screen
+            blink_stop();
+            menu_state = MENU_STATE_C_A_PROGRAM_ACTIVE;
+            disp_clear_buffer(DISP_FRONTBUFFER);
+
+            put_data_to_lcd_buffer(&text_parameters0, 11, 0, 0, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_view, 4, 1, 8, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_configure, 9, 2, 4, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_save, 4, 3, 4, DISP_FRONTBUFFER, 1);
+            put_data_to_lcd_buffer(&text_exit, 4, 3, 12, DISP_FRONTBUFFER, 1);
+
+            blink_init(1, 8, 4, 3);
+        } else
+        if ( menu_state == MENU_STATE_C_A_PROGRAM_ACTIVE || menu_state == MENU_STATE_C_A_PRG_A_view || menu_state == MENU_STATE_C_A_PRG_A_config ||
+            menu_state == MENU_STATE_C_A_PRG_A_save || menu_state == MENU_STATE_C_A_PRG_A_exit ){          // Handle "configure" -> "program" selection
+            if ( menu_state == MENU_STATE_C_A_PROGRAM_ACTIVE || menu_state == MENU_STATE_C_A_PRG_A_view ){      // Handle "configure" -> "program" - > "view" selection
+                if ( actual_character == 34 ){
+                    blink_stop();
+                    blink_init(3, 12, 4, 3);
+                    menu_state = MENU_STATE_C_A_PRG_A_exit;
+                } else 
+                if ( actual_character == 41 ){
+                    blink_stop();
+                    menu_state = MENU_STATE_C_A_PRG_A_view_A;
+                    disp_clear_buffer(DISP_FRONTBUFFER);
+                } else
+                if (  actual_character == 48 ){
+                    blink_stop();
+                    blink_init(2, 4, 9, 3);
+                    menu_state = MENU_STATE_C_A_PRG_A_config;
+                }
+            } else 
+            if ( menu_state == MENU_STATE_C_A_PRG_A_config ) {          // Handle "configure" -> "program" - > "configure" selection
+                if ( actual_character == 34 ){
+                    blink_stop();
+                    blink_init(1, 8, 4, 3);
+                    menu_state = MENU_STATE_C_A_PRG_A_view;
+                } else 
+                if ( actual_character == 41 ){
+                    blink_stop();
+                    menu_state = MENU_STATE_C_A_PRG_A_config_A;
+                    disp_clear_buffer(DISP_FRONTBUFFER);
+
+                    print_settings_options(DISP_FRONTBUFFER);
+                } else
+                if (  actual_character == 48 ){
+                    blink_stop();
+                    blink_init(3, 4, 4, 3);
+                    menu_state = MENU_STATE_C_A_PRG_A_save;
                 }
+            } else 
+            if ( menu_state == MENU_STATE_C_A_PRG_A_save ) {            // Handle "configure" -> "program" - > "save" selection
+                if ( actual_character == 34 ){
+                    blink_stop();
+                    blink_init(2, 4, 9, 3);
+                    menu_state = MENU_STATE_C_A_PRG_A_config;
+                } else 
+                if ( actual_character == 41 ){
+                    blink_stop();
+                    menu_state = MENU_STATE_DRAW_MAIN;
+                } else
+                if (  actual_character == 48 ){
+                    blink_stop();
+                    blink_init(3, 12, 4, 3);
+                    menu_state = MENU_STATE_C_A_PRG_A_exit;
+                }
+            } else 
+            if ( menu_state == MENU_STATE_C_A_PRG_A_exit ) {            // Handle "configure" -> "program" - > "exit" selection
+                if ( actual_character == 34 ){
+                    blink_stop();
+                    blink_init(3, 4, 4, 3);
+                    menu_state = MENU_STATE_C_A_PRG_A_save;
+                } else 
+                if ( actual_character == 41 ){
+                    blink_stop();
+                    menu_state = MENU_STATE_DRAW_CONFIG;
+                } else
+                if (  actual_character == 48 ){
+                    blink_stop();
+                    blink_init(1, 8, 4, 1);
+                    menu_state = MENU_STATE_C_A_PRG_A_view;
+                }
+            }
+        } else
+        if ( menu_state == MENU_STATE_C_A_PRG_A_view_A ){               // Draw "configure" -> "program" -> "view" screen
+            sprintf(val, "metal: %05u", metal_sense_value / 10);
+            put_data_to_lcd_buffer(val, 12, 0, 0, DISP_FRONTBUFFER, 0);
+            sprintf(val, "red: %05u", red_value);
+            put_data_to_lcd_buffer(val, 10, 1, 0, DISP_FRONTBUFFER, 0);
+            //put_one_char('0' + stage1_state, 1, 1, 19, DISP_FRONTBUFFER);
+            sprintf(val, "green: %05u", green_value);
+            put_data_to_lcd_buffer(val, 12, 2, 0, DISP_FRONTBUFFER, 0);
+            sprintf(val, "blue: %05u", blue_value);
+            put_data_to_lcd_buffer(val, 11, 3, 0, DISP_FRONTBUFFER, 0);
+            //put_one_char('0' + stage3_state, 3, 1, 19, DISP_FRONTBUFFER);
+
+            if ( actual_character == 41 ){
+                menu_state = MENU_STATE_DRAW_C_PROG;
+            }
+        } else
+        if ( menu_state == MENU_STATE_C_A_PRG_A_config_A ){
+            if ( actual_character == 41 ){
+                menu_state = MENU_STATE_DRAW_C_PROG;
+            }
+        } else
+        if ( menu_state == MENU_STATE_C_A_SYSTEM_ACTIVE ){
+            if ( actual_character == 41 ){
+                menu_state = MENU_STATE_DRAW_CONFIG;
             }
         }
 
@@ -833,6 +933,9 @@ int main(void){
                 }
             } else
             if ( stage1_state == STAGE_STATE_MEASURE ){
+                if ( stage1_measure_hold == 20 ){
+                    adc_read_count = 0;
+                }
                 if ( --stage1_measure_hold == 0 ){
                     stage1_measure_hold = 20;
                     stage1_state = STAGE_STATE_OUT;
@@ -846,7 +949,7 @@ int main(void){
             } else
             if ( stage1_state == STAGE_STATE_DEFAULT ){
                 if ( --stage1_out_wait == 0 ){
-                    stage1_out_wait = 50;
+                    stage1_out_wait = 30;
                     compbuff_PWM0 = 53;   
                     stage1_state = STAGE_STATE_WAIT; 
                 }
@@ -858,14 +961,50 @@ int main(void){
             } else
             if ( stage3_state == STAGE_STATE_IN ){
                 if ( --stage3_in_wait == 0 ){
-                    stage3_in_wait = 50;
+                    stage3_in_wait = 30;
                     stage3_state = STAGE_STATE_MEASURE;
+                    PIN_clear(RED_LED_port, RED_LED_pin);
+                    stage3_config &= 0xF3;
+                    stage3_config |= 4;
+                    adc_read_count = 0;
                 }
             } else
             if ( stage3_state == STAGE_STATE_MEASURE ){
+                if ( --stage3_color_switch_hold == 0 ){
+                    if ( (stage3_config & 0x0C) == 4 ){
+                        stage3_config &= 0xF3;
+                        stage3_config |= 8;
+                        PIN_set(RED_LED_port, RED_LED_pin);
+                        PIN_clear(GREEN_LED_port, GREEN_LED_pin);
+                        adc_read_count = 0;
+                        red_value = color_sense_value / 10;
+                    } else 
+                    if ( (stage3_config & 0x0C) == 8 ){
+                        stage3_config &= 0xF3;
+                        stage3_config |= 12;
+                        PIN_set(GREEN_LED_port, GREEN_LED_pin);
+                        PIN_clear(BLUE_LED_port, BLUE_LED_pin);
+                        adc_read_count = 0;
+                        green_value = color_sense_value / 10;
+                    } else
+                    if ( (stage3_config & 0x0C) == 12 ){
+                        stage3_config &= 0xF3;
+                        stage3_config |= 4;
+                        PIN_set(BLUE_LED_port, BLUE_LED_pin);
+                        PIN_clear(RED_LED_port, RED_LED_pin);
+                        adc_read_count = 0;
+                        blue_value = color_sense_value / 10;
+                    }
+                    stage3_color_switch_hold = 30;
+                }
+
                 if ( --stage3_measure_hold == 0 ){
-                    stage3_measure_hold = 20;
+                    stage3_measure_hold = 90;
                     stage3_state = STAGE_STATE_OUT;
+                    PIN_set(RED_LED_port, RED_LED_pin);
+                    PIN_set(GREEN_LED_port, GREEN_LED_pin);
+                    PIN_set(BLUE_LED_port, BLUE_LED_pin);
+                    PIN_set(WHITE_LED_port, WHITE_LED_pin);
                 }
             } else
             if ( stage3_state == STAGE_STATE_OUT ){
@@ -874,7 +1013,7 @@ int main(void){
             } else
             if ( stage3_state == STAGE_STATE_DEFAULT ){
                 if ( --stage3_out_wait == 0 ){
-                    stage3_out_wait = 50;
+                    stage3_out_wait = 30;
                     compbuff_PWM2 = 56;   
                     stage3_state = STAGE_STATE_WAIT; 
                 }
