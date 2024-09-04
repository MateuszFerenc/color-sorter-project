diff --git a/definitions.h b/definitions.h
index c7d9577..507da84 100644
--- a/definitions.h
+++ b/definitions.h
@@ -10,6 +10,7 @@
 #include <avr/pgmspace.h>
 #include <avr/wdt.h>
 #include <stdio.h>
+#include <avr/eeprom.h>
 
 // Include section end
 
@@ -26,7 +27,7 @@
 #define STRINGIFY(x) #x
 #define TOSTRING(x) STRINGIFY(x)
 #define stable_version  0
-#define beta_version    1
+#define beta_version    3
 
 // sense stages ADC channels PORTA
 #define metal_sense_adc 0
@@ -69,19 +70,28 @@
 #define BUZZER_pin          PD2
 #define BUZZER_port         PORTD
 
-#define MENU_STATE_MAIN         0
+#define MENU_STATE_DRAW_MAIN         0
 #define MENU_STATE_START        1
-//#define MENU_STATE_STOP         2
 #define MENU_STATE_SELECT       3
 #define MENU_STATE_CONFIG       4
 #define MENU_STATE_PWROFF       5
-#define MENU_STATE_START_ACTIVE         6
-#define MENU_STATE_SELECT_ACTIVE        7
-#define MENU_STATE_CONFIG_ACTIVE      8
-//#define MENU_STATE_PWROFF_ACTIVE        9
-#define MENU_STATE_CFG_PROGRAM              10
-#define MENU_STATE_CFG_SYSTEM              11
-#define MENU_STATE_CFG_EXIT              12
+#define MENU_STATE_S_ACTIVE         6
+#define MENU_STATE_SEL_ACTIVE        7
+#define MENU_STATE_C_ACTIVE      8
+#define MENU_STATE_C_A_PROGRAM              10
+#define MENU_STATE_C_A_SYSTEM              11
+#define MENU_STATE_C_A_EXIT              12
+#define MENU_STATE_C_A_PROGRAM_ACTIVE       13
+#define MENU_STATE_C_A_SYSTEM_ACTIVE       14
+#define MENU_STATE_C_A_PRG_A_view            15
+#define MENU_STATE_C_A_PRG_A_config            16
+#define MENU_STATE_C_A_PRG_A_save            17
+#define MENU_STATE_C_A_PRG_A_exit            18
+#define MENU_STATE_C_A_SYS_ACTIVE            19
+#define MENU_STATE_C_A_PRG_A_view_A            20
+#define MENU_STATE_C_A_PRG_A_config_A            21
+#define MENU_STATE_DRAW_CONFIG                  22
+#define MENU_STATE_DRAW_C_PROG                  23
 #define MENU_STATE_DRIVE                255
 
 #define STAGE_STATE_WAIT                0
@@ -163,7 +173,7 @@ Buzzer                  (PD2)
 LCD Backlight           (PA1)
 */
 
-// Functions definitions start
+// Functions declarations start
 
 void USART_Transmit( char data );
 unsigned char USART_Receive( void );
@@ -174,8 +184,9 @@ void wait_us( uint8_t us );
 void lcd_command( uint8_t command );
 void lcd_write_nibble( uint8_t data );
 void lcd_init( void );
+// TODO change defined pointer to void pointer
 void put_data_to_lcd_buffer(unsigned char* data, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer, uint8_t from_flash);
-void put_line_to_lcd_buffer(unsigned char* text, uint8_t buffer, uint8_t row, uint8_t from_flash);
+void put_one_char(unsigned char character, uint8_t length, uint8_t row, uint8_t col, uint8_t buffer);
 void disp_clear_buffer(uint8_t buffer);
 uint8_t disp_swap_buffers(void);
 uint8_t blink_init(uint8_t row, uint8_t col, uint8_t length, uint8_t period);
@@ -183,15 +194,18 @@ void blink_stop( void );
 uint8_t read_keypad( void );
 unsigned char get_keypad_character( void );
 void fake_shutdown( void );
+void print_settings_options( unsigned char buffer );
+void check_eeprom_variables( void );
 
-// Functions definitions endl
+// Functions declarations end
 
 //  Variables start
-uint16_t metal_sense_buffer[10] = { 0 }, glass_sense_buffer[10] = { 0 }, color_sense_buffer[10] = { 0 }, metal_sense_value = 0, glass_sense_value = 0, color_sense_value = 0;
+uint16_t metal_sense_buffer[10] = { 0 }, color_sense_buffer[10] = { 0 }, metal_sense_value = 0, color_sense_value = 0;
+//uint16_t glass_sense_buffer[10] = { 0 }, glass_sense_value = 0;
 uint16_t system_counter = 0, sec_counter = 0;
 uint8_t sec = 0, min = 0, hour = 0;
 uint8_t adc_hold = 0, adc_read_count = 0;
-uint8_t lcdColumns, lcdRows, currentCol, currentRow, lcdRowStart[4];
+uint8_t currentCol, currentRow, lcdRowStart[4];
 
 uint8_t compare_PWM0, compare_PWM1, compare_PWM2;
 volatile uint8_t compbuff_PWM0, compbuff_PWM1, compbuff_PWM2;
@@ -203,15 +217,12 @@ unsigned char disp_linear_buff[160];
 uint8_t disp_buffers_dirty = 0;        // buffer "dirty" bits, one means buffer updated and ready to display
 // bits: 7-4: disp_linear_buff[79:159] = 7 - 4th line .. 4 - 1st line, 3-0: disp_linear_buff[0:79] = 3 - 4th line .. 0 - 1st line
 
-uint8_t disp_state = DISP_STATE_NOP, disp_last_state = DISP_STATE_NOP, disp_operation = DISP_STATE_NOP;
+uint8_t disp_operation = DISP_STATE_NOP;
 
-uint8_t val_pwm0 = 0;
+uint8_t disp_active_buffer = DISP_FRONTBUFFER;
+void *disp_buffer_pointer = NULL;
 
-uint8_t disp_delay = 0, disp_temp_data = 0, disp_column_counter = 0, disp_active_buffer = DISP_FRONTBUFFER;
-unsigned char *disp_buffer_pointer = NULL;
-
-uint8_t menu_state = MENU_STATE_MAIN, last_menu_state = MENU_STATE_MAIN;
-uint8_t selected_object = 0;            // bit 7 - blink state, bits 6:0 - selection
+uint8_t menu_state = MENU_STATE_DRAW_MAIN, last_menu_state = MENU_STATE_DRAW_MAIN;
 
 uint8_t blink_position = 0;             // bits 7 - blink state, 6:5 - row, 4:0 - column
 uint8_t blink_conf = 0;                 // bits 7:4 - period [0 - off, 1-15], 3:0 - length [1-16 characters]
@@ -220,31 +231,107 @@ uint8_t blink_conf = 0;                 // bits 7:4 - period [0 - off, 1-15], 3:
 unsigned char blink_buffer[16];
 
 uint8_t sorting_state = 0;      // bits 7:4 - error code, 3:0 - sorting stage info
+
+uint8_t EEPROM_variables_config = 0;    // 0xF0 : ROM -> RAM, 0x0F : RAM -> ROM
+uint8_t EEPROM_variable_count = 0;
+
+
+// Sorter process dependent variables, values loaded from "S_"
+uint8_t stage1_servo_accept, stage1_servo_default, stage1_servo_reject;
+//  uint8_t stage2_servo_accept, stage2_servo_default, stage2_servo_reject;
+uint8_t stage3_servo_accept, stage3_servo_default, stage3_servo_reject;
+
+uint8_t stage1_in_wait, stage1_measure_hold, stage1_out_wait;
+// uint8_t stage2_in_wait, stage2_measure_hold, stage2_out_wait;
+uint8_t stage3_in_wait, stage3_measure_hold, stage3_out_wait, stage3_color_switch_hold;
+
+//  Sorter default editable set points loadable from EEPROM, "S_" prefix means set point
+uint8_t S_stage1_servo_accept, S_stage1_servo_default, S_stage1_servo_reject;
+//  uint8_t stage2_servo_accept, stage2_servo_default, stage2_servo_reject;
+uint8_t S_stage3_servo_accept, S_stage3_servo_default, S_stage3_servo_reject;
+
+uint8_t S_stage1_in_wait, S_stage1_measure_hold, S_stage1_out_wait;
+// uint8_t stage2_in_wait, stage2_measure_hold, stage2_out_wait;
+uint8_t S_stage3_in_wait, S_stage3_measure_hold, S_stage3_out_wait, S_stage3_color_switch_hold;
+
 //  Variables end
 
+
+// EEPROM data region start, "E_" prefix means eeprom data region
+
+// Stage Servos Limits
+static EEMEM uint8_t  E_stage1_servo_accept = 65, E_stage1_servo_default = 53, E_stage1_servo_reject = 43;
+//uint8_t EEMEM E_stage2_servo_accept = 0, E_stage2_servo_default = 0, E_stage2_servo_reject = 0;
+static EEMEM uint8_t E_stage3_servo_accept = 68, E_stage3_servo_default = 56, E_stage3_servo_reject = 46;
+
+static EEMEM uint8_t E_stage1_in_wait = 50, E_stage1_measure_hold = 20, E_stage1_out_wait = 30;
+//uint8_t EEMEM E_stage2_in_wait = 2, E_stage2_measure_hold = 100, E_stage2_out_wait = 2;
+static EEMEM uint8_t E_stage3_in_wait = 30, E_stage3_measure_hold = 90, E_stage3_out_wait = 30, E_stage3_color_switch_hold = 30;
+
+// EEPROM data region end
+
+
 //  Constans start
+
+// TODO optimize this
+
+const uint8_t * const epprom_variables_pointer_array [13] PROGMEM = { &E_stage1_servo_accept, &E_stage1_servo_default, &E_stage1_servo_reject, 
+                                                &E_stage3_servo_accept, &E_stage3_servo_default, &E_stage3_servo_reject,
+                                                &E_stage1_in_wait, &E_stage1_measure_hold, &E_stage1_out_wait,
+                                                &E_stage3_in_wait, &E_stage3_measure_hold, &E_stage3_out_wait,
+                                                &E_stage3_color_switch_hold
+                                                //  &E_stage2_servo_accept, &E_stage2_servo_default, &E_stage2_servo_reject,
+                                                //  &E_stage2_in_wait, &E_stage2_measure_hold, &E_stage2_out_wait, 
+                                                };
+
 const unsigned char keypad_num0_keys[5] PROGMEM = "-12-3";
 const unsigned char keypad_num1_keys[5] PROGMEM = "-45-6";
 const unsigned char keypad_num2_keys[5] PROGMEM = "-78-9";
 const unsigned char keypad_num3_keys[5] PROGMEM = "-*0-#";
 const unsigned char keypad_func_keys[5] PROGMEM = "-ABCD";
 
-const unsigned char menu0_line0_start[5] PROGMEM = "START";
-const unsigned char menu0_line0_stop[4] PROGMEM = "STOP";
-const unsigned char menu0_line1[14] PROGMEM = "select program";
-const unsigned char menu0_line2[9] PROGMEM = "configure";
-const unsigned char menu0_line3[9] PROGMEM = "power off";
-
-const unsigned char menu2_line0_config[10] PROGMEM = "Configure:";
-const unsigned char menu2_line0[7] PROGMEM = "program";
-const unsigned char menu2_line1[6] PROGMEM = "system";
-const unsigned char menu2_line1_exit[4] PROGMEM = "exit";
-const unsigned char menu2_line2[7] PROGMEM = "Status:";
-const unsigned char menu2_line2_ok[2] PROGMEM = "OK";
-const unsigned char menu2_line2_err[3] PROGMEM = "Err";
-const unsigned char menu2_line2_stopped[7] PROGMEM = "stopped";
-const unsigned char menu2_line2_running[7] PROGMEM = "running";
-const unsigned char menu2_line3[8] PROGMEM = "Runtime:";
-
+const unsigned char key_map[26] PROGMEM = {
+        '-', '1', '2', '-', '3', '-', '-', '-', 
+        '4', '5', '-', '6', '-', '-', '-', 
+        '7', '8', '-', '9', '-', '-', '-', 
+        '*', '0', '-', '#'
+    };
+//
+
+
+// Menu 0 strings
+const unsigned char text_start[5] PROGMEM = "START";
+const unsigned char text_stop[4] PROGMEM = "STOP";
+const unsigned char text_select_program[14] PROGMEM = "select program";
+const unsigned char text_configure[9] PROGMEM = "configure";
+const unsigned char text_power_off[9] PROGMEM = "power off";
+
+// Menu 1 strings
+const unsigned char text_select_hint[20] PROGMEM = "1.LD 2.DEL 3.SV 4.NW";
+
+// Menu 2 strings
+const unsigned char text_configure0[10] PROGMEM = "Configure:";
+#define text_program *(text_select_program + 7)
+//const unsigned char text_program[7] PROGMEM = "program";
+const unsigned char text_system[6] PROGMEM = "system";
+const unsigned char text_exit[4] PROGMEM = "exit";
+const unsigned char text_status0[7] PROGMEM = "Status:";
+const unsigned char text_ok[2] PROGMEM = "OK";
+const unsigned char text_err[3] PROGMEM = "Err";
+const unsigned char text_stopped[7] PROGMEM = "stopped";
+const unsigned char text_running[7] PROGMEM = "running";
+const unsigned char text_runtime0[8] PROGMEM = "Runtime:";
+
+// Menu 3 strings
+const unsigned char text_parameters0[11] PROGMEM = "Parameters:";
+const unsigned char text_view[4] PROGMEM = "view";
+const unsigned char text_save[4] PROGMEM = "save";
+
+// Menu 4 strings
+
+// Other strings
 const unsigned char text_goodbye[10] PROGMEM = "Goodbye :)";
+const unsigned char sorter_version[11] PROGMEM = "Sorter v" TOSTRING(stable_version) "." TOSTRING(beta_version);
+const unsigned char compilation_date[11] PROGMEM = __DATE__;
+
 //  Constans end
\ No newline at end of file
