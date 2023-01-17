
// Uses Adafruit Feather nRF52832
// needs the following libraries
// - bluemicro_hid
#include <bluemicro_hid.h>
#include "hid_keycodes.h"

#define MATRIX_ROWS 4
#define MATRIX_COLS 12

#define MATRIX_ROW_PINS {3, 14, 13, 11}
#define MATRIX_COL_PINS {5, 4, 16, 15, 30, 29, 28, 27, 26, 25, 7, 18} 

#define  STATUS_BLE_LED_PIN  19  //blue = 0.19
#define  STATUS_KB_LED_PIN 17  //red = 0.17


byte rows[] MATRIX_ROW_PINS;        // Contains the GPIO Pin Numbers 
byte columns[] MATRIX_COL_PINS;     // Contains the GPIO Pin Numbers  

uint8_t keymap[] =    {KC_ESC,    KC_Q,    KC_W,    KC_E,   KC_R,    KC_T,    KC_Y,    KC_U,  KC_I,    KC_O,    KC_P,     KC_BSPACE, \
              KC_TAB,    KC_A,    KC_S,    KC_D,   KC_F,    KC_G,    KC_H,    KC_J,  KC_K,    KC_L,    KC_SCLN,  KC_QUOT, \
              KC_LSFT,   KC_Z,    KC_X,    KC_C,   KC_V,    KC_B,    KC_N,    KC_M,  KC_COMMA,KC_DOT,  KC_SLASH, KC_ENTER, \
              KC_LCTL,   KC_LGUI, KC_LALT, KC_RGUI,LAYER_1, KC_SPC,  KC_SPC, LAYER_2,KC_LEFT, KC_UP,   KC_DOWN,  KC_RIGHT};

void setup() {
  // put your setup code here, to run once:
  bluemicro_hid.begin(); 
  Serial.begin(115200);
  Serial.println("BlueMicro_HID Large Matrix Tests");
}




void loop() {
  // put your main code here, to run repeatedly:
    // use to send key release report
    static bool has_key = false;
    uint32_t pinreg = 0;
    bool btn_pressed = false;

 
    //Setting up Scanning, Enabling all columns
    for (int i = 0; i < MATRIX_COLS; ++i){
        pinMode(columns[i], INPUT_PULLUP);
    } 


// FIRST ROW
    pinMode(rows[0], OUTPUT);
    digitalWrite(rows[0],LOW);
    nrfx_coredep_delay_us(1);   // need for the GPIO lines to settle down electrically before reading first row;
    // READ FIRST ROW
    pinreg = (NRF_GPIO->IN)^0xffffffff;  // press is active high regardless of diode dir
    pinMode(rows[0], INPUT);                                        // 'disables' the row that was just scanned  
// SECOND ROW
    pinMode(rows[1], OUTPUT);
    digitalWrite(rows[1],LOW);
    // PROCESS FIRST ROW - need for the GPIO lines to settle down electrically before reading second row;
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            btn_pressed = true;
            uint8_t keycode[6] = { 0 };
            keycode[0] = keymap[0*MATRIX_COLS + i];
            bluemicro_hid.keyboardReport(0, keycode);
            Serial.print("row: 0 Column");
            Serial.println(i);
            has_key = true;
          }
    }
    // READ SECOND ROW
    pinreg = (NRF_GPIO->IN)^0xffffffff;  // press is active high regardless of diode dir
    pinMode(rows[1], INPUT); 
// THIRD ROW
    pinMode(rows[2], OUTPUT);
    digitalWrite(rows[2],LOW);
    // PROCESS SECOND ROW - need for the GPIO lines to settle down electrically before reading second row;
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            btn_pressed = true;
            uint8_t keycode[6] = { 0 };
            keycode[0] = keymap[1*MATRIX_COLS + i];
            bluemicro_hid.keyboardReport(0, keycode);
            Serial.print("row: 1 Column");
            Serial.println(i);
            has_key = true;
          }
    }
    // READ THIRD ROW
    pinreg = (NRF_GPIO->IN)^0xffffffff;  // press is active high regardless of diode dir
    pinMode(rows[2], INPUT); 
// FOURTH ROW
    pinMode(rows[3], OUTPUT);
    digitalWrite(rows[3],LOW);
    // PROCESS THIRD ROW - need for the GPIO lines to settle down electrically before reading second row;
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            btn_pressed = true;
            uint8_t keycode[6] = { 0 };
            keycode[0] = keymap[2*MATRIX_COLS + i];
            bluemicro_hid.keyboardReport(0, keycode); 
            Serial.print("row: 2 Column");
            Serial.println(i);
            has_key = true;
          }
    }
    // READ FOURTH ROW
    pinreg = (NRF_GPIO->IN)^0xffffffff;  // press is active high regardless of diode dir
    pinMode(rows[3], INPUT); 
    // PROCESS FOURTH ROW - Process 4th row right away...
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            btn_pressed = true;
            uint8_t keycode[6] = { 0 };
            keycode[0] = keymap[3*MATRIX_COLS + i];
            bluemicro_hid.keyboardReport(0, keycode);
            Serial.print("row: 3 Column");
            Serial.println(i);
            has_key = true;
          }
    }

    //Scanning done, disabling all columns - needed to save power
    for (int i = 0; i < MATRIX_COLS; ++i) {                             
        pinMode(columns[i], INPUT);                                     
    }
    if ( ! btn_pressed )
    {
      // send empty key report if previously has key pressed
      if (has_key) bluemicro_hid.keyboardRelease();
      has_key = false;
    }
  bluemicro_hid.processQueues(CONNECTION_MODE_AUTO);
    // poll gpio once each 10 ms
  delay(10);
}
