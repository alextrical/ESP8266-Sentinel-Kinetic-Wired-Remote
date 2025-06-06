/*
** Vent-Axia Remote Control using ESP8266
** Converted from PIC16F627A version with PCF8574 LCD adapter
*/
// Original: ©2021 bmd (brian@brianmarchant.com)
// ESP8266 conversion: 2025

//Requires the folowing Libraries to be installed:
//LiquidCrystal_PCF8574 V2.2.0

//Add ESP8266 support to arduino https://github.com/esp8266/Arduino
//Boards manager link: https://arduino.esp8266.com/stable/package_esp8266com_index.json

// *****************************************************************************
// INCLUDES
// *****************************************************************************
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

// *****************************************************************************
// PIN DEFINITIONS
// *****************************************************************************
// Button pins
#define BTN_DWN_PIN 0  // Replace with actual GPIO pins
#define BTN_UP_PIN  12
#define BTN_SET_PIN 14
#define BTN_FAN_PIN 13

// I2C pins on ESP8266 are:
// SDA - GPIO4 (D2)
// SCL - GPIO5 (D1)

// PCF8574 I2C address (typically 0x27 or 0x3F)
#define LCD_I2C_ADDR 0x27

// *****************************************************************************
// DEFINITIONS
// *****************************************************************************
typedef struct _BTN_STATE {
    uint8_t   pin;
    uint8_t   curr_state;
    uint8_t   last_state;
} BTN_STATE;

BTN_STATE btn_up;
BTN_STATE btn_dwn;
BTN_STATE btn_set;
BTN_STATE btn_fan;
uint8_t button_bits;
unsigned long button_press_start_time = 0; // Time when button was first pressed
bool button_pressed = false;               // Flag to track if any button is pressed
uint8_t commission_set;

// Time thresholds in milliseconds (converted from counter values)
#define SHORT_PRESS_THRESHOLD   2000  // ~0x2000 counter value
#define MEDIUM_PRESS_THRESHOLD  5000  // ~0x3FFF counter value
#define LONG_PRESS_THRESHOLD    8000  // ~0x4FFF counter value
#define EXTRA_LONG_THRESHOLD    10000 // ~0x7FFF counter value

bool data_read;
uint8_t i;

uint8_t uart_chr;
char oled_chr;
char oled_pos;
bool line1_active;
bool line2_active;

// Extra chr as it's easier for the array to be 1-based.
char line1[18];
char line2[18];

// Message sent to MVHR unit at start-up.
uint8_t alive_str[] = {0x04, 0x06, 0xFF, 0xFF, 0xFF, 0x10, 0xFC, 0xE8, '\0'};

#define DATA_DWN_BIT            0x01
#define DATA_UP_BIT             0x02
#define DATA_SET_BIT            0x04
#define DATA_FAN_BIT            0x08

#define BUTTON_MASK             0b00111001
#define NONE_PRESSED            0b00111001

#define BTN_STATE_PLACEHOLDER   0x00
#define CRC_PLACEHOLDER         0x00

#define BTN_STATE_BYTE_POS      5U
#define BTN_CRC_BYTE_POS        7U

#define COMMISSION_BTN_DELAY    5

// Data representing each possible key-press.
uint8_t btn_press_str[] = {0x04, 0x06, 0xFF, 0xFF, 0xFF, BTN_STATE_PLACEHOLDER, 0xFC, CRC_PLACEHOLDER, '\0'};
uint8_t btn_byte;

// Initialize LCD with PCF8574 I2C port expander
LiquidCrystal_PCF8574 lcd(LCD_I2C_ADDR);

// *****************************************************************************
// FUNCTIONS
// *****************************************************************************

// Write the two lines of text to the OLED display.
void update_oled(void)
{
    // Overwrite the display with an indication of button long-press
    // and a hint to move to the top menu item for them to be effective.
    unsigned long button_press_duration = button_pressed ? (millis() - button_press_start_time) : 0;
    
    if (button_press_duration >= SHORT_PRESS_THRESHOLD) {
        line1[16] = '^';
    }

    lcd.setCursor(0, 0);
    lcd.print(&line1[1]); // Line array indexes are 1-based.
    
    lcd.setCursor(0, 1);
    if ((btn_set.curr_state) && (button_press_duration >= LONG_PRESS_THRESHOLD)) {
        lcd.print("ENTER COMMISSION");
    }
    else {
        lcd.print(&line2[1]);
    }
}

// Initialise/Clear the state of a button.
void init_btn(BTN_STATE *btn, const uint8_t pin)
{
    btn->pin = pin;
    btn->curr_state = 0;
    btn->last_state = 0;
    pinMode(pin, INPUT_PULLUP);
}

// Update the state of a button from its pin.
void update_btn(BTN_STATE *btn)
{
    btn->last_state = btn->curr_state;
    btn->curr_state = !digitalRead(btn->pin); // Inverted because of pull-up
}

// Check if any button is pressed
bool any_button_pressed() {
    return (btn_up.curr_state || btn_dwn.curr_state || btn_set.curr_state || btn_fan.curr_state);
}

// Send a string of bytes over UART
void uart_write_text(uint8_t* str) {
    while(*str) {
        Serial.write(*str++);
    }
}

// Check if UART TX is idle
bool uart_tx_idle() {
    return Serial.availableForWrite() > 0;
}

// *****************************************************************************
// SETUP
// *****************************************************************************
void setup()
{
    // Initialize I2C
    Wire.begin();
    
    // Initialize LCD with PCF8574
    lcd.begin(16, 2);
    lcd.setBacklight(255); // Turn on backlight
    lcd.home();
    lcd.clear();
    
    // Initialize Hardware UART module (the link to the MVHR unit).
    Serial.begin(9600);
    
    lcd.setCursor(0, 0);
    lcd.print("VENT-AXIA Remote");
    lcd.setCursor(0, 1);
    lcd.print("V1.0 ESP");
    
    // Show that the display is active with a little animation.
    for (i = 10; i <= 16; i++) {
        lcd.setCursor(i-1, 1);
        lcd.print(".");
        delay(300);
    }
    delay(1000);
    
    // Send an "alive" message to the MVHR unit:
    uart_write_text(alive_str);

    // Terminate the two line strings and clear the display.
    line1[17] = '\0';
    line2[17] = '\0';
    lcd.clear();
    
    // Initialise the button states.
    init_btn(&btn_dwn, BTN_DWN_PIN);
    init_btn(&btn_up, BTN_UP_PIN);
    init_btn(&btn_set, BTN_SET_PIN);
    init_btn(&btn_fan, BTN_FAN_PIN);
    button_press_start_time = 0;
    button_pressed = false;
    commission_set = 0;
    btn_byte = 0x00;

    // Initialise the character counters.
    line1_active = false;
    line2_active = false;
    oled_pos = 0;
}

// *****************************************************************************
// MAIN PROGRAM LOOP
// *****************************************************************************
void loop()
{
    //
    // Priority 1 : Check for characters received from the MVHR unit.
    //
    if (Serial.available()) {
        uart_chr = Serial.read();
        data_read = true;
        
        switch (uart_chr) {
        case 0x15:
            // NAK - start of line 1
            line1_active = true;
            oled_pos = 0;
            break;

        case 0x16:
            // SYN - end of line 1, start of line 2
            line2_active = true;
            oled_pos = 0;
            break;

        default:
            // Record the characters for Line#1 (top)
            if ((line1_active) && (oled_pos > 0)) {
                line1[oled_pos] = uart_chr;
                
                if (oled_pos >= 16) {
                    line1_active = false;
                }
            }
            
            // Record the characters for Line#2 (bottom)
            if ((line2_active) && (oled_pos > 0)) {
                line2[oled_pos] = uart_chr;

                if (oled_pos >= 16) {
                    line2_active = false;
                    
                    update_oled();
                }
            }
            break;
        }
        
        oled_pos++;
    }
    else {
        data_read = false;
        //
        // While we don't have data to read, do something else useful.
        //
        
        // Read the button states
        update_btn(&btn_set);
        update_btn(&btn_dwn);
        update_btn(&btn_up);
        update_btn(&btn_fan);

        // Track button press duration using millis()
        bool currently_pressed = any_button_pressed();
        
        // Button state transition detection
        if (currently_pressed && !button_pressed) {
            // Button just pressed - record start time
            button_press_start_time = millis();
            button_pressed = true;
        }
        else if (!currently_pressed && button_pressed) {
            // Button just released
            button_pressed = false;
        }
        
        // Calculate press duration if a button is currently pressed
        unsigned long button_press_duration = button_pressed ? (millis() - button_press_start_time) : 0;

        if (uart_tx_idle()) {
            // Clear the state all all OR'd buttons.
            btn_press_str[BTN_STATE_BYTE_POS] = 0x00;

            // "DOWN" arrow pressed (edge trigger - sent only once).
            // PLUS: UP+DWN for 5s+ resets "Check Filter" message.
            if ((btn_dwn.curr_state && !btn_dwn.last_state) ||
                ((btn_dwn.curr_state) && (button_press_duration >= MEDIUM_PRESS_THRESHOLD))) {
                btn_press_str[BTN_STATE_BYTE_POS] |= DATA_DWN_BIT;
            }

            // "UP" arrow pressed (edge trigger - sent only once).
            // PLUS: UP+DWN for 5s+ resets "Check Filter" message.
            // PLUS: UP 5s+ should also exit from Commission screens.
            if ((btn_up.curr_state && !btn_up.last_state) ||
                ((btn_up.curr_state) && (button_press_duration >= MEDIUM_PRESS_THRESHOLD))) {
                btn_press_str[BTN_STATE_BYTE_POS] |= DATA_UP_BIT;
            }

            // "FAN" button pressed (edge trigger - sent only once).
            // PLUS long-press pass-through to set PURGE mode.
            if ((btn_fan.curr_state && !btn_fan.last_state) ||
                ((btn_fan.curr_state) && (button_press_duration >= EXTRA_LONG_THRESHOLD))) {
                btn_press_str[BTN_STATE_BYTE_POS] |= DATA_FAN_BIT;
            }
            
            // "SET" button pressed (edge trigger - sent only once).
            if (btn_set.curr_state && !btn_set.last_state) {
                btn_press_str[BTN_STATE_BYTE_POS] |= DATA_SET_BIT;
            }
            // Add in a "long-press" of the SET button to send the special
            // commissioning mode sequence of SET then SET+DWN+UP together.
            if ((btn_set.curr_state) && (button_press_duration >= EXTRA_LONG_THRESHOLD)) {
                btn_press_str[BTN_STATE_BYTE_POS] |= DATA_SET_BIT;
                if (commission_set > COMMISSION_BTN_DELAY) {
                    btn_press_str[BTN_STATE_BYTE_POS] |= DATA_DWN_BIT;
                    btn_press_str[BTN_STATE_BYTE_POS] |= DATA_UP_BIT;
                }
            }
            else {
                commission_set = 0;
            }

            // Calculate the checksum byte and send the byte string.
            if (btn_press_str[BTN_STATE_BYTE_POS] != 0x00) {
                // Usefully the last-sent value will be remembered and
                // is used for diagnostic output. CRC indicates buttons.
                btn_press_str[BTN_CRC_BYTE_POS] = 0xF8 - btn_press_str[BTN_STATE_BYTE_POS];

                uart_write_text(btn_press_str);
                delay(15);

                if (commission_set < 255) {
                    commission_set++;
                }

                // Reset/inhibit the reception of display data.
                line1_active = false;
                line2_active = false;
            }
        }
    }
}
