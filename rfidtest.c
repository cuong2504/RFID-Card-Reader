#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// --- CPU Frequency ---
#ifndef F_CPU
#define F_CPU 16000000UL
#warning "F_CPU not defined, assuming 16MHz for delays and I2C."
#endif

// --- Pin definitions ---
#define MOTION_PIN  PC0  // Input pin for Motion sensor (PIR)
#define RELAY_PIN   PC1  // Output pin for Relay control
#define BUZZER_PIN  PC2  // Output pin for Buzzer control
#define LED1        PD0  // Output pin for Status LED 1 (Green - Success)
#define LED2        PD1  // Output pin for Status LED 2 (Red - Error / No Card Timeout)
#define LED3        PD2  // Output pin for Status LED 3 (Yellow - Waiting)
#define LED4        PD3  // Output pin for Status LED 4 (Unused but defined)
#define LCD_RS      PB0  // LCD Register Select pin
#define LCD_EN      PB1  // LCD Enable pin
#define LCD_D4      PD4  // LCD Data pin 4
#define LCD_D5      PD5  // LCD Data pin 5
#define LCD_D6      PD6  // LCD Data pin 6
#define LCD_D7      PD7  // LCD Data pin 7

// --- I2C Settings ---
#define RTC_ADDRESS 0x68
#define RTC_WRITE_ADDR (RTC_ADDRESS << 1)
#define RTC_READ_ADDR  ((RTC_ADDRESS << 1) | 0x01)

// --- SPI Pins (Placeholders) ---
#define SPI_SS      PB2
#define SPI_MOSI    PB3
#define SPI_MISO    PB4
#define SPI_SCK     PB5

// --- Global variables for LCD display ---
char date[20];
char time[20];
char relayStatus[25];
char cardStatus[25]; // Holds the current RFID status text for LCD

// --- RFID Status State Machine ---
typedef enum {
    RFID_STATE_WAITING,          // Waiting for a card scan event (Yellow LED)
    RFID_STATE_TIMEOUT_NO_CARD,  // Timeout reached while waiting (Red LED, temporary)
    RFID_STATE_SUCCESS,          // Card scan successful (Green LED, temporary)
    RFID_STATE_ERROR             // Card scan failed (Red LED, temporary)
} RFID_Status_t;

// --- Timing Constants (based on LOOP_DELAY_MS) ---
#define LOOP_DELAY_MS 500
// Timeouts in number of loop iterations:
const uint16_t NO_CARD_TIMEOUT_LOOPS = (5000 / LOOP_DELAY_MS);       // 5 seconds
const uint16_t SCAN_EVENT_INTERVAL_LOOPS = (7000 / LOOP_DELAY_MS);    // Simulate scan every 7 seconds
const uint16_t DISPLAY_DURATION_LOOPS = (3000 / LOOP_DELAY_MS);    // Display Success/Error/TimeoutNoCard for 3 seconds
const uint8_t SHUTDOWN_TIMEOUT_COUNT = 3;                           // Shutdown after 3 consecutive timeouts
const uint16_t DOUBLE_BEEP_DELAY_MS = 75;                           // Delay between double beeps

// --- Function Prototypes ---
void init(void);
void GPIO_init(void);
void SPI_init(void);
void I2C_init(void);
void LCD_init(void);
void LCD_command_4bit_init(uint8_t cmd_nibble);
uint8_t SPI_transfer(uint8_t data);
void LCD_command(uint8_t cmd);
void LCD_data(uint8_t data);
void LCD_print(const char *str);
void LCD_clear(void);
void LCD_set_cursor(uint8_t col, uint8_t row);
uint8_t I2C_start(uint8_t address);
void I2C_stop(void);
uint8_t I2C_write(uint8_t data);
uint8_t I2C_read_ack(void);
uint8_t I2C_read_nack(void);
void getDateTime(void);
uint8_t bcd_to_decimal(uint8_t bcd);
// void setDateTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month, uint8_t year);
// uint8_t decimal_to_bcd(uint8_t dec);
void updateLCD(void);
bool getRelayStatus(void);
void processCardStatus(void);
void beep(void);
void ledOn(uint8_t led_pin_define);
void ledOff(uint8_t led_pin_define);
void system_shutdown(void); // Shutdown function


// --- Main Function ---
int main(void) {
    init();
    // Optional: setDateTime(00, 30, 14, 5, 15, 3, 25);
    while (1) {
        // --- Motion Sensor & Relay Logic ---
         if (PINC & (1 << MOTION_PIN)) { PORTC |= (1 << RELAY_PIN); }
         else { PORTC &= ~(1 << RELAY_PIN); }

        processCardStatus(); // Process RFID state machine, LEDs, status text
        updateLCD();         // Update LCD display

        _delay_ms(LOOP_DELAY_MS);
    }
    return 0; // Unreachable
}

// --- Initialization Functions ---
void init(void) {
    GPIO_init(); // Sets initial LED state (Yellow ON)
    SPI_init();
    I2C_init();
    LCD_init();
    LCD_clear(); LCD_print("System Starting..."); _delay_ms(1000);
    LCD_set_cursor(0,1); LCD_print("Testing Buzzer..."); beep(); _delay_ms(500); // Single beep on startup
    LCD_clear();
}

void GPIO_init(void) {
    DDRC &= ~(1 << MOTION_PIN); DDRC |= (1 << RELAY_PIN) | (1 << BUZZER_PIN);
    PORTC &= ~((1 << RELAY_PIN) | (1 << BUZZER_PIN));
    DDRD |= (1 << LED1) | (1 << LED2) | (1 << LED3) | (1 << LED4);
    // Initial State: Waiting
    ledOff(LED1); ledOff(LED2); ledOn(LED3); ledOff(LED4);
    strcpy(cardStatus, "Status: Waiting... "); // Initial status text
}

void SPI_init(void) {
    DDRB |= (1 << SPI_SS) | (1 << SPI_MOSI) | (1 << SPI_SCK);
    DDRB &= ~(1 << SPI_MISO);
    PORTB |= (1 << SPI_SS);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}
void I2C_init(void) {
    TWSR = 0x00;
    TWBR = ((F_CPU / 100000L) - 16) / 2;
    TWCR = (1 << TWEN);
}
void LCD_init(void) {
    DDRD |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);
    DDRB |= (1 << LCD_RS) | (1 << LCD_EN);
    _delay_ms(50);
    LCD_command_4bit_init(0x30); _delay_ms(5); LCD_command_4bit_init(0x30); _delay_us(150);
    LCD_command_4bit_init(0x30); _delay_us(150); LCD_command_4bit_init(0x20); _delay_us(150);
    LCD_command(0x28); LCD_command(0x08); LCD_command(0x01); _delay_ms(2);
    LCD_command(0x06); LCD_command(0x0C);
}

// --- SPI Functions ---
uint8_t SPI_transfer(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

// --- I2C (TWI) Functions ---
uint8_t I2C_start(uint8_t address){
    uint8_t status; TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); status = TWSR & 0xF8;
    if ((status != 0x08) && (status != 0x10)) return 1;
    TWDR = address; TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); status = TWSR & 0xF8;
    if ((status != 0x18) && (status != 0x40)) return 1; return 0;
 }
void I2C_stop(void){ TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO); }
uint8_t I2C_write(uint8_t data){
    uint8_t status; TWDR = data; TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); status = TWSR & 0xF8;
    if (status != 0x28) return 1; return 0;
}
uint8_t I2C_read_ack(void){
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT))); return TWDR;
}
uint8_t I2C_read_nack(void){
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); return TWDR;
}

// --- LCD Functions ---
void LCD_command_4bit_init(uint8_t cmd_nibble){
    PORTB &= ~(1 << LCD_RS); PORTD = (PORTD & 0x0F) | (cmd_nibble & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1); PORTB &= ~(1 << LCD_EN);
}
void LCD_command(uint8_t cmd){
    PORTB &= ~(1 << LCD_RS); PORTD = (PORTD & 0x0F) | (cmd & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1); PORTB &= ~(1 << LCD_EN); _delay_us(1);
    PORTD = (PORTD & 0x0F) | ((cmd << 4) & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1); PORTB &= ~(1 << LCD_EN);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2); else _delay_us(40);
}
void LCD_data(uint8_t data){
     PORTB |= (1 << LCD_RS); PORTD = (PORTD & 0x0F) | (data & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1); PORTB &= ~(1 << LCD_EN); _delay_us(1);
    PORTD = (PORTD & 0x0F) | ((data << 4) & 0xF0);
    PORTB |= (1 << LCD_EN); _delay_us(1); PORTB &= ~(1 << LCD_EN);
    _delay_us(40);
}
void LCD_print(const char *str){ while (*str) LCD_data(*str++); }
void LCD_clear(void){ LCD_command(0x01); }
void LCD_set_cursor(uint8_t col, uint8_t row){
    const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row >= 4) row = 3; if (col >= 20) col = 19;
    LCD_command(0x80 | (row_offsets[row] + col));
}

// --- RTC Functions ---
void getDateTime(void){
    uint8_t sec, min, hour, day, date_of_month, month, year;
    if (I2C_start(RTC_WRITE_ADDR)) { goto i2c_read_error; }
    if (I2C_write(0x00)) { goto i2c_read_error; }
    if (I2C_start(RTC_READ_ADDR)) { goto i2c_read_error; }
    sec=I2C_read_ack(); min=I2C_read_ack(); hour=I2C_read_ack(); day=I2C_read_ack();
    date_of_month=I2C_read_ack(); month=I2C_read_ack(); year=I2C_read_nack(); I2C_stop();
    if (sec & 0x80) { strcpy(date, "RTC HALTED"); strcpy(time, "  :  :  "); return; }
    sec=bcd_to_decimal(sec & 0x7F); min=bcd_to_decimal(min); hour=bcd_to_decimal(hour & 0x3F);
    date_of_month=bcd_to_decimal(date_of_month); month=bcd_to_decimal(month & 0x1F); year=bcd_to_decimal(year);
    sprintf(date, "%02d/%02d/20%02d", date_of_month, month, year); sprintf(time, "%02d:%02d:%02d", hour, min, sec); return;
i2c_read_error: I2C_stop(); strcpy(date, "I2C ERROR"); strcpy(time, "  :  :  ");
}
uint8_t bcd_to_decimal(uint8_t bcd){ return ((bcd >> 4) * 10) + (bcd & 0x0F); }
/* // Optional Set Time Functions
uint8_t decimal_to_bcd(uint8_t dec){ return ((dec / 10) << 4) | (dec % 10); }
void setDateTime(uint8_t sec,uint8_t min,uint8_t hour,uint8_t dayOfWeek,uint8_t dayOfMonth,uint8_t month,uint8_t year){...}
*/

// --- Application Logic Functions ---

void ledOn(uint8_t led_pin_define) { PORTD |= (1 << led_pin_define); }
void ledOff(uint8_t led_pin_define) { PORTD &= ~(1 << led_pin_define); }

void beep(void) {
    PORTC |= (1 << BUZZER_PIN); _delay_ms(100); PORTC &= ~(1 << BUZZER_PIN);
}

// System Shutdown Function
void system_shutdown(void) {
    ledOff(LED1); ledOff(LED2); ledOff(LED3); ledOff(LED4);
    PORTC &= ~(1 << RELAY_PIN);
    PORTC &= ~(1 << BUZZER_PIN);
    LCD_clear();
    LCD_set_cursor(0, 0); LCD_print("Power OFF         ");
    LCD_set_cursor(0, 1); LCD_print("No card detected  ");
    LCD_set_cursor(0, 2); LCD_print("                    ");
    LCD_set_cursor(0, 3); LCD_print("                    ");
    while(1) { /* Halt */ }
}

// Process simulated RFID status, control LEDs, set status text, and handle beeps
void processCardStatus(void) {
    static RFID_Status_t current_rfid_state = RFID_STATE_WAITING; // Initial state
    static uint16_t state_timer = 0; // Timer for state durations / timeouts
    static uint8_t no_card_timeout_counter = 0; // Counter for consecutive timeouts

    state_timer++; // Increment timer every loop iteration

    RFID_Status_t next_state = current_rfid_state; // Assume no change initially

    // --- State Machine Logic ---
    switch (current_rfid_state) {
        case RFID_STATE_WAITING:
            ledOn(LED3); ledOff(LED1); ledOff(LED2); // Yellow ON
            strcpy(cardStatus, "Status: Waiting... ");

            if (state_timer >= SCAN_EVENT_INTERVAL_LOOPS) {
                no_card_timeout_counter = 0;
                if ((state_timer / 5) % 2 == 0) { next_state = RFID_STATE_SUCCESS; }
                else { next_state = RFID_STATE_ERROR; }
            }
            else if (state_timer >= NO_CARD_TIMEOUT_LOOPS) {
                no_card_timeout_counter++;
                if (no_card_timeout_counter >= SHUTDOWN_TIMEOUT_COUNT) { system_shutdown(); }
                next_state = RFID_STATE_TIMEOUT_NO_CARD;
            }
            break;

        case RFID_STATE_TIMEOUT_NO_CARD:
            ledOff(LED3); ledOff(LED1); ledOn(LED2); // Red ON
            strcpy(cardStatus, "No Card Detected ");

            if (state_timer >= DISPLAY_DURATION_LOOPS) {
                 next_state = RFID_STATE_WAITING;
            }
            else if (state_timer >= SCAN_EVENT_INTERVAL_LOOPS) { // Allow recovery during display
                 no_card_timeout_counter = 0;
                 if ((state_timer / 5) % 2 == 0) { next_state = RFID_STATE_SUCCESS; }
                 else { next_state = RFID_STATE_ERROR; }
            }
            break;

        case RFID_STATE_SUCCESS:
            ledOff(LED3); ledOn(LED1); ledOff(LED2); // Green ON
            strcpy(cardStatus, "Status: Scan OK    ");

            if (state_timer >= DISPLAY_DURATION_LOOPS) {
                next_state = RFID_STATE_WAITING;
            }
            break;

        case RFID_STATE_ERROR:
             ledOff(LED3); ledOff(LED1); ledOn(LED2); // Red ON
             strcpy(cardStatus, "Status: Scan Error ");

            if (state_timer >= DISPLAY_DURATION_LOOPS) {
                 next_state = RFID_STATE_WAITING;
            }
            break;
    }

    // --- Handle State Transition and Beeping ---
    if (next_state != current_rfid_state) {
        // Beep according to the *next* state we are entering
        if (next_state == RFID_STATE_SUCCESS ||
            next_state == RFID_STATE_ERROR ||
            next_state == RFID_STATE_TIMEOUT_NO_CARD) // Green or Red LED states
        {
            // Double beep
            beep();
            _delay_ms(DOUBLE_BEEP_DELAY_MS); // Short delay between beeps
            beep();
        }
        else // Entering RFID_STATE_WAITING (Yellow LED)
        {
            // Single beep
            beep();
        }

        // Update state and reset timer
        current_rfid_state = next_state;
        state_timer = 0;

    }

    // Final check for waiting text (if just entered waiting state)
    if (current_rfid_state == RFID_STATE_WAITING && state_timer == 0) {
         strcpy(cardStatus, "Status: Waiting... ");
    }
}


void updateLCD(void) {
    getDateTime();
    bool relayIsOn = getRelayStatus();
    // processCardStatus() is called in main loop, updates 'cardStatus' and LEDs

    sprintf(relayStatus, "Relay Status: %s ", relayIsOn ? "ON" : "OFF");

    LCD_set_cursor(0, 0); LCD_print(date); LCD_print(" "); LCD_print(time); LCD_print(" ");
    LCD_set_cursor(0, 1); LCD_print(relayStatus); LCD_print("          ");
    LCD_set_cursor(0, 2); LCD_print(cardStatus); LCD_print("     "); // Adjust spaces if needed
    LCD_set_cursor(0, 3); LCD_print("                    ");
}

bool getRelayStatus(void) {
    return (PORTC & (1 << RELAY_PIN));
}

// --- END OF CODE ---
