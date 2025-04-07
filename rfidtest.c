#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// Pin definitions
#define MOTION_PIN  PC0  // Motion sensor input
#define RELAY_PIN   PC1  // Relay control output
#define BUZZER_PIN  PC2  // Buzzer control output
#define LED1        PD0  // Status LED 1
#define LED2        PD1  // Status LED 2
#define LED3        PD2  // Status LED 3
#define LED4        PD3  // Status LED 4
#define LCD_RS      PB0  // LCD Register Select
#define LCD_EN      PB1  // LCD Enable
#define LCD_D4      PD4  // LCD Data 4
#define LCD_D5      PD5  // LCD Data 5
#define LCD_D6      PD6  // LCD Data 6
#define LCD_D7      PD7  // LCD Data 7

// I2C address for DS1307 RTC
#define RTC_ADDRESS 0x68

// SPI pins (RC522 RFID)
#define SPI_SS      PB2
#define SPI_MOSI    PB3
#define SPI_MISO    PB4
#define SPI_SCK     PB5

// Global variables for LCD display
char date[20];        // e.g., "15/03/2025"
char time[20];        // e.g., "14:30:45"
char relayStatus[10]; // e.g., "Relay: ON"
char cardStatus[20];  // e.g., "ID: 1234 - Authorized"

// Function prototypes
void SPI_init(void);
uint8_t SPI_transfer(uint8_t data);
void LCD_init(void);
void LCD_command(uint8_t cmd);
void LCD_data(uint8_t data);
void LCD_print(const char *str);
void LCD_clear(void);
void LCD_set_cursor(uint8_t col, uint8_t row);
void I2C_init(void);
uint8_t I2C_start(uint8_t address);
void I2C_stop(void);
uint8_t I2C_write(uint8_t data);
uint8_t I2C_read_ack(void);
uint8_t I2C_read_nack(void);
void init(void);
void updateLCD(void);
void getDateTime(void);
bool getRelayStatus(void);
void getCardStatus(void);
void beep(void);
void blinkLEDs(void);

// Main function
int main(void) {
    init(); // Initialize all components
    while (1) {
        // Check motion sensor
        if (PINC & (1 << MOTION_PIN)) {
            PORTC |= (1 << RELAY_PIN);  // Turn relay ON
            beep();                     // Beep when motion detected
        } else {
            PORTC &= ~(1 << RELAY_PIN); // Turn relay OFF
        }

        // Update RFID status (placeholder)
        getCardStatus();

        // Update LCD display
        updateLCD();

        // Blink LEDs for status
        blinkLEDs();

        _delay_ms(1000); // Update every second
    }
    return 0;
}

// Initialize all components
void init(void) {
    // Set motion sensor as input, relay and buzzer as outputs
    DDRC &= ~(1 << MOTION_PIN);     // PC0 input (motion)
    DDRC |= (1 << RELAY_PIN) | (1 << BUZZER_PIN); // PC1 (relay), PC2 (buzzer) outputs

    // Set LEDs as outputs
    DDRD |= (1 << LED1) | (1 << LED2) | (1 << LED3) | (1 << LED4);

    // Initialize SPI, I2C, and LCD
    SPI_init();
    I2C_init();
    LCD_init();
}

// --- SPI Functions ---
void SPI_init(void) {
    // Set MOSI (PB3), SCK (PB5), and SS (PB2) as outputs, MISO (PB4) as input
    DDRB |= (1 << SPI_SS) | (1 << SPI_MOSI) | (1 << SPI_SCK);
    DDRB &= ~(1 << SPI_MISO);
    // Enable SPI, Master mode, clock rate fck/16
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t SPI_transfer(uint8_t data) {
    SPDR = data;                    // Load data into SPI data register
    while (!(SPSR & (1 << SPIF)));  // Wait for transmission to complete
    return SPDR;                    // Return received data
}

// --- I2C Functions (TWI) ---
void I2C_init(void) {
    // Set SCL frequency to 100kHz (with 16MHz F_CPU)
    TWBR = 72;
    TWSR = 0;  // Prescaler = 1
}

uint8_t I2C_start(uint8_t address) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != 0x08) return 1;  // Error if not start condition
    TWDR = address;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    uint8_t twst = TWSR & 0xF8;
    if ((twst != 0x18) && (twst != 0x40)) return 1;  // Error if not ACK
    return 0;
}

void I2C_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while (TWCR & (1 << TWSTO));
}

uint8_t I2C_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != 0x28) return 1;  // Error if not ACK
    return 0;
}

uint8_t I2C_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t I2C_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// --- LCD Functions ---
void LCD_init(void) {
    // Set LCD pins as outputs
    DDRD |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);
    DDRB |= (1 << LCD_RS) | (1 << LCD_EN);

    _delay_ms(50);      // Wait for LCD to power up
    LCD_command(0x03);  // Start initialization
    _delay_ms(5);
    LCD_command(0x03);
    _delay_us(150);
    LCD_command(0x03);
    LCD_command(0x02);  // Set 4-bit mode
    LCD_command(0x28);  // 2 lines, 5x8 font (adjustable for 20x4)
    LCD_command(0x0C);  // Display ON, cursor OFF
    LCD_command(0x06);  // Entry mode
    LCD_command(0x01);  // Clear display
    _delay_ms(2);
}

void LCD_command(uint8_t cmd) {
    PORTB &= ~(1 << LCD_RS);               // RS low for command
    PORTD = (PORTD & 0x0F) | (cmd & 0xF0); // High nibble
    PORTB |= (1 << LCD_EN);                // Pulse EN
    _delay_us(1);
    PORTB &= ~(1 << LCD_EN);
    _delay_us(200);
    PORTD = (PORTD & 0x0F) | ((cmd << 4) & 0xF0); // Low nibble
    PORTB |= (1 << LCD_EN);
    _delay_us(1);
    PORTB &= ~(1 << LCD_EN);
    _delay_ms(2);
}

void LCD_data(uint8_t data) {
    PORTB |= (1 << LCD_RS);                 // RS high for data
    PORTD = (PORTD & 0x0F) | (data & 0xF0); // High nibble
    PORTB |= (1 << LCD_EN);                 // Pulse EN
    _delay_us(1);
    PORTB &= ~(1 << LCD_EN);
    _delay_us(200);
    PORTD = (PORTD & 0x0F) | ((data << 4) & 0xF0); // Low nibble
    PORTB |= (1 << LCD_EN);
    _delay_us(1);
    PORTB &= ~(1 << LCD_EN);
    _delay_us(100);
}

void LCD_print(const char *str) {
    while (*str) {
        LCD_data(*str++);
    }
}

void LCD_clear(void) {
    LCD_command(0x01);
    _delay_ms(2);
}

void LCD_set_cursor(uint8_t col, uint8_t row) {
    uint8_t addr;
    switch (row) {
        case 0: addr = 0x00 + col; break; // Line 1
        case 1: addr = 0x40 + col; break; // Line 2
        case 2: addr = 0x14 + col; break; // Line 3
        case 3: addr = 0x54 + col; break; // Line 4
        default: addr = 0x00;
    }
    LCD_command(0x80 | addr);
}

// Update LCD with current information
void updateLCD(void) {
    getDateTime(); // Fetch date and time from RTC
    bool relayOn = getRelayStatus();
    sprintf(relayStatus, "Relay: %s", relayOn ? "ON" : "OFF");

    LCD_clear();
    LCD_set_cursor(0, 0); LCD_print(date);       // Line 1: Date
    LCD_set_cursor(0, 1); LCD_print(time);       // Line 2: Time
    LCD_set_cursor(0, 2); LCD_print(relayStatus); // Line 3: Relay status
    LCD_set_cursor(0, 3); LCD_print(cardStatus);  // Line 4: RFID status
}

// Get date and time from DS1307 RTC (placeholder)
void getDateTime(void) {
    // Implement I2C communication to read RTC registers
    // For now, use dummy data
    strcpy(date, "15/03/2025");
    strcpy(time, "14:30:45");
}

// Get relay status
bool getRelayStatus(void) {
    return (PORTC & (1 << RELAY_PIN)) ? true : false;
}

// Get RFID card status (placeholder)
void getCardStatus(void) {
    // Implement SPI communication with RC522
    // For now, use dummy data
    strcpy(cardStatus, "ID: 1234 - Authorized");
}

// Trigger buzzer for feedback
void beep(void) {
    PORTC |= (1 << BUZZER_PIN);  // Turn buzzer ON
    _delay_ms(100);              // Beep for 100ms
    PORTC &= ~(1 << BUZZER_PIN); // Turn buzzer OFF
}

// Blink LEDs for status indication
void blinkLEDs(void) {
    static bool ledState = false;
    ledState = !ledState;
    if (ledState) {
        PORTD |= (1 << LED1) | (1 << LED2) | (1 << LED3) | (1 << LED4);
    } else {
        PORTD &= ~((1 << LED1) | (1 << LED2) | (1 << LED3) | (1 << LED4));
    }
}
