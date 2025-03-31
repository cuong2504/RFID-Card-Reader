#define F_CPU 16000000UL  // 16 MHz clock (adjust if different)

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

// Pin definitions for ATmega328P
#define SS_PIN   PB2  // PORTB2 (D10 equivalent)
#define RST_PIN  PB1  // PORTB1 (D9 equivalent)

// SPI pins
#define SPI_PORT PORTB
#define SPI_DDR  DDRB
#define SPI_MOSI PB3  // MOSI
#define SPI_MISO PB4  // MISO
#define SPI_SCK  PB5  // SCK
#define SPI_SS   PB2  // Slave Select

// MFRC522 registers (simplified subset)
#define MFRC522_REG_COMMAND    0x01
#define MFRC522_REG_COM_IEN    0x02
#define MFRC522_REG_COM_IRQ    0x04
#define MFRC522_REG_FIFO_DATA  0x09
#define MFRC522_REG_FIFO_LEVEL 0x0A
#define MFRC522_REG_CONTROL    0x0C
#define MFRC522_REG_BIT_FRAMING 0x0D
#define MFRC522_REG_MODE       0x11
#define MFRC522_REG_TX_CONTROL 0x14
#define MFRC522_REG_TX_ASK     0x15
#define MFRC522_REG_STATUS2    0x08

// MFRC522 commands
#define MFRC522_CMD_IDLE       0x00
#define MFRC522_CMD_TRANSCEIVE 0x0C
#define MFRC522_CMD_SOFT_RESET 0x0F

// PICC commands
#define PICC_CMD_REQA    0x26
#define PICC_CMD_SEL_CL1 0x93

// SPI functions
void spi_init() {
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);
    SPI_DDR &= ~(1 << SPI_MISO);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // SPI enabled, Master, fck/16
}

uint8_t spi_transmit(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

// MFRC522 low-level functions
void mfrc522_write(uint8_t addr, uint8_t val) {
    SPI_PORT &= ~(1 << SPI_SS); // SS low
    spi_transmit((addr << 1) & 0x7E); // Address format: 0b0AAAAAAR (W=0)
    spi_transmit(val);
    SPI_PORT |= (1 << SPI_SS); // SS high
}

uint8_t mfrc522_read(uint8_t addr) {
    SPI_PORT &= ~(1 << SPI_SS); // SS low
    spi_transmit(((addr << 1) & 0x7E) | 0x80); // Address format: 0b1AAAAAAR (R=1)
    uint8_t val = spi_transmit(0x00); // Dummy byte to read
    SPI_PORT |= (1 << SPI_SS); // SS high
    return val;
}

// MFRC522 basic operations
void mfrc522_init() {
    // Reset
    SPI_PORT |= (1 << RST_PIN); // RST high
    _delay_ms(50);
    mfrc522_write(MFRC522_REG_COMMAND, MFRC522_CMD_SOFT_RESET);
    _delay_ms(50);

    // Configure
    mfrc522_write(MFRC522_REG_TX_CONTROL, 0x03); // Enable antenna
    mfrc522_write(MFRC522_REG_MODE, 0x3D);       // CRC preset
    mfrc522_write(MFRC522_REG_TX_ASK, 0x40);     // 100% ASK modulation
}

uint8_t mfrc522_request(uint8_t* buffer, uint8_t* length) {
    mfrc522_write(MFRC522_REG_BIT_FRAMING, 0x07); // 7-bit frame
    buffer[0] = PICC_CMD_REQA;
    *length = 1;
    
    mfrc522_write(MFRC522_REG_COMMAND, MFRC522_CMD_TRANSCEIVE);
    mfrc522_write(MFRC522_REG_CONTROL, 0x80); // Start transmission
    
    _delay_ms(10);
    if (mfrc522_read(MFRC522_REG_COM_IRQ) & 0x01) { // Rx complete
        *length = mfrc522_read(MFRC522_REG_FIFO_LEVEL);
        for (uint8_t i = 0; i < *length; i++) {
            buffer[i] = mfrc522_read(MFRC522_REG_FIFO_DATA);
        }
        return 1; // Card present
    }
    return 0; // No card
}

uint8_t mfrc522_select(uint8_t* uid, uint8_t* length) {
    uint8_t buffer[9] = {PICC_CMD_SEL_CL1, 0x20};
    *length = 2;
    
    mfrc522_write(MFRC522_REG_BIT_FRAMING, 0x00); // Full byte
    mfrc522_write(MFRC522_REG_FIFO_LEVEL, 0x80);  // Flush FIFO
    for (uint8_t i = 0; i < *length; i++) {
        mfrc522_write(MFRC522_REG_FIFO_DATA, buffer[i]);
    }
    
    mfrc522_write(MFRC522_REG_COMMAND, MFRC522_CMD_TRANSCEIVE);
    mfrc522_write(MFRC522_REG_CONTROL, 0x80);
    
    _delay_ms(10);
    if (mfrc522_read(MFRC522_REG_COM_IRQ) & 0x01) {
        *length = mfrc522_read(MFRC522_REG_FIFO_LEVEL);
        for (uint8_t i = 0; i < *length; i++) {
            uid[i] = mfrc522_read(MFRC522_REG_FIFO_DATA);
        }
        return 1; // UID read
    }
    return 0;
}

// UART functions
void uart_init(uint16_t baud) {
    uint16_t ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_print(const char* str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

void setup() {
    uart_init(9600);
    spi_init();
    DDRB |= (1 << SS_PIN) | (1 << RST_PIN);
    SPI_PORT |= (1 << SPI_SS); // SS high initially
    mfrc522_init();
    uart_print("Approximate your card to the reader...\n");
}

void loop() {
    uint8_t buffer[16];
    uint8_t length;
    
    // Look for new cards
    if (!mfrc522_request(buffer, &length)) {
        return;
    }
    
    // Select card and read UID
    uint8_t uid[10];
    if (!mfrc522_select(uid, &length)) {
        return;
    }
    
    // Show UID
    uart_print("UID tag :");
    char content[32] = "";
    char temp[4];
    for (uint8_t i = 0; i < 4; i++) { // Assuming 4-byte UID
        uart_transmit(uid[i] < 0x10 ? ' 0' : ' ');
        sprintf(temp, "%X", uid[i]);
        uart_print(temp);
        strcat(content, uid[i] < 0x10 ? " 0" : " ");
        strcat(content, temp);
    }
    uart_print("\nMessage : ");
    
    // Check UID
    if (strcmp(content + 1, "BD 31 15 2B") == 0) {
        uart_print("Authorized access\n\n");
        _delay_ms(3000);
    } else {
        uart_print("Access denied\n");
        _delay_ms(3000);
    }
}

int main(void) {
    setup();
    while (1) {
        loop();
    }
    return 0;
}
