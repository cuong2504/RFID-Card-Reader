#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// ??nh ngh?a chân (ánh x? t? chân Arduino sang c?ng ATmega328P)
#define GREEN_LED   PD7  // Chân 7
#define BLUE_LED    PD6  // Chân 6
#define RED_LED     PD5  // Chân 5
#define BUZZER_PIN  PD4  // Chân 4
#define WIPE_BUTTON PD3  // Chân 3
#define SS_PIN      PB2  // Chân 10 (SPI Slave Select)
#define RST_PIN     PB1  // Chân 9 (Reset cho MFRC522, không dùng trong code này)

// ??a ch? I2C c?a LCD
#define LCD_ADDRESS 0x27
#define MAGIC_NUMBER 143  // S? ma thu?t ?? ki?m tra th? master trong EEPROM
#define EEPROM_SIZE 1024  // Kích th??c EEPROM c?a ATmega328P (byte)

// Bi?n toàn c?c
uint8_t masterCard[4];
uint8_t readCard[4];
uint8_t storedCard[4];
bool match = false;
bool programMode = false;
bool replaceMaster = false;
uint8_t successRead;

// Khai báo nguyên m?u hàm
uint8_t getID(void);
void ShowReaderDetails(void);
void cycleLeds(void);
void normalModeOn(void);
void readID(uint8_t number);
void writeID(uint8_t a[]);
void deleteID(uint8_t a[]);
bool checkTwo(uint8_t a[], uint8_t b[]);
uint8_t findIDSLOT(uint8_t find[]);
bool findID(uint8_t find[]);
void BlinkLEDS(uint8_t led);
bool isMaster(uint8_t test[]);
bool monitorWipeButton(uint32_t interval);
void ShowOnLCD(void);
void LCD_clear(void);
void granted(void);
void denied(void);

// Hàm SPI cho MFRC522
void SPI_init(void) {
	DDRB |= (1 << DDB3) | (1 << DDB5) | (1 << DDB2); // MOSI, SCK, SS làm ??u ra
	PORTB |= (1 << PORTB2); // SS ? m?c cao ban ??u
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Kích ho?t SPI, Master, fck/16
}

uint8_t SPI_transfer(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}

void MFRC522_WriteRegister(uint8_t reg, uint8_t value) {
	PORTB &= ~(1 << PORTB2); // SS xu?ng th?p
	SPI_transfer((reg << 1) & 0x7E); // ??a ch?
	SPI_transfer(value); // Giá tr?
	PORTB |= (1 << PORTB2); // SS lên cao
}

uint8_t MFRC522_ReadRegister(uint8_t reg) {
	PORTB &= ~(1 << PORTB2); // SS xu?ng th?p
	SPI_transfer(((reg << 1) & 0x7E) | 0x80); // ??a ch? v?i bit ??c
	uint8_t value = SPI_transfer(0x00); // G?i byte gi? ?? ??c
	PORTB |= (1 << PORTB2); // SS lên cao
	return value;
}

void MFRC522_Init(void) {
	MFRC522_WriteRegister(0x01, 0x0F); // Reset
	MFRC522_WriteRegister(0x2A, 0x8D); // Cài ??t timer
	MFRC522_WriteRegister(0x2B, 0x3E); // Cài ??t timer
	MFRC522_WriteRegister(0x2D, 0x01); // Cài ??t timer
	MFRC522_WriteRegister(0x0C, 0x80); // B?t anten
}

uint8_t MFRC522_IsNewCardPresent(void) {
	MFRC522_WriteRegister(0x04, 0x00); // Xóa ng?t
	MFRC522_WriteRegister(0x09, 0x26); // L?nh yêu c?u
	MFRC522_WriteRegister(0x01, 0x0C); // Truy?n
	_delay_us(50);
	uint8_t status = MFRC522_ReadRegister(0x06); // Ki?m tra tr?ng thái
	return (status & 0x20) ? 1 : 0; // Tr? v? 1 n?u có th?
}

uint8_t MFRC522_ReadCardSerial(uint8_t *uid) {
	MFRC522_WriteRegister(0x09, 0xC2); // Ch?ng va ch?m
	MFRC522_WriteRegister(0x01, 0x0C); // Truy?n
	_delay_us(50);
	uint8_t status = MFRC522_ReadRegister(0x06);
	if (status & 0x20) {
		for (uint8_t i = 0; i < 4; i++) {
			uid[i] = MFRC522_ReadRegister(0x0A); // ??c UID 4 byte
		}
		return 1;
	}
	return 0;
}

void MFRC522_Halt(void) {
	MFRC522_WriteRegister(0x09, 0x00); // L?nh d?ng
	MFRC522_WriteRegister(0x01, 0x0C);
}

// Hàm TWI (I2C) cho LCD
void TWI_init(void) {
	TWBR = 72; // 100kHz t?i F_CPU 16MHz
	TWSR = 0;  // Prescaler = 1
	TWCR = (1 << TWEN); // Kích ho?t TWI
}

uint8_t TWI_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return (TWSR & 0xF8) == 0x08 ? 0 : 1;
}

uint8_t TWI_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return (TWSR & 0xF8) == 0x28 ? 0 : 1;
}

void TWI_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void LCD_send(uint8_t value, uint8_t rs) {
	uint8_t data[4];
	data[0] = (value & 0xF0) | (rs ? 0x05 : 0x04) | 0x08; // N?a cao, E=1
	data[1] = data[0] & ~0x08; // E=0
	data[2] = ((value << 4) & 0xF0) | (rs ? 0x05 : 0x04) | 0x08; // N?a th?p, E=1
	data[3] = data[2] & ~0x08; // E=0
	TWI_start();
	TWI_write(LCD_ADDRESS << 1);
	for (uint8_t i = 0; i < 4; i++) {
		TWI_write(data[i]);
	}
	TWI_stop();
}

void LCD_send_command(uint8_t cmd) {
	LCD_send(cmd, 0);
}

void LCD_send_data(uint8_t data) {
	LCD_send(data, 1);
}

void LCD_init(void) {
	_delay_ms(50);
	LCD_send_command(0x33); // Kh?i t?o
	_delay_ms(5);
	LCD_send_command(0x32); // Chuy?n sang ch? ?? 4-bit
	_delay_us(100);
	LCD_send_command(0x28); // 2 dòng, font 5x8
	LCD_send_command(0x0C); // B?t hi?n th?, t?t con tr?
	LCD_send_command(0x06); // Ch? ?? nh?p
	LCD_send_command(0x01); // Xóa màn hình
	_delay_ms(2);
}

void LCD_setCursor(uint8_t col, uint8_t row) {
	uint8_t addr = row ? 0x40 : 0x00;
	addr += col;
	LCD_send_command(0x80 | addr);
}

void LCD_print(const char *str) {
	while (*str) {
		LCD_send_data(*str++);
	}
}

// Hàm h? tr?
void granted(void) {
	PORTD &= ~(1 << BLUE_LED) & ~(1 << RED_LED);
	PORTD |= (1 << GREEN_LED);
	// Kích ho?t còi báo thành công: 2 ti?ng bíp ng?n
	PORTD |= (1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD &= ~(1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD |= (1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD &= ~(1 << BUZZER_PIN);
	_delay_ms(1000); // Gi? LED xanh sáng lâu h?n
}

void denied(void) {
	PORTD &= ~(1 << GREEN_LED) & ~(1 << BLUE_LED);
	PORTD |= (1 << RED_LED);
	PORTD |= (1 << BUZZER_PIN);
	_delay_ms(1000);
	PORTD &= ~(1 << BUZZER_PIN);
}

uint8_t getID(void) {
	if (!MFRC522_IsNewCardPresent()) return 0;
	if (!MFRC522_ReadCardSerial(readCard)) return 0;
	MFRC522_Halt();
	return 1;
}

void ShowReaderDetails(void) {
	uint8_t v = MFRC522_ReadRegister(0x37); // ??c phiên b?n
	if (v == 0x00 || v == 0xFF) {
		LCD_setCursor(0, 0);
		LCD_print("Comm Failure");
		LCD_setCursor(0, 1);
		LCD_print("Check Conn");
		PORTD |= (1 << BUZZER_PIN);
		_delay_ms(2000);
		PORTD &= ~(1 << BUZZER_PIN);
		PORTD |= (1 << RED_LED);
		PORTD &= ~(1 << GREEN_LED) & ~(1 << BLUE_LED);
		while (1);
	}
}

void cycleLeds(void) {
	PORTD = (PORTD & ~(1 << RED_LED)) | (1 << GREEN_LED) & ~(1 << BLUE_LED);
	_delay_ms(200);
	PORTD = (PORTD & ~(1 << RED_LED) & ~(1 << GREEN_LED)) | (1 << BLUE_LED);
	_delay_ms(200);
	PORTD = (PORTD | (1 << RED_LED)) & ~(1 << GREEN_LED) & ~(1 << BLUE_LED);
	_delay_ms(200);
}

void normalModeOn(void) {
	PORTD = (PORTD | (1 << BLUE_LED)) & ~(1 << RED_LED) & ~(1 << GREEN_LED);
}

void readID(uint8_t number) {
	uint8_t start = (number * 4) + 2;
	for (uint8_t i = 0; i < 4; i++) {
		storedCard[i] = eeprom_read_byte((uint8_t*)(start + i));
	}
}

void writeID(uint8_t a[]) {
	if (!findID(a)) {
		uint8_t num = eeprom_read_byte((uint8_t*)0);
		uint8_t start = (num * 4) + 6;
		num++;
		eeprom_write_byte((uint8_t*)0, num);
		for (uint8_t j = 0; j < 4; j++) {
			eeprom_write_byte((uint8_t*)(start + j), a[j]);
		}
		BlinkLEDS(GREEN_LED);
		LCD_setCursor(0, 1);
		LCD_print("Added");
		_delay_ms(1000);
		} else {
		BlinkLEDS(RED_LED);
		LCD_setCursor(0, 0);
		LCD_print("Failed!");
		LCD_setCursor(0, 1);
		LCD_print("Wrong ID/Bad EEPROM");
		_delay_ms(2000);
	}
}

void deleteID(uint8_t a[]) {
	if (!findID(a)) {
		BlinkLEDS(RED_LED);
		LCD_setCursor(0, 0);
		LCD_print("Failed!");
		LCD_setCursor(0, 1);
		LCD_print("Wrong ID/Bad EEPROM");
		_delay_ms(2000);
		} else {
		uint8_t num = eeprom_read_byte((uint8_t*)0);
		uint8_t slot = findIDSLOT(a);
		uint8_t start = (slot * 4) + 2;
		uint8_t looping = ((num - slot) * 4);
		num--;
		eeprom_write_byte((uint8_t*)0, num);
		for (uint8_t j = 0; j < looping; j++) {
			eeprom_write_byte((uint8_t*)(start + j), eeprom_read_byte((uint8_t*)(start + 4 + j)));
		}
		for (uint8_t k = 0; k < 4; k++) {
			eeprom_write_byte((uint8_t*)(start + looping + k), 0);
		}
		BlinkLEDS(BLUE_LED);
		LCD_setCursor(0, 1);
		LCD_print("Removed");
		_delay_ms(1000);
	}
}

bool checkTwo(uint8_t a[], uint8_t b[]) {
	match = true;
	for (uint8_t k = 0; k < 4; k++) {
		if (a[k] != b[k]) match = false;
	}
	return match;
}

uint8_t findIDSLOT(uint8_t find[]) {
	uint8_t count = eeprom_read_byte((uint8_t*)0);
	for (uint8_t i = 1; i <= count; i++) {
		readID(i);
		if (checkTwo(find, storedCard)) return i;
	}
	return 0;
}

bool findID(uint8_t find[]) {
	uint8_t count = eeprom_read_byte((uint8_t*)0);
	for (uint8_t i = 1; i <= count; i++) {
		readID(i);
		if (checkTwo(find, storedCard)) return true;
	}
	return false;
}

void BlinkLEDS(uint8_t led) {
	PORTD &= ~(1 << BLUE_LED) & ~(1 << RED_LED) & ~(1 << GREEN_LED);
	PORTD |= (1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD = (PORTD | (1 << led)) & ~(1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD &= ~(1 << led);
	PORTD |= (1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD = (PORTD | (1 << led)) & ~(1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD &= ~(1 << led);
	PORTD |= (1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD = (PORTD | (1 << led)) & ~(1 << BUZZER_PIN);
	_delay_ms(200);
	PORTD &= ~(1 << led);
}

bool isMaster(uint8_t test[]) {
	return checkTwo(test, masterCard);
}

bool monitorWipeButton(uint32_t interval) {
	for (uint16_t i = 0; i < interval / 1000; i++) {
		LCD_setCursor(10, 1);
		char buf[3];
		itoa(i, buf, 10);
		LCD_print(buf);
		if (PIND & (1 << WIPE_BUTTON)) return false;
		_delay_ms(1000);
	}
	return true;
}

void ShowOnLCD(void) {
	LCD_clear();
	LCD_setCursor(0, 0);
	LCD_print(" Access Control");
	LCD_setCursor(0, 1);
	LCD_print("   Scan a Tag");
}

void LCD_clear(void) {
	LCD_send_command(0x01);
	_delay_ms(2);
}

// Hàm chính
int main(void) {
	// C?u hình chân
	DDRD |= (1 << GREEN_LED) | (1 << BLUE_LED) | (1 << RED_LED) | (1 << BUZZER_PIN);
	DDRD &= ~(1 << WIPE_BUTTON);
	PORTD |= (1 << WIPE_BUTTON); // Kéo lên

	// Tr?ng thái ban ??u
	PORTD &= ~(1 << RED_LED) & ~(1 << GREEN_LED) & ~(1 << BLUE_LED);

	// Kh?i t?o ngo?i vi
	SPI_init();
	TWI_init();
	LCD_init();
	MFRC522_Init();

	ShowReaderDetails();

	// Xóa EEPROM n?u nh?n nút
	if (!(PIND & (1 << WIPE_BUTTON))) {
		PORTD |= (1 << RED_LED);
		LCD_setCursor(0, 0);
		LCD_print("Button Pressed");
		PORTD |= (1 << BUZZER_PIN);
		_delay_ms(1000);
		PORTD &= ~(1 << BUZZER_PIN);
		LCD_clear();
		LCD_setCursor(0, 0);
		LCD_print("This will remove");
		LCD_setCursor(0, 1);
		LCD_print("all records");
		_delay_ms(2000);
		LCD_clear();
		LCD_setCursor(0, 0);
		LCD_print("You have 10 ");
		LCD_setCursor(0, 1);
		LCD_print("secs to Cancel");
		_delay_ms(2000);
		LCD_clear();
		LCD_setCursor(0, 0);
		LCD_print("Unpress to cancel");
		LCD_setCursor(0, 1);
		LCD_print("Counting: ");
		if (monitorWipeButton(10000)) {
			LCD_print("Wiping EEPROM...");
			for (uint16_t x = 0; x < EEPROM_SIZE; x++) {
				if (eeprom_read_byte((uint8_t*)x) != 0) {
					eeprom_write_byte((uint8_t*)x, 0);
				}
			}
			LCD_clear();
			LCD_setCursor(0, 0);
			LCD_print("Wiping Done");
			PORTD &= ~(1 << RED_LED);
			for (uint8_t i = 0; i < 3; i++) {
				PORTD |= (1 << BUZZER_PIN);
				_delay_ms(200);
				PORTD &= ~(1 << BUZZER_PIN);
				PORTD ^= (1 << RED_LED);
				_delay_ms(200);
			}
			} else {
			LCD_clear();
			LCD_setCursor(0, 0);
			LCD_print("Wiping Cancelled");
			PORTD &= ~(1 << RED_LED);
		}
	}

	// Ki?m tra ho?c ??nh ngh?a th? master
	if (eeprom_read_byte((uint8_t*)1) != MAGIC_NUMBER) {
		LCD_clear();
		LCD_setCursor(0, 0);
		LCD_print("No Master Card ");
		LCD_setCursor(0, 1);
		LCD_print("Defined");
		_delay_ms(2000);
		LCD_setCursor(0, 0);
		LCD_print("Scan A Tag to ");
		LCD_setCursor(0, 1);
		LCD_print("Define as Master");
		do {
			successRead = getID();
			PORTD ^= (1 << BLUE_LED);
			PORTD ^= (1 << BUZZER_PIN);
			_delay_ms(200);
			PORTD ^= (1 << BUZZER_PIN);
		} while (!successRead);
		for (uint8_t j = 0; j < 4; j++) {
			eeprom_write_byte((uint8_t*)(2 + j), readCard[j]);
		}
		eeprom_write_byte((uint8_t*)1, MAGIC_NUMBER);
		LCD_clear();
		LCD_setCursor(0, 0);
		LCD_print("Master Defined");
		_delay_ms(2000);
	}

	for (uint8_t i = 0; i < 4; i++) {
		masterCard[i] = eeprom_read_byte((uint8_t*)(2 + i));
	}

	ShowOnLCD();
	cycleLeds();

	// Vòng l?p chính
	while (1) {
		do {
			successRead = getID();
			if (programMode) {
				cycleLeds();
				} else {
				normalModeOn();
			}
		} while (!successRead);

		if (programMode) {
			if (isMaster(readCard)) {
				LCD_clear();
				LCD_setCursor(0, 0);
				LCD_print("Exiting Program");
				PORTD |= (1 << BUZZER_PIN);
				_delay_ms(1000);
				PORTD &= ~(1 << BUZZER_PIN);
				ShowOnLCD();
				programMode = false;
				} else {
				if (findID(readCard)) {
					LCD_clear();
					LCD_setCursor(0, 0);
					LCD_print("Already there");
					deleteID(readCard);
					LCD_clear();
					LCD_setCursor(0, 0);
					LCD_print("Tag to ADD/REM");
					LCD_setCursor(0, 1);
					LCD_print("Master to Exit");
					} else {
					LCD_clear();
					LCD_setCursor(0, 0);
					LCD_print("New Tag,adding...");
					writeID(readCard);
					LCD_clear();
					LCD_setCursor(0, 0);
					LCD_print("Scan to ADD/REM");
					LCD_setCursor(0, 1);
					LCD_print("Master to Exit");
				}
			}
			} else {
			if (isMaster(readCard)) {
				programMode = true;
				LCD_clear();
				LCD_setCursor(0, 0);
				LCD_print("Program Mode");
				uint8_t count = eeprom_read_byte((uint8_t*)0);
				LCD_setCursor(0, 1);
				LCD_print("I have ");
				char buf[3];
				itoa(count, buf, 10);
				LCD_print(buf);
				LCD_print(" records");
				PORTD |= (1 << BUZZER_PIN);
				_delay_ms(2000);
				PORTD &= ~(1 << BUZZER_PIN);
				LCD_clear();
				LCD_setCursor(0, 0);
				LCD_print("Scan a Tag to ");
				LCD_setCursor(0, 1);
				LCD_print("ADD/REMOVE");
				} else {
				if (findID(readCard)) {
					LCD_clear();
					LCD_setCursor(0, 0);
					LCD_print("Access Granted");
					granted();
					ShowOnLCD();
					} else {
					LCD_clear();
					LCD_setCursor(0, 0);
					LCD_print("Access Denied");
					denied();
					ShowOnLCD();
				}
			}
		}
	}

	return 0;
}