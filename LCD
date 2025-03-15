

          

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

// LCD interface (should agree with the diagram above)
//   make sure that the LCD RW pin is connected to GND
#define lcd_D7_port     PORTD                   // lcd D7 connection
#define lcd_D7_bit      PORTD7
#define lcd_D7_ddr      DDRD

#define lcd_D6_port     PORTD                   // lcd D6 connection
#define lcd_D6_bit      PORTD6
#define lcd_D6_ddr      DDRD

#define lcd_D5_port     PORTD                   // lcd D5 connection
#define lcd_D5_bit      PORTD5
#define lcd_D5_ddr      DDRD

#define lcd_D4_port     PORTD                   // lcd D4 connection
#define lcd_D4_bit      PORTD4
#define lcd_D4_ddr      DDRD

#define lcd_D3_port     PORTD                   // lcd D3 connection
#define lcd_D3_bit      PORTD3
#define lcd_D3_ddr      DDRD

#define lcd_D2_port     PORTD                   // lcd D2 connection
#define lcd_D2_bit      PORTD2
#define lcd_D2_ddr      DDRD

#define lcd_D1_port     PORTD                   // lcd D1 connection
#define lcd_D1_bit      PORTD1
#define lcd_D1_ddr      DDRD

#define lcd_D0_port     PORTD                   // lcd D0 connection
#define lcd_D0_bit      PORTD0
#define lcd_D0_ddr      DDRD

#define lcd_E_port      PORTB                   // lcd Enable pin
#define lcd_E_bit       PORTB1
#define lcd_E_ddr       DDRB

#define lcd_RS_port     PORTB                   // lcd Register Select pin
#define lcd_RS_bit      PORTB0
#define lcd_RS_ddr      DDRB

// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2
#define   lcd_LineThree   0x14                  // start of line 3 (20x4)
#define   lcd_lineFour    0x54                  // start of line 4 (20x4)
//#define   lcd_LineThree   0x10                  // start of line 3 (16x4)
//#define   lcd_lineFour    0x50                  // start of line 4 (16x4)

// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet8bit 0b00111000          // 8-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position

// Program ID
uint8_t program_date[]   = "15 march 2025";
uint8_t program_timne[]	=	"10pm";
uint8_t program_relay[]  = " ON ";
uint8_t program_id[]     = "25042004";

// Function Prototypes
void lcd_write_8(uint8_t);
void lcd_write_instruction_8d(uint8_t);
void lcd_write_character_8d(uint8_t);
void lcd_write_string_8d(uint8_t *);
void lcd_init_8d(void);

/******************************* Main Program Code *************************/
int main(void)
{
// configure the microprocessor pins for the data lines
    lcd_D7_ddr |= (1<<lcd_D7_bit);                  // 8 data lines - output
    lcd_D6_ddr |= (1<<lcd_D6_bit);
    lcd_D5_ddr |= (1<<lcd_D5_bit);
    lcd_D4_ddr |= (1<<lcd_D4_bit);
    lcd_D3_ddr |= (1<<lcd_D3_bit);
    lcd_D2_ddr |= (1<<lcd_D2_bit);
    lcd_D1_ddr |= (1<<lcd_D1_bit);
    lcd_D0_ddr |= (1<<lcd_D0_bit);

// configure the microprocessor pins for the control lines
    lcd_E_ddr |= (1<<lcd_E_bit);                    // E line - output
    lcd_RS_ddr |= (1<<lcd_RS_bit);                  // RS line - output

// initialize the LCD controller as determined by the defines (LCD instructions)
    lcd_init_8d();                                  // initialize the LCD display for an 8-bit interface

// display the first line of information
    lcd_write_string_8d(program_date);

// set cursor to start of second line
    lcd_write_instruction_8d(lcd_SetCursor | lcd_LineTwo);
    _delay_us(80);                                  // 40 uS delay (min)

// display the second line of information
    lcd_write_string_8d(program_timne);
	
	// set cursor to start of third line
	lcd_write_instruction_8d(lcd_SetCursor | lcd_LineThree);
	_delay_us(80);
	
	// display the third line of information
	lcd_write_string_8d(program_relay);
	
	lcd_write_instruction_8d(lcd_SetCursor | lcd_lineFour);
	_delay_us(80);
		
		// display the third line of information
		lcd_write_string_8d(program_id);
// endless loop
    while(1);
    return 0;


void lcd_init_8d(void)
{
// Power-up delay
    _delay_ms(100);                                 // initial 40 mSec delay

// Reset the LCD controller
    lcd_write_instruction_8d(lcd_FunctionReset);    // first part of reset sequence
    _delay_ms(10);                                  // 4.1 mS delay (min)

    lcd_write_instruction_8d(lcd_FunctionReset);    // second part of reset sequence
    _delay_us(200);                                 // 100uS delay (min)

    lcd_write_instruction_8d(lcd_FunctionReset);    // third part of reset sequence
    _delay_us(200);                                 // this delay is omitted in the data sheet

// Function Set instruction
    lcd_write_instruction_8d(lcd_FunctionSet8bit);  // set mode, lines, and font
    _delay_us(80);                                  // 40uS delay (min)


// Display On/Off Control instruction
    lcd_write_instruction_8d(lcd_DisplayOff);       // turn display OFF
    _delay_us(80);                                  // 40 uS delay (min)

// Clear Display instruction
    lcd_write_instruction_8d(lcd_Clear);            // clear display RAM
    _delay_ms(4);                                   // 1.64 mS delay (min)

// ; Entry Mode Set instruction
    lcd_write_instruction_8d(lcd_EntryMode);        // set desired shift characteristics
    _delay_us(80);                                  // 40 uS delay (min)



// Display On/Off Control instruction
    lcd_write_instruction_8d(lcd_DisplayOn);        // turn the display ON
    _delay_us(80);                                  // 40 uS delay (min)
}

/*...........................................................................
 
void lcd_write_string_8d(uint8_t theString[])
{
    volatile int i = 0;                             // character counter*/
    while (theString[i] != 0)
    {
        lcd_write_character_8d(theString[i]);
        i++;
        _delay_us(80);                              // 40 uS delay (min)
    }
}

)

void lcd_write_character_8d(uint8_t theData)
{
    lcd_RS_port |= (1<<lcd_RS_bit);                 // select the Data Register (RS high)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write_8(theData);                           // write the data
}


void lcd_write_instruction_8d(uint8_t theInstruction)
{
    lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write_8(theInstruction);                    // write the instruction
}


void lcd_write_8(uint8_t theByte)
{
    lcd_D7_port &= ~(1<<lcd_D7_bit);                        // assume that data is '0'
    if (theByte & 1<<7) lcd_D7_port |= (1<<lcd_D7_bit);     // make data = '1' if necessary

    lcd_D6_port &= ~(1<<lcd_D6_bit);                        // repeat for each data bit
    if (theByte & 1<<6) lcd_D6_port |= (1<<lcd_D6_bit);

    lcd_D5_port &= ~(1<<lcd_D5_bit);
    if (theByte & 1<<5) lcd_D5_port |= (1<<lcd_D5_bit);

    lcd_D4_port &= ~(1<<lcd_D4_bit);
    if (theByte & 1<<4) lcd_D4_port |= (1<<lcd_D4_bit);

    lcd_D3_port &= ~(1<<lcd_D3_bit);
    if (theByte & 1<<3) lcd_D3_port |= (1<<lcd_D3_bit);

    lcd_D2_port &= ~(1<<lcd_D2_bit);
    if (theByte & 1<<2) lcd_D2_port |= (1<<lcd_D2_bit);

    lcd_D1_port &= ~(1<<lcd_D1_bit);
    if (theByte & 1<<1) lcd_D1_port |= (1<<lcd_D1_bit);

    lcd_D0_port &= ~(1<<lcd_D0_bit);
    if (theByte & 1<<0) lcd_D0_port |= (1<<lcd_D0_bit);

// write the data
                                                    // 'Address set-up time' (40 nS)
    lcd_E_port |= (1<<lcd_E_bit);                   // Enable pin high
    _delay_us(1);                                   // implement 'Data set-up time' (80 nS) and 'Enable pulse width' (230 nS)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // Enable pin low
    _delay_us(1);                                   // implement 'Data hold time' (10 nS) and 'Enable cycle time' (500 nS)
}


 
     
