/*
 * ATmega164
 *
 * Created: 11/25/2020 10:43:10 AM
 * Author: Savu Alexandru-Mihai
 *
 *
 * Freq = 20 MHz
 * Timer = 20M / 1024 (max pre-scalar for timer0) = 19.53125 KHz (CS00 = 1, CS01 = 0, CS02 = 1)
 * Tick = 1 / 19.53125K = 51.2 mu seconds
 * Total = 51.2 * 255 (timer0 max count) = 13 ms (1 second = 76.9 x 13 or 77 x 13 aprox 1 sec) (for 1 ms set OCR0 to 20 because 51.2 * 20 = 1.024 ms)
 * Timer0 = 13 ms (also Timer2)
 * Timer1 = 3.356 seconds
 *
 */

#define F_CPU 1000000UL
#define BAUDRATE 9600
#define UBRRVAL ((F_CPU/(BAUDRATE * 16UL))-1)

#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#define P_SCK  PINB7
#define P_COPI PINB5
#define P_CD   PINB6
#define P_RST  PINB4
#define SCREEN_ROW 6        // 48 pixels, 8 pixels each
#define SCREEN_COLUMN 84    // 84 pixels
#define P_LED  PIND6
#define P_SW1  PIND5
#define SW_VERSION		13
#define P_UP PIND0
#define P_RGT PIND1
#define P_LFT PIND2
#define P_DWN PIND3

extern const PROGMEM unsigned char FONT[] = {
    0x00, 0x00, 0x00,   // 0x20  
    0x00, 0x2e, 0x00,   // 0x21 !
    0x06, 0x00, 0x06,   // 0x22 "
    0x3e, 0x14, 0x3e,   // 0x23 #
    0x2c, 0x7e, 0x34,   // 0x24 $
    0x12, 0x08, 0x24,   // 0x25 %
    0x14, 0x2a, 0x34,   // 0x26 &
    0x00, 0x06, 0x00,   // 0x27 '
    0x1c, 0x22, 0x00,   // 0x28 (
    0x00, 0x22, 0x1c,   // 0x29 )
    0x0a, 0x04, 0x0a,   // 0x2a *
    0x08, 0x1c, 0x08,   // 0x2b +
    0x40, 0x30, 0x00,   // 0x2c ,
    0x08, 0x08, 0x08,   // 0x2d -
    0x00, 0x60, 0x60,   // 0x2e .
    0x10, 0x08, 0x04,   // 0x2f /
    0x1c, 0x2a, 0x1c,   // 0x30 0
    0x04, 0x3e, 0x00,   // 0x31 1
    0x32, 0x2a, 0x24,   // 0x32 2
    0x2a, 0x2a, 0x14,   // 0x33 3
    0x18, 0x14, 0x3a,   // 0x34 4
    0x2e, 0x2a, 0x3a,   // 0x35 5
    0x3e, 0x2a, 0x3a,   // 0x36 6
    0x02, 0x3a, 0x06,   // 0x37 7
    0x3e, 0x2a, 0x3e,   // 0x38 8
    0x2e, 0x2a, 0x3e,   // 0x39 9
    0x00, 0x14, 0x00,   // 0x3a :
    0x20, 0x14, 0x00,   // 0x3b ;
    0x08, 0x1c, 0x14,   // 0x3c <
    0x14, 0x14, 0x14,   // 0x3d =
    0x14, 0x1c, 0x08,   // 0x3e >
    0x02, 0x2a, 0x04,   // 0x3f ?
    0x00, 0x00, 0x00,   // 0x40 @
    0x3c, 0x0a, 0x3c,   // 0x41 A
    0x3e, 0x2a, 0x14,   // 0x42 B
    0x1c, 0x22, 0x22,   // 0x43 C
    0x3e, 0x22, 0x1c,   // 0x44 D
    0x3e, 0x2a, 0x2a,   // 0x45 E
    0x3e, 0x0a, 0x0a,   // 0x46 F
    0x1c, 0x22, 0x3a,   // 0x47 G
    0x3e, 0x08, 0x3e,   // 0x48 H
    0x22, 0x3e, 0x22,   // 0x49 I
    0x10, 0x20, 0x1e,   // 0x4a J
    0x3e, 0x08, 0x36,   // 0x4b K
    0x3e, 0x20, 0x20,   // 0x4c L
    0x3e, 0x0c, 0x3e,   // 0x4d M
    0x3e, 0x1c, 0x3e,   // 0x4e N
    0x1c, 0x22, 0x1c,   // 0x4f O
    0x3e, 0x0a, 0x04,   // 0x50 P
    0x1c, 0x32, 0x3c,   // 0x51 Q
    0x3e, 0x0a, 0x34,   // 0x52 R
    0x24, 0x2a, 0x12,   // 0x53 S
    0x02, 0x3e, 0x02,   // 0x54 T
    0x3e, 0x20, 0x3e,   // 0x55 U
    0x0e, 0x30, 0x0e,   // 0x56 V
    0x3e, 0x18, 0x3e,   // 0x57 W
    0x36, 0x08, 0x36,   // 0x58 X
    0x06, 0x38, 0x06,   // 0x59 Y
    0x32, 0x2a, 0x26,   // 0x5a Z
    0x3e, 0x22, 0x00,   // 0x5b [
    0x04, 0x08, 0x10,   // 0x5c \/
    0x00, 0x22, 0x3e,   // 0x5d ]
    0x06, 0x03, 0x06,   // 0x5e ^
    0x20, 0x20, 0x20,   // 0x5f _
    0x02, 0x04, 0x00,   // 0x60 `
    0x10, 0x28, 0x38,   // 0x61 a
    0x3e, 0x28, 0x10,   // 0x62 b
    0x30, 0x28, 0x28,   // 0x63 c
    0x10, 0x28, 0x3e,   // 0x64 d
    0x18, 0x2c, 0x2c,   // 0x65 e
    0x08, 0x3c, 0x0a,   // 0x66 f
    0x90, 0xa8, 0x78,   // 0x67 g
    0x3e, 0x08, 0x30,   // 0x68 h
    0x00, 0x3a, 0x00,   // 0x69 i
    0x40, 0x3a, 0x00,   // 0x6a j
    0x3e, 0x10, 0x28,   // 0x6b k
    0x00, 0x3e, 0x00,   // 0x6c l
    0x38, 0x18, 0x38,   // 0x6d m
    0x38, 0x08, 0x30,   // 0x6e n
    0x38, 0x28, 0x38,   // 0x6f o
    0xf8, 0x28, 0x10,   // 0x70 p
    0x10, 0x28, 0xf8,   // 0x71 q
    0x38, 0x08, 0x08,   // 0x72 r
    0x20, 0x38, 0x08,   // 0x73 s
    0x04, 0x3e, 0x24,   // 0x74 t
    0x38, 0x20, 0x38,   // 0x75 u
    0x18, 0x30, 0x18,   // 0x76 v
    0x38, 0x30, 0x38,   // 0x77 w
    0x28, 0x10, 0x28,   // 0x78 x
    0x98, 0x60, 0x18,   // 0x79 y
    0x08, 0x38, 0x20,   // 0x7a z
    0x08, 0x1c, 0x22,   // 0x7b {
    0x00, 0x7e, 0x00,   // 0x7c |
    0x22, 0x1c, 0x08,   // 0x7d }
    0x08, 0x18, 0x10,   // 0x7e ~
};

const char TXT_ABOUT_line_0[] PROGMEM = "A game where you must";
const char TXT_ABOUT_line_1[] PROGMEM = "guess the word. This";
const char TXT_ABOUT_line_2[] PROGMEM = "software was written";
const char TXT_ABOUT_line_3[] PROGMEM = "by Alexandru-Mihai";
const char TXT_ABOUT_line_4[] PROGMEM = "Savu group 1242 EB";
const char TXT_ABOUT_line_5[] PROGMEM = "from FILS.";

PGM_P const TXT_ABOUT[] PROGMEM =
{
	TXT_ABOUT_line_0,
	TXT_ABOUT_line_1,
	TXT_ABOUT_line_2,
	TXT_ABOUT_line_3,
	TXT_ABOUT_line_4,
	TXT_ABOUT_line_5,
};

const char TXT_HOWTOPLAY_line_0[] PROGMEM = "Controls: Use the up";
const char TXT_HOWTOPLAY_line_1[] PROGMEM = "and down keys to";
const char TXT_HOWTOPLAY_line_2[] PROGMEM = "select a letter and";
const char TXT_HOWTOPLAY_line_3[] PROGMEM = "confirm the selection";
const char TXT_HOWTOPLAY_line_4[] PROGMEM = "with the right key.";
const char TXT_HOWTOPLAY_line_5[] PROGMEM = "Rules: You must guess";
const char TXT_HOWTOPLAY_line_6[] PROGMEM = "the word by selecting";
const char TXT_HOWTOPLAY_line_7[] PROGMEM = "the letters which";
const char TXT_HOWTOPLAY_line_8[] PROGMEM = "occur in the word.";
const char TXT_HOWTOPLAY_line_9[] PROGMEM = "Each selection";
const char TXT_HOWTOPLAY_line_10[] PROGMEM = "reveals part of the";
const char TXT_HOWTOPLAY_line_11[] PROGMEM = "word and when the";
const char TXT_HOWTOPLAY_line_12[] PROGMEM = "whole word is";
const char TXT_HOWTOPLAY_line_13[] PROGMEM = "revealed, you win. If";
const char TXT_HOWTOPLAY_line_14[] PROGMEM = "you guess incorrectly";
const char TXT_HOWTOPLAY_line_15[] PROGMEM = "then you will receive";
const char TXT_HOWTOPLAY_line_16[] PROGMEM = "a strike. The 3rd";
const char TXT_HOWTOPLAY_line_17[] PROGMEM = "strike results in a";
const char TXT_HOWTOPLAY_line_18[] PROGMEM = "loss.";

PGM_P const TXT_HOWTOPLAY[] PROGMEM =
{
	TXT_HOWTOPLAY_line_0,
	TXT_HOWTOPLAY_line_1,
	TXT_HOWTOPLAY_line_2,
	TXT_HOWTOPLAY_line_3,
	TXT_HOWTOPLAY_line_4,
	TXT_HOWTOPLAY_line_5,
	TXT_HOWTOPLAY_line_6,
	TXT_HOWTOPLAY_line_7,
	TXT_HOWTOPLAY_line_8,
	TXT_HOWTOPLAY_line_9,
	TXT_HOWTOPLAY_line_10,
	TXT_HOWTOPLAY_line_11,
	TXT_HOWTOPLAY_line_12,
	TXT_HOWTOPLAY_line_13,
	TXT_HOWTOPLAY_line_14,
	TXT_HOWTOPLAY_line_15,
	TXT_HOWTOPLAY_line_16,
	TXT_HOWTOPLAY_line_17,
	TXT_HOWTOPLAY_line_18,
};

const char TXT_WORDTOGUESS_word_0[] PROGMEM = "MICROCONTROLLER";
const char TXT_WORDTOGUESS_word_1[] PROGMEM = "SCREEN";
const char TXT_WORDTOGUESS_word_2[] PROGMEM = "CHIP";
const char TXT_WORDTOGUESS_word_3[] PROGMEM = "COMPUTER";
const char TXT_WORDTOGUESS_word_4[] PROGMEM = "GRAPHICS";
const char TXT_WORDTOGUESS_word_5[] PROGMEM = "GROUND";

PGM_P const TXT_WORDTOGUESS[] PROGMEM =
{
	TXT_WORDTOGUESS_word_0,
	TXT_WORDTOGUESS_word_1,
	TXT_WORDTOGUESS_word_2,
	TXT_WORDTOGUESS_word_3,
	TXT_WORDTOGUESS_word_4,
	TXT_WORDTOGUESS_word_5,
};

// Make a pin HIGH (MUST GIVE THE PORT BY HAND)
// Ex: High(PINB2, 'B')
void High(unsigned char pin, unsigned char port)
{
    if (port == 'A' || port == 'a')
    {
        PORTA |= (1 << pin);
    }
    else if (port == 'B' || port == 'b')
    {
        PORTB |= (1 << pin);
    }
    else if (port == 'C' || port == 'c')
    {
        PORTC |= (1 << pin);
    }
    else if (port == 'D' || port == 'd')
    {
        PORTD |= (1 << pin);
    }
}

// Make a pin LOW (MUST GIVE THE PORT BY HAND)
// Ex: Low(PINB2, 'B')
void Low(unsigned char pin, unsigned char port)
{
    if (port == 'A' || port == 'a')
    {
        PORTA &= ~(1 << pin);
    }
    else if (port == 'B' || port == 'b')
    {
        PORTB &= ~(1 << pin);
    }
    else if (port == 'C' || port == 'c')
    {
        PORTC &= ~(1 << pin);
    }
    else if (port == 'D' || port == 'd')
    {
        PORTD &= ~(1 << pin);
    }
}

// Toggle a pin HIGH or LOW (MUST GIVE THE PORT BY HAND)
// Ex: Toggle(PINB2, 'B')
void Toggle(unsigned char pin, unsigned char port)
{
    if (port == 'A' || port == 'a')
    {
        PORTA ^= (1 << pin);
    }
    else if (port == 'B' || port == 'b')
    {
        PORTB ^= (1 << pin);
    }
    else if (port == 'C' || port == 'c')
    {
        PORTC ^= (1 << pin);
    }
    else if (port == 'D' || port == 'd')
    {
        PORTD ^= (1 << pin);
    }
}

bool keyDown = false;

// Read a pin (MUST GIVE THE PORT BY HAND)
// Ex: Read(PINB2, 'B')
bool Read(unsigned char pin, unsigned char port)
{
	// Get input is being pressed (returns true while button held)
    if (port == 'A' || port == 'a')
    {
		keyDown = !((PINA & (1 << pin)) >> pin);
    }
    else if (port == 'B' || port == 'b')
    {
		keyDown = !((PINB & (1 << pin)) >> pin);
    }
    else if (port == 'C' || port == 'c')
    {
		keyDown = !((PINC & (1 << pin)) >> pin);
    }
    else if (port == 'D' || port == 'd')
    {
		keyDown = !((PIND & (1 << pin)) >> pin);
    }
    else
    {
		keyDown = false;
    }
	// Return the button is being held.
	return keyDown;
}

// Software SPI method
void spiWrite(unsigned short data)
{
    unsigned char i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)    // Compare data to 1000 0000 (MSB)
        {
            PORTB |= (1 << P_COPI);   // Set high
        }
        else
        {
            PORTB &= ~(1 << P_COPI);  // Set low
        }
        PORTB |= (1 << P_SCK);       // Clock high for write
        PORTB &= ~(1 << P_SCK);      // Clock low for next write
        data <<= 1;     // Shift data to next bit to write
    }
}

// Send data to display with Software SPI
void DisplayData(unsigned char data)
{
    PORTB |= (1 << P_CD);   // CD HIGH for data
    spiWrite(data);
}

// Send data to display with Software SPI
void DisplayCMD(unsigned char cmd)
{
    PORTB &= ~(1 << P_CD);  // CD LOW for command
    spiWrite(cmd);
}

// Clear the whole display by setting all pixels to 0
void DisplayClear()
{
    unsigned short i;
    DisplayCMD(0x80);   // Pos set
    DisplayCMD(0x40);   // Pos set

    for (i = 0; i < (SCREEN_COLUMN * SCREEN_ROW); i++)
    {
        DisplayData(0x00);
    }
}

// Initializing the display (needs a reset before sending commands by making reset pin low then high
void DisplayInit()
{
    PORTB &= ~((1 << P_SCK) | (1 << P_COPI) | (1 << P_RST) | (1 << P_CD));       // Set all pins low before setting as output
    DDRB |= ((1 << P_SCK) | (1 << P_COPI) | (1 << P_RST) | (1 << P_CD));          // Set all pins as output
    _delay_ms(20);                   // Reset screen wait to go from low to high
    PORTB |= (1 << P_RST);          // Set Reset High to init screen
    DisplayCMD(0x21);               // Extended commands
    DisplayCMD(0xA1);               // Set vop contrast
    DisplayCMD(0x04);               // Set temp coefficient
    DisplayCMD(0x14);               // Bias mode
    DisplayCMD(0x20);               // Normal commands
    DisplayCMD(0x0C);               // Normal display with horizontal address
}

// Used for indexing columns in character sets (it skips 4 pixels to the right * the column you want to start at)
bool colIndex = true;
// Print a character on screen at column (character indexed not pixel indexed) and row
void DisplayPrintChar(unsigned char row, unsigned char col, char letter)
{
    if (colIndex)
    {
        col += col * 4;
    }
    if (letter == 0x40 || !((letter >= 0x20) && (letter <= 0x7e)))
    {
        letter = '*';
    }
    if ((col <= SCREEN_COLUMN) && (row <= SCREEN_ROW))
    {
        unsigned char pixels;
        const PROGMEM unsigned char* letterData = FONT + ((letter - 0x20) * 3);     // Missing ASCII will give * as result 

        DisplayCMD(0x80 | col);
        DisplayCMD(0x40 | row % SCREEN_ROW);

        for (pixels = 0; (pixels < 3) && (col < SCREEN_COLUMN); pixels++, col++, letterData++)
        {
            unsigned char data = pgm_read_byte_near(letterData);    // Read byte from flash memory
            DisplayData(data);
        }
        if (col < SCREEN_COLUMN)
        {
            DisplayData(0x00);
        }
    }
}

// Print a line using the print character method (No wrapping just clipping)
void DisplayPrintLine(unsigned char row, unsigned char col, char* string)
{
	colIndex = false;
	col += col * 4;
	for (; (*string != '\0') && (col < SCREEN_COLUMN); col += 4, string++)
    {
        DisplayPrintChar(row, col, *string);
    }
    colIndex = true;
}

// Timer0 stops main loop in seconds
void TimeoutS(unsigned char seconds)
{
    unsigned int TOVCount;                  // Counter for overflow 1 second = aprox. 77 overflows (SEE AT TOP DEFINITION)
    TCNT0 = 0x00;                           // From mega164.h
    TCCR0B = (1 << CS00) | (1 << CS02);     // Set CS00 and CS02 bits clock source to 1024
    for (TOVCount = 0; TOVCount < seconds * 77; TOVCount++)
    {
        while ((TIFR0 & 0x01) == 0);
        TCNT0 = 0x00;                       // Reset values for flag and timer
        TIFR0 = 0x01;
    }
}

// Timer0 stops main loop in milliseconds
void TimeoutMS(unsigned int ms)
{
    unsigned int TOVCount;
    TCNT0 = 0x00;                                   // Set timer to 0
    TCCR0B = (1 << CS00) | (1 << CS02);             // Set pre-scalar as 1024
    OCR0A = 0x14;                                   // Count to 20
    for (TOVCount = 0; TOVCount < ms; TOVCount++)
    {
        while ((TIFR0 & (1 << OCF0A)) == 0);
        TCNT0 = 0x00;                               // Reset values for flag and timer
        TIFR0 |= (1 << OCF0A);
    }
}

// Timer0 variables for interrupt
unsigned int time = 0;
unsigned int counter = 0;
bool timerMSEnabled = false;
bool timerSEnabled = false;

// Set timer0 for interrupt in milliseconds
void TimeoutMSInterrupt(unsigned int ms)
{
    time = ms;
    timerMSEnabled = true;
    counter = 0;
    TCNT0 = 0x00;
    OCR0A = 0x14;           // Count to 20 because this is aprox 1 ms (CHECK AT TOP)
}

// Set timer0 for interrupt in seconds
void TimeoutSInterrupt(unsigned char seconds)
{
    time = seconds * 77;    // Calculated that 77 counts of overflow is aprox 1 second (CHECK AT TOP)
    timerSEnabled = true;
    counter = 0;
    TCNT0 = 0x00;
}

/*
// Timer0 Overflow interrupt
interrupt[TIM0_OVF] void timer0_ovf_isr(void)
{
    if (timerSEnabled)
    {
        counter++;
        if (counter == time)
        {
            counter = 0;
            timerSEnabled = false;
        }
    }
}

// Timer0 Comp interrupt
interrupt[TIM0_COMPA] void timer0_compa_isr(void)
{
    if (timerMSEnabled)
    {
        TCNT0 = 0x00;   // Don't wait for overflow reset the timer to 0
        counter++;
        if (counter == time)
        {
            counter = 0;
            timerMSEnabled = false;
        }
    }

}
*/

void initInput()
{
	DDRD &= ~(1 << P_UP);
	DDRD &= ~(1 << P_RGT);
	DDRD &= ~(1 << P_LFT);
	DDRD &= ~(1 << P_DWN);
	High(P_UP, 'D');
	High(P_RGT, 'D');
	High(P_LFT, 'D');
	High(P_DWN, 'D');
}

void HandleInput(unsigned char *indexPg, bool *update, unsigned char *selection, unsigned char *scrollIndex, unsigned char *scrollLength, bool *confirmSelect)
{
	if(Read(P_RGT, 'D'))
	{
		_delay_ms(30);
		if(Read(P_RGT, 'D'))
		{
			if(*indexPg < 1)
			{
				(*indexPg)++;
				*update = true;
			}
			else if((*indexPg == 1) && (*selection == 0))
			{
				*confirmSelect = true;
				*update = true;
			}	
		}
	}
	else if(Read(P_DWN, 'D'))
	{
		_delay_ms(30);
		if(Read(P_DWN, 'D'))
		{
			if((*indexPg == 0) && (*selection < 2))
			{
				(*selection)++;
				*update = true;
			}
			else if((*indexPg == 1) && (*selection == 0) && (*scrollIndex < *scrollLength))
			{
				(*scrollIndex)++;
				*update = true;
			}
			else if((*indexPg == 1) && (*selection > 0) && ((*scrollIndex) + 5) < ((*scrollLength)))
			{
				(*scrollIndex)++;
				*update = true;
			}
		}
	}
	else if(Read(P_UP, 'D'))
	{
		_delay_ms(30);
		if(Read(P_UP, 'D'))
		{
			if((*indexPg == 0) && (*selection > 0))
			{
				(*selection)--;
				*update = true;
			}
			else if((*indexPg == 1) && (*selection == 0) && (*scrollIndex > 0))
			{
				(*scrollIndex)--;
				*update = true;
			}
			else if((*indexPg == 1) && (*selection > 0) && ((*scrollIndex) > 0))
			{
				(*scrollIndex)--;
				*update = true;
			}	
		}
	}
	else if(Read(P_LFT, 'D'))
	{
		_delay_ms(30);
		if(Read(P_LFT, 'D'))
		{
			if(*indexPg == 1)
			{
				(*scrollIndex) = 0;
				(*indexPg)--;
				*update = true;
			}
			if((*indexPg == 2) || (*indexPg == 3))
			{
				(*scrollIndex) = 0;
				(*indexPg) = 0;
				*update = true;
			}	
		}
	}
}

void main(void)
{
	unsigned char scrollIndex = 0;
	unsigned char indexPg = 0;
	unsigned char selection = 0;
    unsigned char i;
	unsigned char j;
	unsigned char scrollLength = 0;
	unsigned char strikes = 0;
	char strWord[17];
	char strStrikes[17] = "Strikes: ______"; // 9, 10, 11, 12, 13, 14 are the X's
	char alphabet[26] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	char chList[26];
	char chListGuessed[26];
	bool update = true;
	bool confirmSelect = false;
	bool generate = true;
    TCNT0 = 0x00;                                   // Set timer to 0
    TCCR0B = (1 << CS00) | (1 << CS02);             // Set pre-scalar as 1024
    TIMSK0 = (1 << OCIE0A) | (1 << TOIE0);          // Timer0 interrupt setup
	initInput();
	DisplayInit();
	DisplayClear();
	srand(67);
    while (1)
    {
		HandleInput(&indexPg, &update, &selection, &scrollIndex, &scrollLength, &confirmSelect);
		if(update && indexPg == 0)
		{
			update = false;
			DisplayClear();
			if(selection == 0)
			{
				DisplayPrintLine(0,0,">1. Start Game");
			}
			else
			{
				DisplayPrintLine(0,0,"1. Start Game");
			}
			if(selection == 1)
			{
				DisplayPrintLine(1,0,">2. How To Play");
			}
			else
			{
				DisplayPrintLine(1,0,"2. How To Play");
			}
			if(selection == 2)
			{
				DisplayPrintLine(2,0,">3. About");	
			}
			else
			{
				DisplayPrintLine(2,0,"3. About");
			}
			DisplayPrintLine(5,0,"        ^    v    >");
		}
		else if(update && indexPg == 1)
		{
			update = false;
			DisplayClear();
			if(selection == 0)
			{
				scrollLength = 25;
				// main game
				if(generate)
				{
					unsigned char randNum = rand() / (RAND_MAX / 5 + 1);
					strcpy_P(strWord, (char*)pgm_read_word(&(TXT_WORDTOGUESS[randNum])));
					bool isLetter = false;
					for(i = 0; i < strlen(strWord); i++)
					{
						for(j = 0; j < strlen(chList); j++)
						{
							if(chList[j] == strWord[i])
							{
								isLetter = true;
							}
						}
						if(!isLetter)
						{
							chList[strlen(chList)] = strWord[i];
						}
						isLetter = false;
					}
					generate = false;
				}
				if(confirmSelect)
				{
					bool isCorrectGuess = false;
					bool isLetter = false;
					for(i = 0; i < strlen(chList); i++)
					{
						if(alphabet[scrollIndex] == chList[i])
						{
							isCorrectGuess = true;
							for(j = 0; j < strlen(chListGuessed); j++)
							{
								if(alphabet[scrollIndex] == chListGuessed[j])
								{
									isLetter = true;
								}
							}
						}
					}
					if(!isLetter && isCorrectGuess)
					{
						chListGuessed[strlen(chListGuessed)] = alphabet[scrollIndex];
					}
					if(!isCorrectGuess)
					{
						strikes++;
						strStrikes[strikes + 8] = 'X';
					}
					confirmSelect = false;
				}
				
				if(strikes < 6)
				{
					if(strlen(chList) == strlen(chListGuessed))
					{
						scrollIndex = 0;
						scrollLength = 0;
						selection = 0;
						strikes = 0;
						confirmSelect = false;
						memset(chList, 0, strlen(chList));
						memset(chListGuessed, 0, strlen(chListGuessed));
						strStrikes[9] = '_';
						strStrikes[10] = '_';
						strStrikes[11] = '_';
						strStrikes[12] = '_';
						strStrikes[13] = '_';
						strStrikes[14] = '_';
						generate = true;
						indexPg = 2;
						update = true;
					}
					else
					{
						DisplayPrintLine(0,0, strStrikes);
						bool isLetter = false;
						for(i = 0; i < strlen(strWord); i++)
						{
							for(j = 0; j < strlen(chListGuessed); j++)
							{
								if(chListGuessed[j] == strWord[i])
								{
									DisplayPrintChar(1, i, strWord[i]);
									isLetter = true;
								}
							}
							if(!isLetter)
							{
								DisplayPrintChar(1, i, '_');
							}
							isLetter = false;
						}
						for(i = 0; i < strlen(chListGuessed); i++)
						{
							DisplayPrintChar(2, i + 1, chListGuessed[i]);
						}
						//DisplayPrintLine(1,0, strWord);
						DisplayPrintChar(2,0, scrollIndex + '0');
						for(i = 0; i < 16; i++)
						{
							if(i < scrollIndex)
							{
								DisplayPrintChar(3, i, alphabet[i]);	
							}
							else if(i > scrollIndex)
							{
								DisplayPrintChar(3, i+1, alphabet[i]);
							}
							else
							{
								DisplayPrintChar(3, i, '>');
								DisplayPrintChar(3, i+1, alphabet[i]);
							}
						}
						for(i = 0; i < 10; i++)
						{
							if((i + 16) < scrollIndex)
							{
								DisplayPrintChar(4, i, alphabet[i + 16]);
							}
							else if((i + 16) > scrollIndex)
							{
								DisplayPrintChar(4, i+1, alphabet[i + 16]);
							}
							else
							{
								DisplayPrintChar(4, i, '>');
								DisplayPrintChar(4, i+1, alphabet[i + 16]);
							}
						}
						DisplayPrintLine(5,0, "   <    ^    v    >");
					}
				}
				else
				{
					scrollIndex = 0;
					scrollLength = 0;
					selection = 0;
					strikes = 0;
					confirmSelect = false;
					memset(chList, 0, strlen(chList));
					memset(chListGuessed, 0, strlen(chListGuessed));
					strStrikes[9] = '_';
					strStrikes[10] = '_';
					strStrikes[11] = '_';
					strStrikes[12] = '_';
					strStrikes[13] = '_';
					strStrikes[14] = '_';
					generate = true;
					indexPg = 3;
					update = true;
				}
				_delay_ms(50);
				
			}
			else if(selection == 1)
			{
				scrollLength = 19;
				char strBuff[30];
				for(i = 0; i < 5; i++)
				{
					// Copy a string from PGM to a buffer to use in printline function
					strcpy_P(strBuff, (char*)pgm_read_word(&(TXT_HOWTOPLAY[i + scrollIndex])));
					DisplayPrintLine(i,0,strBuff);
				}
			}
			else if(selection == 2)
			{
				scrollLength = 6;
				char strBuff[30];
				for(i = 0; i < 5; i++)
				{
					// Copy a string from PGM to a buffer to use in printline function
					strcpy_P(strBuff, (char*)pgm_read_word(&(TXT_ABOUT[i + scrollIndex])));
					DisplayPrintLine(i,0,strBuff);
				}
			}
			DisplayPrintLine(5,0, "   <    ^    v    >");
		}
		else if(update && indexPg == 2)
		{
			DisplayPrintLine(1,5, "You Won!");
			DisplayPrintLine(5,0, "   <");
		}
		else if(update && indexPg == 3)
		{
			DisplayPrintLine(1,4, "You Lost!");
			DisplayPrintLine(5,0, "   <");
		}
    }
}