#ifndef KEYPAD_H
#define KEYPAD_H

/*
*		GPIO		|		KEYPAD		|		PIN-MODE			
*		PA3			|		ROW1		|		INPUT
*		PA4			|		ROW2		|		INPUT
*		PA5			|		ROW3		|		INPUT
*		PA6			|		ROW4		|		INPUT
*		PA7			|		COLUMN1		|		OUTPUT
*		PB0			|		COLUMN2		|		OUTPUT
*		PB1			|		COLUMN3		|		OUTPUT
*		PB10	   	|		COLUMN4		|		OUTPUT
*/


extern void keyPad_Init(void);
extern char get_keyPadChar(void);
void registerKeyPress(char key, char* keyBuffer, int len);


#endif/* KEYPAD_H */
