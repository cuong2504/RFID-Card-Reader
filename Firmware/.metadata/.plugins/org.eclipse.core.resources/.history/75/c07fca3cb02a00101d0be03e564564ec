#include "stm32f10x.h"                  // Device header
#include "keypad.h"
#include <stdbool.h>
#include "sysTimer_struct.h"
#include "systick.h"
#include "gpio.h"

#define KEY_PRESSED  				0
#define NUMBER_OF_COLUMNS   4
#define NUMBER_OF_ROWS			4
#define Null			   			'\0'

enum rows{
	ROW1 = 1,
	ROW2 = 4,
	ROW3 = 5,
	ROW4 = 6
};

enum column{
	COLUMN1,
	COLUMN2,
	COLUMN3,
	COLUMN4
};

static uint8_t keyPadRowPin[4] = {ROW1,ROW2,ROW3,ROW4};

static char KEYPAD[4][4] = {{'1','4','7','*'},
														{'2','5','8','0'},
														{'3','6','9','#'},
														{'A','B','C','D'}
													};

static void activateColumn(int column){
	switch(column){
	case COLUMN1:
		//clear PB1 and set PB5,8,9
		GPIO_Output_Clear_Reg(GPIOB, 1<<GPIO_PIN1);
		GPIO_Output_Reg_Write(GPIOB, ((1<<GPIO_PIN5) | (GPIO_PIN8) | (GPIO_PIN9)) );
		break;
	case COLUMN2:
		//clear PB5 and set PB1,8,9
		GPIO_Output_Clear_Reg(GPIOB, 1<<GPIO_PIN5);
		GPIO_Output_Reg_Write(GPIOB, ((1<<GPIO_PIN1) | (1<<GPIO_PIN8) | (1<<GPIO_PIN9)));
		break;
	case COLUMN3:
		//clear PB8 and set PB1,5,9
		GPIO_Output_Clear_Reg(GPIOB, 1<<GPIO_PIN8);
		GPIO_Output_Reg_Write(GPIOB, ((1<<GPIO_PIN1) | (1<<GPIO_PIN5) | (1<<GPIO_PIN9)));
		break;
	case COLUMN4:
		//clear PB9 and set PB1,5,8
		GPIO_Output_Clear_Reg(GPIOB, 1<<GPIO_PIN9);
		GPIO_Output_Reg_Write(GPIOB, ((1<<GPIO_PIN1) | (1<<GPIO_PIN5) | (1<<GPIO_PIN8)));
		break;
	}
}


void keyPad_Init(void)
{
	GPIO_Reset(GPIOA);
	GPIO_Reset(GPIOB);
	//Configure PB1,5,8,9, as output
	GPIO_Output_Init( GPIOB, 0 , (GPIO_PIN1_OUTPUT_MODE_2MHZ | GPIO_PIN5_OUTPUT_MODE_2MHZ), GPIO_GEN_PUR_OUTPUT_PUSH_PULL );
	GPIO_Output_Init( GPIOB, 1 , (GPIO_PIN8_OUTPUT_MODE_2MHZ | GPIO_PIN9_OUTPUT_MODE_2MHZ), GPIO_GEN_PUR_OUTPUT_PUSH_PULL );
	
	//Configure PA1,4,5,6 as input with pull-up	
	GPIO_Input_Init_Reg(GPIOA, 0 , ((1<<GPIO_PIN1) | (1<<GPIO_PIN4) | (1<<GPIO_PIN5) | (1<<GPIO_PIN6)),
																	(GPIO_PIN1_INPUT_PULLUP_OR_PULLDOWN | GPIO_PIN4_INPUT_PULLUP_OR_PULLDOWN | 
																	GPIO_PIN5_INPUT_PULLUP_OR_PULLDOWN | GPIO_PIN6_INPUT_PULLUP_OR_PULLDOWN),
																	GPIO_PULLUP_ENABLE);

	
}

char get_keyPadChar(void){

	for(int i = 0; i < NUMBER_OF_COLUMNS; i++){
		activateColumn(i);
		for(int j = 0; j < NUMBER_OF_ROWS; j++){
			/*if a key is pressed,
			  wait a bit for the keypad to stabilize and then return character corresponding to it
			 */
			
			if(GPIO_Input_Read(GPIOA, keyPadRowPin[j]) == KEY_PRESSED){
			SysTick_DelayMs(300); //300ms delay
			return KEYPAD[i][j];
				
			}
		}
	}
	return Null;
}

void registerKeyPress(char key, char* keyBuffer, int len)
{
	for(int i = 0; i < len -1; i++)
	{
		keyBuffer[i] = keyBuffer[i + 1];
	}
	
	keyBuffer[len-1] = key;
}


