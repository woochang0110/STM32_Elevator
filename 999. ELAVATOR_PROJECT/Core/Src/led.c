#include "led.h"

extern int led1_count;
extern int led2_count;
extern int led3_count;


void led_main()
{
	  if(led1_count%2 == 1)
	  {
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);//led1 green(PB0)
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);//led1 green(PB0)
	  }
	  if(led2_count%2 == 1)
	  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);//led2 blue(PB7)
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);//led2 blue(PB7)
	  }

	  if(led3_count%2 == 1)
	  {
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);//led3 red(PB14)
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);//led3 red(PB14)
	  }
}

