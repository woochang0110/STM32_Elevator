#include "button.h"
extern int target_floor;
extern int led1_count;
extern int led2_count;
extern int led3_count;
extern int stepmotor_activate;

char button_status[BUTTON_NUMBER] =
{BUTTON_RELEASE,BUTTON_RELEASE,BUTTON_RELEASE,BUTTON_RELEASE,BUTTON_RELEASE};






int get_button(GPIO_TypeDef *GPIO, uint16_t GPIO_PIN, uint8_t button_number)
{
	unsigned char curr_state;
	curr_state = HAL_GPIO_ReadPin(GPIO,GPIO_PIN);
	if(curr_state == BUTTON_PRESS && button_status[button_number] == BUTTON_RELEASE)
	{
		HAL_Delay(100);
		button_status[button_number]=BUTTON_PRESS;
		return BUTTON_RELEASE;
	}
	else if(curr_state == BUTTON_RELEASE && button_status[button_number] == BUTTON_PRESS)
	{
		button_status[button_number]=BUTTON_RELEASE;
		return BUTTON_PRESS;
	}
	return BUTTON_RELEASE;
}

void button_pushed()
{
	  if(get_button(BTN_0_GPIO_Port, BTN_0_Pin, 0)== BUTTON_PRESS)
	  {
		  if(target_floor==1)
		  {
			  stepmotor_activate=0;
		  }
		  else
		  {
			  target_floor = 1;
			  stepmotor_activate=1;
		  }
	  }
	  if(get_button(BTN_1_GPIO_Port, BTN_1_Pin, 1)== BUTTON_PRESS)
	  {
		  if(target_floor==2)
		  {
			  stepmotor_activate=0;
		  }
		  else
		  {
			  target_floor = 2;
			  stepmotor_activate=1;
		  }
	  }
	  if(get_button(BTN_2_GPIO_Port, BTN_2_Pin, 2)== BUTTON_PRESS)
	  {
		  if(target_floor==3)
		  {
			  stepmotor_activate=0;
		  }
		  else
		  {
			  target_floor = 3;
			  stepmotor_activate=1;
		  }
	  }
	  if(get_button(BTN_3_GPIO_Port, BTN_3_Pin, 3)== BUTTON_PRESS)
	  {
		  if(target_floor==4)
		  {
			  stepmotor_activate=0;
		  }
		  else
		  {
			  target_floor = 4;
			  stepmotor_activate=1;
		  }
	  }
	  if(get_button(BTN_4_GPIO_Port, BTN_4_Pin, 4)== BUTTON_PRESS)
	  {
		  if(target_floor==5)
		  {
			  stepmotor_activate=0;
		  }
		  else
		  {
			  target_floor = 5;
			  stepmotor_activate=1;
		  }
	  }
}
