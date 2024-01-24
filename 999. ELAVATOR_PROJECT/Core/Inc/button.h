#include "main.h"
#include "stepmotor.h"

#define BUTTON_RELEASE 1
#define BUTTON_PRESS 0
#define BUTTON_NUMBER 5
void button_pushed();
int get_button(GPIO_TypeDef *GPIO, uint16_t GPIO_PIN, uint8_t button_number);

