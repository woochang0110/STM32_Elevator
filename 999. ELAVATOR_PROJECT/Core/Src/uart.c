#include "uart.h"
#include <string.h>   // strncmp, strcpy....
#include <stdlib.h>

void pc_command_processing(void);

uint8_t rx_data;   // uart3 rx data

extern UART_HandleTypeDef huart3;  // PC

// HW와 SW의 만나는 약속장소 : call back function
// move from HAL_UART_RxCpltCallback of stm32f4xx_hal_uart to here
// UART로 1 byte가 수신되면 H/W가 call을 해 준다.
// UART RX INT가 발생이 되면 이곳으로 자동적으로 들어 온다.

#if 0
#define COMMAND_SU 20
#define COMMAND_LENGTH 40
volatile unsigned char rx_buff[COMMAND_SU][COMMAND_LENGTH];  // UART3으로부터 수신된 char를 저장하는 공간(\n을 만날때 까지)
volatile int rx_index=0;  // rx_buff의 save위치
volatile int newline_detect_flag=0;  // new line을 만났을때의 indicator 예) ledallon\n
#else  // orginal
#define COMMAND_LENGTH 40
volatile unsigned char rx_buff[COMMAND_LENGTH];  // UART3으로부터 수신된 char를 저장하는 공간(\n을 만날때 까지)
volatile int rx_index=0;  // rx_buff의 save위치
volatile int newline_detect_flag=0;  // new line을 만났을때의 indicator 예) ledallon\n
#endif
volatile unsigned char bt_rx_buff[COMMAND_LENGTH];  // UART6으로부터 수신된 char를 저장하는 공간(\n을 만날때 까지)
volatile int bt_rx_index=0;  // bt rx_buff의 save위치
volatile int bt_newline_detect_flag=0;  // new line을 만났을때의 indicator 예) ledallon\n

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3)   // comport master와 연결된 uart
	{
		if (rx_index < COMMAND_LENGTH) // 현재까지 들어온 byte가 40byte를 넘지 않으면
		{
			if (rx_data == '\n' || rx_data == '\r')
			{
				rx_buff[rx_index] = 0; // '\0'
				newline_detect_flag=1;   // new line을 만났다는 flag를 set한다.
				rx_index=0;   // 다음 message저장을 위해서 rx_index값을 0으로 한다.
			}
			else
			{
				rx_buff[rx_index++]=rx_data;
			}

		}
		else
		{
			rx_index=0;
			printf("Message Overflow !!!!\n");
		}
		// 주의: 반드시 HAL_UART_Receive_IT를 call 해줘야 다음 INT가 발생이 된다.
		HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	}

}
/*
void pc_command_processing(void)
{
	if (newline_detect_flag)   // comport master로 부터 완전한 문장이 들어 오면 (\n을 만나면)
	{
		newline_detect_flag=0;
		printf("%s\n", rx_buff);
		if (!strncmp(rx_buff, "led_all_on", strlen("led_all_on")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			led_all_on();
			return;
		}
		if (!strncmp(rx_buff, "led_all_off", strlen("led_all_off")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			led_all_off();
			return;
		}
		if (!strncmp(rx_buff, "led_on_down", strlen("led_on_down")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			led_on_down();
			return;
		}
		if (!strncmp(rx_buff, "led_on_up", strlen("led_on_up")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			led_on_up();
			return;
		}
		if (!strncmp(rx_buff, "flower_on", strlen("flower_on")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			flower_on();
			return;
		}
		if (!strncmp(rx_buff, "flower_off", strlen("flower_off")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			flower_off();
			return;
		}
		if (!strncmp(rx_buff, "led_keepon_up", strlen("led_keepon_up")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			led_keepon_up();
			return;
		}
		if (!strncmp(rx_buff, "led_keepon_down", strlen("led_keepon_down")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			led_keepon_down();
			return;
		}
		//dht11time150
		if (!strncmp(rx_buff, "dht11time", strlen("dht11time")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			dht11time = atoi(rx_buff+9);
			return;
		}
		if (!strncmp(rx_buff, "setrtc", strlen("setrtc")))  // if (strncmp(rx_buff, "ledallon", strlen("ledallon") == 0)
		{
			set_rtc(rx_buff);
			return;
		}

	}
}
*/
