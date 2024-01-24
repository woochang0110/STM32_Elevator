#include "dotmatrix.h"

extern int current_floor;
extern int target_floor;

GPIO_TypeDef *col_port[]=
{
	COL1_GPIO_Port,COL2_GPIO_Port,COL3_GPIO_Port,COL4_GPIO_Port,
	COL5_GPIO_Port,COL6_GPIO_Port,COL7_GPIO_Port,COL8_GPIO_Port
};

GPIO_TypeDef *row_port[]=
{
	ROW1_GPIO_Port,ROW2_GPIO_Port,ROW3_GPIO_Port,ROW4_GPIO_Port,
	ROW5_GPIO_Port,ROW6_GPIO_Port,ROW7_GPIO_Port,ROW8_GPIO_Port
};

uint16_t col_pin[] =
{
	COL1_Pin,COL2_Pin,COL3_Pin,COL4_Pin,
	COL5_Pin,COL6_Pin,COL7_Pin,COL8_Pin
};

uint16_t row_pin[] =
{
	ROW1_Pin,ROW2_Pin,ROW3_Pin,ROW4_Pin,
	ROW5_Pin,ROW6_Pin,ROW7_Pin,ROW8_Pin
};


unsigned char all_on[] =//all on 문자 정의
{
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111
};

uint8_t smile[8] = {			// 스마일 문자 정의
	0b00000000,
	0b00100000,
	0b01000100,
	0b00100010,
	0b00100010,
	0b01000100,
	0b00100000,
	0b00000000
};


uint8_t up[8] =
{
		0b00011000,
		0b00110000,
		0b01100000,
		0b11100000,
		0b11100000,
		0b01100000,
		0b00110000,
		0b00011000
};

uint8_t down[8] =
{
	0b00011000,
	0b00001100,
	0b00000110,
	0b00000011,
	0b00000011,
	0b00000110,
	0b00001100,
	0b00011000
};

uint8_t number_data[20][10] =
{
	{
		0b01000000,	//1
		0b11000000,
		0b01000000,
		0b01000000,
		0b01000000,
		0b01000000,
		0b11100000
	},
	{
		0b01110000,	//2
		0b10001000,
		0b00001000,
		0b00010000,
		0b00100000,
		0b01000000,
		0b11111000
	},
	{
		0b11111000,	//3
	    0b00010000,
		0b00100000,
		0b00010000,
		0b00001000,
		0b10001000,
		0b01110000
	},
	{
		0b00010000,	//4
		0b00110000,
		0b01010000,
		0b10010000,
		0b11111000,
		0b00010000,
		0b00010000
	},
	{
		0b11111000,	//5
		0b10000000,
		0b11110000,
		0b00001000,
		0b00001000,
		0b10001000,
		0b01110000
	}
};

unsigned char display_data[8];  // 최종 8x8 출력할 데이터
unsigned char scroll_buffer[50][8] = {0};  // 스코롤할 데이타가 들어있는 버퍼
int number_of_character = 5;  // 출력할 문자 갯수



void write_column_data(int col)
{
	for(int i = 0; i < 8; i++)
	{
		if(i == col)
		{
			HAL_GPIO_WritePin(col_port[i], col_pin[i], 0);//on
		}
		else
		{
			HAL_GPIO_WritePin(col_port[i], col_pin[i], 1);//off
		}
	}
}


void write_row_data(unsigned char data)
{
	unsigned char d;

	d = data;//0b00111100

	for(int j = 0; j < 8; j++)
	{
		if (d & (1 << j))//1을 0만큼 shift하면 00000001 , 1만큼 shift하면 00000010 그걸 data와 AND연산
		{
			HAL_GPIO_WritePin(row_port[j],row_pin[j],1);
		}
		else
		{
			HAL_GPIO_WritePin(row_port[j],row_pin[j],0);
		}
	}
}

void init_dotmatrix(void)
{
	for (int i=0; i < 8; i++)
	{
		display_data[i] = number_data[i];
	}
	for (int i=1; i < number_of_character+1; i++)
	{
		for (int j=0; j < 8; j++) // scroll_buffer[0] = blank
		{
			scroll_buffer[i][j] = number_data[i-1][j];
		}
	}
	for (int i=0; i < 8; i++)
	{
		HAL_GPIO_WritePin(col_port[i], col_pin[i], 1); // led all off
	}
}




void dotmatrix_going_up(void)
{
	static uint32_t past_time=0;  // 이전 tick값 저장
	static int count=0;  // 컬럼 count

	uint32_t now = HAL_GetTick();  // 1ms
	// 1.처음시작시 past_time=0; now: 500 --> past_time=500
	if (now - past_time >= 500) // 500ms scroll
	{
		past_time = now;
		for (int i=0; i < 8; i++)
		{
			display_data[i] = (up[i] >> count)|(up[i] << 8 - count);
		}
	}
	if (++count >= 8) // 8칼람을 다 처리 했으면 다음 scroll_buffer로 이동
	{
		count =0;
	}
	for (int i=0; i < 8; i++)
	{
		// 공통 양극 방식
		// column에는 0을 ROW에는 1을 출력해야 해당 LED가 on된다.
		write_column_data(i);
		write_row_data(display_data[i]);
		HAL_Delay(1);
	}
}
void dotmatrix_going_down(void)
{
	static uint32_t past_time=0;  // 이전 tick값 저장
	static int count=0;  // 컬럼 count

	uint32_t now = HAL_GetTick();  // 1ms
	// 1.처음시작시 past_time=0; now: 500 --> past_time=500
	if (now - past_time >= 500) // 500ms scroll
	{
		past_time = now;
		for (int i=0; i < 8; i++)
		{

			display_data[i] = (down[i] << count) |
					(down[i] >> 8 - count);
		}
	}
	if (++count >= 8) // 8칼람을 다 처리 했으면 다음 scroll_buffer로 이동
	{
		count =0;
	}
	for (int i=0; i < 8; i++)
	{
		// 공통 양극 방식
		// column에는 0을 ROW에는 1을 출력해야 해당 LED가 on된다.
		write_column_data(i);
		write_row_data(display_data[i]);
		HAL_Delay(1);
	}
}
void dotmatrix_smile(void)
{
	for (int i=0; i < 8; i++)
	{
		// 공통 양극 방식
		// column에는 0을 ROW에는 1을 출력해야 해당 LED가 on된다.
		write_column_data(i);
		write_row_data(smile[i]);
		HAL_Delay(1);
	}
}




void dotmatrix_main(void)
{
	static int count=0;  // 컬럼 count
	static int index=0;  // scroll_buffer의 2차원 index값
	static uint32_t past_time=0;  // 이전 tick값 저장

	uint32_t now = HAL_GetTick();  // 1ms
	// 1.처음시작시 past_time=0; now: 500 --> past_time=500
	if (now - past_time >= 500) // 500ms scroll
	{
		past_time = now;
		for (int i=0; i < 8; i++)
		{

			display_data[i] = (scroll_buffer[index][i] >> count) |
					(scroll_buffer[index+1][i] << 8 - count);
		}
		if (++count == 8) // 8칼람을 다 처리 했으면 다음 scroll_buffer로 이동
		{
			count =0;
			index++;  // 다음 scroll_buffer로 이동
			if (index == number_of_character+1) index=0;
			// 11개의 문자를 다 처리 했으면 0번 scroll_buffer를 처리 하기위해 이동
		}
	}
	for (int i=0; i < 8; i++)
	{
		// 공통 양극 방식
		// column에는 0을 ROW에는 1을 출력해야 해당 LED가 on된다.
		write_column_data(i);
		write_row_data(display_data[i]);
		HAL_Delay(1);
	}
}


