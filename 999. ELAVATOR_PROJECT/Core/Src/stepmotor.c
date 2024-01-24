#include "stepmotor.h"


// RPM(Revolutions Per Minutes) : 분당 회전수
// 1분: 60sec : 1,000,000us(1초) x 60 = 60,000,000us
// 1,000,000us(1초)
//  --> 1초(1000ms) ==> 1ms(1000us) x 1000ms ==> 1,000,000us
//  4096스텝 : 1바퀴(4096 스텝이동)
//--- 1바퀴도는데 필요한 총 스텝수 : 4096
// 4096 / 8(0.7) ==> 512 sequence : 360도
// 1 sequence(8step) : 0.70312도
// 0.70312도 x 512sequence = 360

//------- set_rpm(13) 으로 지정시의 동작 상황 ---
// 60,000,000us(1분) / 4096 / rpm
// 1126us(1스텝 idle타임) x4096 = 4,612,096us
//                           = 4612ms
//                           = 4.6초
// 60초 / 4.6(1회전시 소요시간 초) ==> 13회전
// 시계방향으로 1회전 <---> 반시계방향으로 1회전
void set_rpm(int rpm) // rpm 1~ 13
{
	delay_us(60000000/4096/rpm);
	// 최대 speed 기준(13) : delay_us(1126);
}
void stepmotor_main_test(void)
{
	for(int i=0; i < 2048; i++)  //512 => 시계방향 1회전
	{
		for (int j=0; j < 8; j++)
		{
			stepmotor_drive(j);
			set_rpm(13);
		}
	}

	for(int i=0; i < 2048; i++)  //반시계방향 1회전
	{
		for (int j=7; j >= 0; j--)
		{
			stepmotor_drive(j);
			set_rpm(13);  // rpm값만큼 wait
		}
	}
}

void stepmotor_forward(int forward_active_flag)
{
#if 1
	if(forward_active_flag==1)
	{
		static int step = 0;
		step = (step+1)%8;
		stepmotor_drive(step);
		set_rpm(13);
	}

#else

	for(int i=0; i < 850; i++)  //512 => 시계방향 1회전
	{
		for (int j=0; j < 8; j++)
		{
			stepmotor_drive(j);
			set_rpm(13);
		}
	}
#endif
}

void stepmotor_backward(int backward_active_flag)
{
#if 1

	if(backward_active_flag==1)
	{

		static int step = 8;
		step = (step-1);
		if(step==-1){step=7;}
		stepmotor_drive(step);
		set_rpm(13);
	}

#else
	for(int i=0; i < 850; i++)  //반시계방향 1회전
	{
		for (int j=7; j >= 0; j--)
		{
			stepmotor_drive(j);
			set_rpm(13);  // rpm값만큼 wait
		}
	}
#endif
}

void stepmotor_drive(int step)
{
	switch(step){
	case 0:
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 1);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 1);
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 1);
		break;
	}
}


