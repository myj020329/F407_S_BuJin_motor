#include "S_Speed.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stdio.h"

/*算法相关结构体变量定义*/
SpeedCalc_TypeDef Speed ;
Stepper_Typedef Stepper;

uint8_t print_flag=0;

void CalcSpeed(int32_t Vo, int32_t Vt, float T)
{
	
  uint8_t Is_Dec = FALSE;     
  int32_t i = 0;
  int32_t Vm =0;              // 中间点速度
  float K = 0;             // 加加速度
  float Ti = 0;               // 时间间隔 dt
  float Sumt = 0;             // 时间累加量
  float DeltaV = 0;           // 速度的增量dv  
	
	/***************************************************************************/
	/*判断初速度和末速度的关系，来决定加减速*/
  if(Vo > Vt )
	{                               
    Is_Dec = TRUE;
    Speed.Vo = CONVER(Vt);  
    Speed.Vt = CONVER(Vo); 
  }
  else
  {
    Is_Dec = FALSE;
    Speed.Vo = CONVER(Vo);    
    Speed.Vt = CONVER(Vt);    
  }
	/***************************************************************************/
	/*计算初始参数*/
	T = T / 2;						//加加速段的时间（加速度斜率>0的时间）
	
  Vm = (Speed.Vo + Speed.Vt) / 2;	//计算中点的速度
	
  K = fabs(( 2 * ((Vm) - (Speed.Vo)) ) / pow((T),2));// 根据中点速度计算加加速度
		
  Speed.INC_AccelTotalStep = (int32_t)((Speed.Vo * T)+(((K) * pow((T), 3)) / 6));// 加加速需要的步数
	
  Speed.Dec_AccelTotalStep = (int32_t)(((Speed.Vt + Speed.Vo) * T - Speed.INC_AccelTotalStep));   // 减加速需要的步数 S = Vt * Time - S1
  
	/***************************************************************************/
	/*计算共需要的步数，并校检内存大小，申请内存空间存放速度表*/
  Speed.AccelTotalStep = Speed.Dec_AccelTotalStep + Speed.INC_AccelTotalStep;              // 加速需要的步数 
  if( Speed.AccelTotalStep  % 2 != 0)     // 由于浮点型数据转换成整形数据带来了误差,所以这里加1
    Speed.AccelTotalStep  += 1;
	
	/*判断内存长度*/
	if(FORM_LEN<Speed.AccelTotalStep)
	{
		printf("FORM_LEN 缓存长度不足\r\n,请将 FORM_LEN 修改为 %d \r\n",Speed.AccelTotalStep);
		return ;
	}

	/***************************************************************************/
	/* 计算第一步的时间 */
		
	/*根据第一步的时间计算，第一步的速度和脉冲时间间隔*/
	/*根据位移为0的时候的情况，计算时间的关系式 ->  根据位移和时间的公式S = 1/2 * K * Ti^3  可得 Ti=6 * 1 / K开1/3次方 */
  Ti = pow((6.0f * 1.0f / K),(1 / 3.0f) ); //开方求解 Ti 时间常数
  Sumt += Ti;//累计时间常数
	/*根据V=1/2*K*T^2,可以计算第一步的速度*/
  DeltaV = 0.5f * K * pow(Sumt,2);
	/*在初始速度的基础上增加速度*/
  Speed.Form[0] = Speed.Vo + DeltaV;
  
	/***************************************************************************/
	/*最小速度限幅*/
  if( Speed.Form[0] <= MIN_SPEED )//以当前定时器频率所能达到的最低速度
    Speed.Form[0] = MIN_SPEED;
	
  /***************************************************************************/
	/*计算S形速度表*/
  for(i = 1; i < Speed.AccelTotalStep; i++)
  {
	
		/*根据时间周期与频率成反比的关系，可以计算出Ti,在这里每次计算上一步时间，用于积累到当前时间*/
		Ti = 1.0f / Speed.Form[i-1];   
    /* 加加速度计算 */
    if( i < Speed.INC_AccelTotalStep)
    {
			/*累积时间*/
      Sumt += Ti;
			/*速度的变化量 dV = 1/2 * K * Ti^2 */
      DeltaV = 0.5f * K * pow(Sumt,2);
			/*根据初始速度和变化量求得速度表*/
      Speed.Form[i] = Speed.Vo + DeltaV;  
			/*为了保证在最后一步可以使得时间严谨的与预期计算的时间一致，在最后一步进行处理*/
      if(i == Speed.INC_AccelTotalStep - 1)
        Sumt  = fabs(Sumt - T );
    }
    /* 减加速度计算 */
    else
    {
			/*时间累积*/
      Sumt += Ti;                                       
			/*计算速度*/
      DeltaV = 0.5f * K * pow(fabs( T - Sumt),2); 
      Speed.Form[i] = Speed.Vt - DeltaV;          
    }
  } 
	/***************************************************************************/
	/*减速运动，倒序排列*/
  if(Is_Dec == TRUE)
  {
    float tmp_Speed = 0;  
    /* 倒序排序 */
    for(i = 0; i< (Speed.AccelTotalStep / 2); i++)
    {
      tmp_Speed = Speed.Form[i];
      Speed.Form[i] = Speed.Form[Speed.AccelTotalStep-1 - i];
      Speed.Form[Speed.AccelTotalStep-1 - i] = tmp_Speed;
    }
  }
}


/**
  * @brief  速度决策
	*	@note 	在中断中使用，每进一次中断，决策一次
  * @retval 无
  */
void speed_decision(void)
{
	/*计数临时变量*/
  float temp_p = 0;
	/*脉冲计数*/
  static uint8_t i = 0;  	
  
	if(__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_CC1) !=RESET)
	{
		/*清除定时器中断*/
		__HAL_TIM_CLEAR_IT(&htim8, TIM_IT_CC1);
		
		/******************************************************************/
		/*两次为一个脉冲周期*/
		i++; 
    if(i == 2)
    {
			/*脉冲周期完整后清零*/
      i = 0;   
			/*判断当前的状态，*/
      if(Stepper.status == ACCEL || Stepper.status == DECEL)
      {
				/*步数位置索引递增*/
        Stepper.pos++;
        if(Stepper.pos  < Speed.AccelTotalStep )
        { 
					/*获取每一步的定时器计数值*/
          temp_p = T1_FREQ / Speed.Form[Stepper.pos];
          if((temp_p / 2) >= 0xFFFF)
            {Stepper.pluse_time = 0xFFFF;}
          else
            {Stepper.pluse_time = (uint16_t)roundf((temp_p / 2));}
        }
        else
        {
					/*加速部分结束后接下来就是匀速状态或者停止状态*/
          if(Stepper.status == ACCEL)   
					{
					  Stepper.status = AVESPEED;
					}          
          else
          {
						/*停止状态，清空速度表并且关闭通道*/
            Stepper.status = STOP; 
						memset(Speed.Form,0,sizeof(float)*FORM_LEN);
            TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_DISABLE);// 使能定时器通道 
            
          }
        }
      }
    }
		/**********************************************************************/
		// 获取当前计数器数值
		uint32_t tim_count=__HAL_TIM_GET_COUNTER(&htim8);
		/*计算下一次时间*/
		uint32_t tmp = tim_count+Stepper.pluse_time;
		/*设置比较值*/
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,tmp);
		
	}
}


/**
  * @brief  初始化状态并且设置第一步的速度
  * @param  无
	* @param  无
	*	@note 		无
  * @retval 无
  */
void stepper_start_run()
{

	/*初始化结构体*/
	memset(&Stepper,0,sizeof(Stepper_Typedef));
	/*初始电机状态*/
	Stepper.status=ACCEL;
	/*初始电机位置*/
	Stepper.pos=0;
	
	/*计算第一次脉冲间隔*/
  if(Speed.Form[0] == 0)	//排除分母为0的情况
    Stepper.pluse_time = 0xFFFF;
  else										//分母不为0的情况
    Stepper.pluse_time  = (uint32_t)(T1_FREQ/Speed.Form[0]);
	
	/*获取当前计数值*/
	uint32_t temp=__HAL_TIM_GET_COUNTER(&htim8);
	/*在当前计数值基础上设置定时器比较值*/
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,temp +Stepper.pluse_time); 
	/*开启中断输出比较*/
	HAL_TIM_OC_Start_IT(&htim8,TIM_CHANNEL_1);
	/*使能定时器通道*/
	TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_ENABLE);
}


/*! \brief 给固定的时间和速度，使得步进电机在固定时间内达到目标速度
 *  \param start_speed   	初始速度
 *  \param end_speed  		结束速度
 *  \param time  					时间
 */
void stepper_move_S(int start_speed,int end_speed,float time)
{
	/*计算参数*/
	CalcSpeed(start_speed,end_speed,time);
	/*开始旋转*/
	stepper_start_run();
}

void TIM8_CC_IRQHandler(void)
{ 
	HAL_TIM_IRQHandler(&htim8);
	/*速度状态决策*/
	speed_decision();
}
