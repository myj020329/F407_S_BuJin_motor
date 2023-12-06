#include "S_Speed.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stdio.h"

/*�㷨��ؽṹ���������*/
SpeedCalc_TypeDef Speed ;
Stepper_Typedef Stepper;

uint8_t print_flag=0;

void CalcSpeed(int32_t Vo, int32_t Vt, float T)
{
	
  uint8_t Is_Dec = FALSE;     
  int32_t i = 0;
  int32_t Vm =0;              // �м���ٶ�
  float K = 0;             // �Ӽ��ٶ�
  float Ti = 0;               // ʱ���� dt
  float Sumt = 0;             // ʱ���ۼ���
  float DeltaV = 0;           // �ٶȵ�����dv  
	
	/***************************************************************************/
	/*�жϳ��ٶȺ�ĩ�ٶȵĹ�ϵ���������Ӽ���*/
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
	/*�����ʼ����*/
	T = T / 2;						//�Ӽ��ٶε�ʱ�䣨���ٶ�б��>0��ʱ�䣩
	
  Vm = (Speed.Vo + Speed.Vt) / 2;	//�����е���ٶ�
	
  K = fabs(( 2 * ((Vm) - (Speed.Vo)) ) / pow((T),2));// �����е��ٶȼ���Ӽ��ٶ�
		
  Speed.INC_AccelTotalStep = (int32_t)((Speed.Vo * T)+(((K) * pow((T), 3)) / 6));// �Ӽ�����Ҫ�Ĳ���
	
  Speed.Dec_AccelTotalStep = (int32_t)(((Speed.Vt + Speed.Vo) * T - Speed.INC_AccelTotalStep));   // ��������Ҫ�Ĳ��� S = Vt * Time - S1
  
	/***************************************************************************/
	/*���㹲��Ҫ�Ĳ�������У���ڴ��С�������ڴ�ռ����ٶȱ�*/
  Speed.AccelTotalStep = Speed.Dec_AccelTotalStep + Speed.INC_AccelTotalStep;              // ������Ҫ�Ĳ��� 
  if( Speed.AccelTotalStep  % 2 != 0)     // ���ڸ���������ת�����������ݴ��������,���������1
    Speed.AccelTotalStep  += 1;
	
	/*�ж��ڴ泤��*/
	if(FORM_LEN<Speed.AccelTotalStep)
	{
		printf("FORM_LEN ���泤�Ȳ���\r\n,�뽫 FORM_LEN �޸�Ϊ %d \r\n",Speed.AccelTotalStep);
		return ;
	}

	/***************************************************************************/
	/* �����һ����ʱ�� */
		
	/*���ݵ�һ����ʱ����㣬��һ�����ٶȺ�����ʱ����*/
	/*����λ��Ϊ0��ʱ������������ʱ��Ĺ�ϵʽ ->  ����λ�ƺ�ʱ��Ĺ�ʽS = 1/2 * K * Ti^3  �ɵ� Ti=6 * 1 / K��1/3�η� */
  Ti = pow((6.0f * 1.0f / K),(1 / 3.0f) ); //������� Ti ʱ�䳣��
  Sumt += Ti;//�ۼ�ʱ�䳣��
	/*����V=1/2*K*T^2,���Լ����һ�����ٶ�*/
  DeltaV = 0.5f * K * pow(Sumt,2);
	/*�ڳ�ʼ�ٶȵĻ����������ٶ�*/
  Speed.Form[0] = Speed.Vo + DeltaV;
  
	/***************************************************************************/
	/*��С�ٶ��޷�*/
  if( Speed.Form[0] <= MIN_SPEED )//�Ե�ǰ��ʱ��Ƶ�����ܴﵽ������ٶ�
    Speed.Form[0] = MIN_SPEED;
	
  /***************************************************************************/
	/*����S���ٶȱ�*/
  for(i = 1; i < Speed.AccelTotalStep; i++)
  {
	
		/*����ʱ��������Ƶ�ʳɷ��ȵĹ�ϵ�����Լ����Ti,������ÿ�μ�����һ��ʱ�䣬���ڻ��۵���ǰʱ��*/
		Ti = 1.0f / Speed.Form[i-1];   
    /* �Ӽ��ٶȼ��� */
    if( i < Speed.INC_AccelTotalStep)
    {
			/*�ۻ�ʱ��*/
      Sumt += Ti;
			/*�ٶȵı仯�� dV = 1/2 * K * Ti^2 */
      DeltaV = 0.5f * K * pow(Sumt,2);
			/*���ݳ�ʼ�ٶȺͱ仯������ٶȱ�*/
      Speed.Form[i] = Speed.Vo + DeltaV;  
			/*Ϊ�˱�֤�����һ������ʹ��ʱ���Ͻ�����Ԥ�ڼ����ʱ��һ�£������һ�����д���*/
      if(i == Speed.INC_AccelTotalStep - 1)
        Sumt  = fabs(Sumt - T );
    }
    /* �����ٶȼ��� */
    else
    {
			/*ʱ���ۻ�*/
      Sumt += Ti;                                       
			/*�����ٶ�*/
      DeltaV = 0.5f * K * pow(fabs( T - Sumt),2); 
      Speed.Form[i] = Speed.Vt - DeltaV;          
    }
  } 
	/***************************************************************************/
	/*�����˶�����������*/
  if(Is_Dec == TRUE)
  {
    float tmp_Speed = 0;  
    /* �������� */
    for(i = 0; i< (Speed.AccelTotalStep / 2); i++)
    {
      tmp_Speed = Speed.Form[i];
      Speed.Form[i] = Speed.Form[Speed.AccelTotalStep-1 - i];
      Speed.Form[Speed.AccelTotalStep-1 - i] = tmp_Speed;
    }
  }
}


/**
  * @brief  �ٶȾ���
	*	@note 	���ж���ʹ�ã�ÿ��һ���жϣ�����һ��
  * @retval ��
  */
void speed_decision(void)
{
	/*������ʱ����*/
  float temp_p = 0;
	/*�������*/
  static uint8_t i = 0;  	
  
	if(__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_CC1) !=RESET)
	{
		/*�����ʱ���ж�*/
		__HAL_TIM_CLEAR_IT(&htim8, TIM_IT_CC1);
		
		/******************************************************************/
		/*����Ϊһ����������*/
		i++; 
    if(i == 2)
    {
			/*������������������*/
      i = 0;   
			/*�жϵ�ǰ��״̬��*/
      if(Stepper.status == ACCEL || Stepper.status == DECEL)
      {
				/*����λ����������*/
        Stepper.pos++;
        if(Stepper.pos  < Speed.AccelTotalStep )
        { 
					/*��ȡÿһ���Ķ�ʱ������ֵ*/
          temp_p = T1_FREQ / Speed.Form[Stepper.pos];
          if((temp_p / 2) >= 0xFFFF)
            {Stepper.pluse_time = 0xFFFF;}
          else
            {Stepper.pluse_time = (uint16_t)roundf((temp_p / 2));}
        }
        else
        {
					/*���ٲ��ֽ������������������״̬����ֹͣ״̬*/
          if(Stepper.status == ACCEL)   
					{
					  Stepper.status = AVESPEED;
					}          
          else
          {
						/*ֹͣ״̬������ٶȱ��ҹر�ͨ��*/
            Stepper.status = STOP; 
						memset(Speed.Form,0,sizeof(float)*FORM_LEN);
            TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_DISABLE);// ʹ�ܶ�ʱ��ͨ�� 
            
          }
        }
      }
    }
		/**********************************************************************/
		// ��ȡ��ǰ��������ֵ
		uint32_t tim_count=__HAL_TIM_GET_COUNTER(&htim8);
		/*������һ��ʱ��*/
		uint32_t tmp = tim_count+Stepper.pluse_time;
		/*���ñȽ�ֵ*/
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,tmp);
		
	}
}


/**
  * @brief  ��ʼ��״̬�������õ�һ�����ٶ�
  * @param  ��
	* @param  ��
	*	@note 		��
  * @retval ��
  */
void stepper_start_run()
{

	/*��ʼ���ṹ��*/
	memset(&Stepper,0,sizeof(Stepper_Typedef));
	/*��ʼ���״̬*/
	Stepper.status=ACCEL;
	/*��ʼ���λ��*/
	Stepper.pos=0;
	
	/*�����һ��������*/
  if(Speed.Form[0] == 0)	//�ų���ĸΪ0�����
    Stepper.pluse_time = 0xFFFF;
  else										//��ĸ��Ϊ0�����
    Stepper.pluse_time  = (uint32_t)(T1_FREQ/Speed.Form[0]);
	
	/*��ȡ��ǰ����ֵ*/
	uint32_t temp=__HAL_TIM_GET_COUNTER(&htim8);
	/*�ڵ�ǰ����ֵ���������ö�ʱ���Ƚ�ֵ*/
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,temp +Stepper.pluse_time); 
	/*�����ж�����Ƚ�*/
	HAL_TIM_OC_Start_IT(&htim8,TIM_CHANNEL_1);
	/*ʹ�ܶ�ʱ��ͨ��*/
	TIM_CCxChannelCmd(TIM8, TIM_CHANNEL_1, TIM_CCx_ENABLE);
}


/*! \brief ���̶���ʱ����ٶȣ�ʹ�ò�������ڹ̶�ʱ���ڴﵽĿ���ٶ�
 *  \param start_speed   	��ʼ�ٶ�
 *  \param end_speed  		�����ٶ�
 *  \param time  					ʱ��
 */
void stepper_move_S(int start_speed,int end_speed,float time)
{
	/*�������*/
	CalcSpeed(start_speed,end_speed,time);
	/*��ʼ��ת*/
	stepper_start_run();
}

void TIM8_CC_IRQHandler(void)
{ 
	HAL_TIM_IRQHandler(&htim8);
	/*�ٶ�״̬����*/
	speed_decision();
}
