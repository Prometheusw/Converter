/********************************Copyright (c)**********************************\
**                   (c) Copyright 2019, Main, China
**                           All Rights Reserved
**                           By(��ʵ����ҽ�ƿƼ����޹�˾)
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: Timer_Dri.c
** ������Ա: ����
** ��������: 2019-07-17
** �ĵ�����:
******************************************************************************/
#include "myconfig.h"
T_HeartBeat HeartBeat= {0,0}; //������־
/*******************************************************************************
** ��������: Timer2_Init
** ��������: Modbus�ֽڼ����ʱ  1us'  1.5�ַ����
** ����˵��: arr: 1000-1
**			 psc: 84-1
** ����˵��: None
** ������Ա: ����
** ��������: 2019-07-31
********************************************************************************/
void Timer2_Init(u16 arr,u16 psc)    //100us����1�θ����¼�
{
    TIM_TimeBaseInitTypeDef timer;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    TIM_DeInit(TIM2);
    timer.TIM_Period=arr;//   99  1000-1
    timer.TIM_Prescaler=psc;// 84M/72=1MHZ-->1us   84-1
    timer.TIM_ClockDivision=TIM_CKD_DIV1;
    timer.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2,&timer);
    TIM_Cmd(TIM2,ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
    NVIC_InitTypeDef  isr;
    isr.NVIC_IRQChannel=TIM2_IRQn;
    isr.NVIC_IRQChannelCmd=ENABLE;
    isr.NVIC_IRQChannelPreemptionPriority=3;
    isr.NVIC_IRQChannelSubPriority=0;
    NVIC_Init(&isr);   //
}

/*******************************************************************************
** ��������: TIM2_IRQHandler
** ��������: �жϺ�����ModBus��ʱ,1.5�ַ����
** ����˵��: : [����/��]
** ����˵��: None
** ������Ա: ����
** ��������: 2019-07-31
********************************************************************************/
void TIM2_IRQHandler()//��ʱ��2���жϷ����Ӻ���  1msһ���ж�
{
    u8 st;
    OSIntEnter();
    st= TIM_GetFlagStatus(TIM2, TIM_FLAG_Update);
    if(st==SET)
    {
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
        if(modbus.timrun!=0)
        {
            modbus.timout++;
            if(modbus.timout>=4)  //���ʱ��ﵽ��ʱ��
            {
                modbus.timrun=0;//�رն�ʱ��--ֹͣ��ʱ
                modbus.reflag=1;  //�յ�һ֡����
                OSSemPost(ModBusFlagSem);
            }
        }
    }
    OSIntExit();
}
/*******************************End of File************************************/


