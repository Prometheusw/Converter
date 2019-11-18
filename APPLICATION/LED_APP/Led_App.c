/********************************Copyright (c)**********************************\
**                   (c) Copyright 2019, Main, China
**                           All Rights Reserved
**                           By(��ʵ����ҽ�ƿƼ����޹�˾)
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: Led_App.c
** ������Ա: ����
** ��������: 2019-06-21
** �ĵ�����: LEDӦ�ò�����ָʾ�������к͹���״̬
*******************************************************************************/
#include "myconfig.h"
/*******************************************************************************
** ��������: RUN_Task
** ��������: ָʾϵͳ��������Led_Blink_Mode��������OStimedelay�������������
** ����˵��: pdata: [����/��] LED_NUM_0���ƺ����ţ�LED_MODE_3����˸ģʽ
** ����˵��: None
** ������Ա: ����
** ��������: 2019-06-13
********************************************************************************/
void RUN_Task(void *pdata)
{
  	pdata=pdata;
	  u8 useage;
	  u8 count=0;
    while(1)
    {
        GPIO_ResetBits(GPIOF,GPIO_Pin_9);
        OSTimeDlyHMSM(0,0,0,500);
//       UpdataNowLoction();
        GPIO_SetBits(GPIOF,GPIO_Pin_9);
        OSTimeDlyHMSM(0,0,0,500);
			
			 if(FramRecTimeFlag[0]++==10)
				 {
					 FramRecTimeFlag[0]=0;
					 LastFramFlag=0xff;
				 }
			  if(FramRecTimeFlag[1]++==10)
				 {
					 FramRecTimeFlag[1]=0;
					 alreadlyuptrack=0xff;
				 }
				 if(FramRecTimeFlag[2]++==10)
				 {
					 FramRecTimeFlag[2]=0;
					 alreadlydowntrack=0xff;
				 }
				 if(FramRecTimeFlag[3]++==15)
				 {
					 FramRecTimeFlag[3]=0;
           memset(LastPermissionFARM,0,sizeof(CAN_DATA_FRAME));
				 }
				 
				 
			  useage=my_mem_perused(SRAMIN);
			  count++;
			if(count==9)
				{
					count=0;
         // printf("LastFramFlag%d;TransStatus.TrackUse.TrackStatus&0x3c=%d;�ڴ�ʹ��=%d;\r\n",LastFramFlag,TransStatus.TrackUse.TrackStatus&0x3c,useage);
			  if(TransStatus.DeviceMode!=1||TransStatus.ErrorCode!=0||useage>1)
					/*����豸����ģʽ��Ϊ1���ߴ�����벻Ϊ�޴�������ڴ������ʲ�Ϊ1����ӡ*/
				  {
					
					 // printf("TransStatus.DeviceMode=%d;TransStatus.ErrorCode!=%d;�ڴ�ʹ��=%d;\r\n",
					//  TransStatus.DeviceMode,TransStatus.ErrorCode,useage);
						
				  }
				}
        Self_Function();//������
    }
}

/*******************************************************************************
** ��������: LED_Task
** ��������: LED_TIME�ṹ���е����ݷ����ı䣬�������ͻ�ִ�����е��õĺ������ò�ͬ�ĵ���һ��
** ����˵��: pdata: [����/��]
** ����˵��: None
** ������Ա: ����
** ��������: 2019-06-20
********************************************************************************/
void LED_Task(void *pdata)
{
    pdata=pdata;
    while(1)
    {
        Led_Blink_Times();
        Beep_Chirp();
        OSTimeDlyHMSM(0,0,0,10);
    }
}

/*******************************End of File************************************/

