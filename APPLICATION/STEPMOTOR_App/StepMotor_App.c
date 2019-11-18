/********************************Copyright (c)**********************************\
**                   (c) Copyright 2019, Main, China
**                           All Rights Reserved
**                           By(��ʵ����ҽ�ƿƼ����޹�˾)
**----------------------------------�ļ���Ϣ------------------------------------
** �ļ�����: StepMotor_App.c
** ������Ա: ����
** ��������: 2019-06-21
** �ĵ�����: �������Ӧ������
******************************************************************************/
#include "myconfig.h"
void STEPMOTOR_Task(void *pdata)
{
    INT32S StepMotorCount;
    INT32U MotorCount;
    CAN_DATA_FRAME * tempNode;
    INT8U dirction;
    INT8U oserr;
    OS_CPU_SR cpu_sr;
    pdata=pdata;
    while(1)
    {
        tempNode = (CAN_DATA_FRAME *)OSMboxPend(StepRunMbox,0,&oserr);//�ȴ�����ת�����ź�����
        /*���������ͷ���*/
        StepMotorCount=((tempNode->canMsg.dataBuf[3])<<24)+((tempNode->canMsg.dataBuf[2])<<16)+
                       ((tempNode->canMsg.dataBuf[1])<<8)+(tempNode->canMsg.dataBuf[0]);
        dirction=(StepMotorCount&0x80000000)>>31;
        if(StepMotorCount<0)
            MotorCount=(~StepMotorCount+1);
        else
            MotorCount=StepMotorCount;
        if(ZeroDirction==anticlockwise)//�������ʱ��У��
        {
            dirction=1-dirction;//����Ϊ����ʱ��dir��0��1��˳ʱ�룬����Ϊ����ʱ��dir��1��0����ʱ��
        }
        else if(ZeroDirction==clockwise)//�����˳ʱ������
        {
            dirction=dirction;//����Ϊ��Ӧ�õ����ʱ��0��ת������Ϊ��Ӧ�õ��˳ָ����ת1
        }
        /*ִ�е���˶�����*/
        StepMotor_Run(100,dirction,MotorCount);
        OSSemPend(arrivePosSem,2000,&oserr);//�ȴ���λ�źţ��ڶ�ʱ��10�жϷ������з���10
        if(oserr==OS_ERR_TIMEOUT)//��ʱ//��Ԥ�ڵ�ʱ�䲻���ϣ�˵�����ܶ���������
        {
            //BeepAlwaysON=1;//������һֱ��
            // BeepChirptimes=BeepChirptimes+20;
            TransStatus.ErrorCode=MotorOverTime;//���������ʱ����
            StepMotor_Run(200,1-dirction,1000);//��ʱ������1000����
            OSSemPend(arrivePosSem,1000,&oserr);//�ȴ���λ�źţ�������ǳ�ʱ��˵���������ߣ��ǳ��Ӱ�ת��������
            if(oserr!=OS_ERR_NONE)////��Ȼ��ʱ
            {
                TransStatus.ErrorCode=BeStuckByCar;//ת������������
            }
            else//��������߳ɹ�
            {
                StepMotor_Run(200,dirction,1000);//��������
            }
            OSSemPend(arrivePosSem,0,&oserr);//�ȴ���λ�źţ��������޵ȴ�������Ȳ�����˵���������ع��ϣ��˹�����
            TransStatus.ErrorCode=No_Err;//����ȵ�˵���������Իָ�
            BeepAlwaysON=0;//�ص�һֱ��
            OSSemPost(stepFreeSem);//ִ�����е����
        }
        else
        {
            OSSemPost(stepFreeSem);//ִ�����е����
        }
//					  StepMotor_Run(200,1-dirction,500);//���ԣ������ƶ�500
//					  OSSemPend(arrivePosSem,0,&oserr);//�ȴ���λ�źţ��ڶ�ʱ��10�жϷ������з���
        //StepMotor_Run(200,dirction,500);//����500
        // OSSemPend(arrivePosSem,1000,&oserr);//�ȴ���λ�źţ��ڶ�ʱ��10�жϷ������з���
//            if(oserr==OS_ERR_NONE)//ͨ�����Ժ�����
//						{
//							GPIO_ResetBits(GPIOF,GPIO_Pin_8);//�ص�beep
//              FindZeroFuction();//���㹦�ܺ���
//						}
//						else if(oserr==OS_ERR_TIMEOUT)
//						{
//						 GPIO_SetBits(GPIOF,GPIO_Pin_8);
//						}
//
////            while(1)//�õ�������߳��������֪ͨ�ֶ�
////            {
////                CAN2_Single_Send(tempNode);
////                GPIO_SetBits(GPIOF,GPIO_Pin_8);
////                OSTimeDlyHMSM(0,0,0,1000);
////                GPIO_ResetBits(GPIOF,GPIO_Pin_8);
////                OSTimeDlyHMSM(0,0,0,1000);
////                __set_FAULTMASK(1);//�����жϹر�
////                NVIC_SystemReset();//������λ
////            }
//        }
        /*�˴��������жϣ���oserr�����ж�ִ�в�ͬ�ķ��ͺ���*/
        //CAN2_Single_Send(tempNode);//���õ�֡�ظ������ظ�һ֡����
        OS_ENTER_CRITICAL();
        myfree(SRAMIN,tempNode);//�ͷ����ź����䴫�����Ľ��
        OS_EXIT_CRITICAL();
    }
}

/*******************************************************************************
** ��������: TransLocation_TASK
** ��������: �����λ����,�����ֶ�ģʽ�µ���������ͳһʹ����Ϣ����
** ����˵��: pdata: [����/��]
** ����˵��: None
** ������Ա: ����
** ��������: 2019-06-28
********************************************************************************/
void TransLocation_TASK(void *pdata)
{
    CAN_DATA_FRAME * tempNode2;
    CAN_SEND_FRAME *TempResultFram;
    INT8U oserr;
    INT16U Num;
    OS_CPU_SR cpu_sr;
    pdata=pdata;
    while(1)
    {
        tempNode2=(CAN_DATA_FRAME *)OSQPend(TransLocationQeue,0,&oserr);//���ܶ�λ֡
        OS_ENTER_CRITICAL();
        TempResultFram=(CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));//���֡
        OS_EXIT_CRITICAL();
        memset(TempResultFram,0,sizeof(CAN_SEND_FRAME));//���뵽���ڴ�����
        /************************����Ƕ�λ��������λ******************************************/
        if(tempNode2->id.idBit.index==TransLocation)//��λ֡
        {
            Num=tempNode2->canMsg.dataBuf[0] | (tempNode2->canMsg.dataBuf[1]<<8);//������Ҫ��λ�Ķ�������
            //���ù���ı亯�����ҷ���ֵλ��ȷ��	neiyou�ȴ��������ִ���̷߳��ص��ź���
            if(Choose_TransLocate_Change(0,tempNode2->canMsg.dataBuf[0])==(u8)Num)
                // if(Choose_TransLocate_Change(tempNode2->canMsg.dataBuf[1],tempNode2->canMsg.dataBuf[0])==Num)

            {
                BeepChirptimes++;
                if(tempNode2->id.MasteridBit.deviceid==ThisTransitionNumber)//������ⲿ��λ ��modbus ��Ҫ�ظ�
                {
                    /*������ظ�����֡������*/
                    TempResultFram->len=1;
                    TempResultFram->id=(tempNode2->id.canId);//
                    TempResultFram->canMsg.dataBuf[0]=0x55;
                    TempResultFram->nextMsg=NULL;
                    CAN_Post_Queue(CAN2_CHANNEL,TempResultFram);//�뷢�Ͷ���
                }
                /*2019-07-05����:�޸ģ�������֡���Ӵ�����룬������������ݵĵ�һ���ֽ�*/
//                tempNode2->canMsg.dataBuf[0]=No_Err;//
//                CAN2_Single_Send(tempNode2);//���ͽ��֡
//							  OSSemPend(can2InfoAckSem,0,&sserr);//�ȴ����Ѿ����յ�����ɹ���Ϣ�Ļ�Ӧ�ź�
                OS_ENTER_CRITICAL();
                myfree(SRAMIN,TempResultFram);
                myfree(SRAMIN,tempNode2);
                OS_EXIT_CRITICAL();
                TransStatus.TrackUse.Usebit.ExeCommands=T_No;//����ִ������
            }
            else//����ͨ�ųɹ�������ִ��ʧ��
            {
                tempNode2->canMsg.dataBuf[0]=Comexefailed;
                tempNode2->dataLen=1;
                CAN2_Single_Send(tempNode2);//���ͽ��֡
                OS_ENTER_CRITICAL();
                myfree(SRAMIN,TempResultFram);
                myfree(SRAMIN,tempNode2);
                OS_EXIT_CRITICAL();
                TransStatus.TrackUse.Usebit.ExeCommands=T_No;//����ִ�������ͷ�
            }
        }

        /*****************************************��������*****************************************************************/
        else if(tempNode2->id.idBit.index==TransFindzero)//��������
        {
            FindZeroFuction(Need_Back);//���㺯��
            if(tempNode2->id.MasteridBit.deviceid==ThisTransitionNumber)
            {
                TempResultFram->len=1;
                TempResultFram->id=tempNode2->id.canId;//
                TempResultFram->canMsg.dataBuf[0]=No_Err;
                TempResultFram->nextMsg=NULL;
                CAN_Post_Queue(CAN2_CHANNEL,TempResultFram);//�뷢�Ͷ���
            }
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,TempResultFram);
            myfree(SRAMIN,tempNode2);
            OS_EXIT_CRITICAL();
            TransStatus.TrackUse.Usebit.ExeCommands=T_No;//����ִ�������ͷ�
        }
        /********************************************������Լ�����***********************************************/
        else if(tempNode2->id.MasteridBit.MainIndex==2&&tempNode2->id.MasteridBit.Subindex==ControlCheck)
        {
            u8 MovingErdoc=0;
            u8 StaticErdoc=0;
            u8 chooseErdoc;
            u8 twiceErdoc;
            FindZeroFuction(Need_Back);//������
            for(StaticErdoc=0; StaticErdoc<MAX_FixedTrack; StaticErdoc++) //����count
            {
                if(TrackCount[MovingErdoc][StaticErdoc]!=0xffff)//����0xffff������Ч�Ŀ��Ե����
                {
                    chooseErdoc=StaticErdoc;//
                }
            }
            if(TrackCount[0][0]!=0xffff)//����������Ч�ģ��Ͷ�λ�����
            {
                twiceErdoc=0;
            }
            else//������ܶ�λ���Ͷ�λ��һ��
            {
                twiceErdoc=1;
            }
            if(Choose_TransLocate_Change(0,chooseErdoc)==(chooseErdoc|(0x00<<8)))//����TransLocate������ţ�����ı亯�����ҷ���ֵλ��ȷ��	neiyou�ȴ��������ִ���̷߳��ص��ź���
            {
                {
                    if(Choose_TransLocate_Change(0,twiceErdoc)==(twiceErdoc|(0x00<<8)))
                        /*2019-07-05����:�޸ģ�������֡���Ӵ�����룬������������ݵĵ�һ���ֽ�*/
                        //memset(tempNode2->canMsg.dataBuf,0,sizeof(CAN_MSG));
                        if(tempNode2->id.MasteridBit.deviceid==ThisTransitionNumber)
                        {
                            TempResultFram->len=8;
                            TempResultFram->id=tempNode2->id.canId;//
                            TempResultFram->canMsg.dataBuf[0]=TransStatus.DeviceMode;//���ֽ��ǹ���ģʽ
                            TempResultFram->canMsg.dataBuf[1]=TransStatus.WarningCode;//�������
                            TempResultFram->canMsg.dataBuf[2]=TransStatus.ErrorCode;//�������
                            TempResultFram->canMsg.dataBuf[3]=TransStatus.DockedNumber;//��׼�Ĺ����//�ɲ�Ҫ
                            TempResultFram->canMsg.dataBuf[4]=TransStatus.TrackUse.TrackStatus;//�����
                            TempResultFram->canMsg.dataBuf[5]=(INT8U)TransStatus.EncoderCount;//�Ͱ�λ//��ǰ������
                            TempResultFram->canMsg.dataBuf[6]=(TransStatus.EncoderCount)>>8;//�߰�λ//��ǰ������
                            TempResultFram->canMsg.dataBuf[7]=TransStatus.TrackUse.TrackStatus;//��ǰλ��//01 12 23 11 22 33 ff
                            TempResultFram->nextMsg=NULL;
                            CAN_Post_Queue(CAN2_CHANNEL,TempResultFram);//�뷢�Ͷ���
                        }
                    OS_ENTER_CRITICAL();
                    myfree(SRAMIN,TempResultFram);
                    myfree(SRAMIN,tempNode2);
                    OS_EXIT_CRITICAL();

                    TransStatus.TrackUse.Usebit.ExeCommands=T_No;//����ִ�������ͷ�
                }
            }
        }
        /*********************************************�ƶ�����************************************************************/
        else if(tempNode2->id.MasteridBit.Subindex==TransMove&&tempNode2->id.MasteridBit.MasterslaveBit==MasterSlave)
        {
            if (tempNode2->canMsg.dataBuf[0]+(tempNode2->canMsg.dataBuf[1]<<8)==1)
            {

                if(ZeroDirction==anticlockwise)//�������ʱ��У��
                {
                    StepMotor_Run(200,1,30);//��ط����ٶȲ���̫С������û�е�λ�ź�
                }
                else
                {
                    StepMotor_Run(200,0,30);//��ط����ٶȲ���̫С������û�е�λ�ź�
                }

                OSSemPend(arrivePosSem,0,&oserr);
            }
            else if ((INT16S)(tempNode2->canMsg.dataBuf[0]+(tempNode2->canMsg.dataBuf[1]<<8))==-1)
            {

                if(ZeroDirction==anticlockwise)//�������ʱ��У��
                {
                    StepMotor_Run(200,0,30);//��ط����ٶȲ���̫С������û�е�λ�ź�
                }
                else
                {
                    StepMotor_Run(200,1,30);//��ط����ٶȲ���̫С������û�е�λ�ź�
                }


                OSSemPend(arrivePosSem,0,&oserr);
            }
            else
//�����������̵߳�ִ�У�������߳��лᷢ��һ������ͷ��ź����������������ź�������������Ӱ�������̵߳�ִ��
            {

                OSMboxPost(StepRunMbox,tempNode2);
                OSSemPend(stepFreeSem,0,&oserr);//�ȴ��������ִ���̷߳��ص��ź���
            }
            if(tempNode2->id.MasteridBit.deviceid==ThisTransitionNumber)
            {
                TempResultFram->len=1;
                TempResultFram->id=tempNode2->id.canId;//
                TempResultFram->canMsg.dataBuf[0]=No_Err;
                TempResultFram->nextMsg=NULL;
                CAN_Post_Queue(CAN2_CHANNEL,TempResultFram);//�뷢�Ͷ���
            }
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,TempResultFram);
            myfree(SRAMIN,tempNode2);
            OS_EXIT_CRITICAL();
        }
        else
        {
            tempNode2->canMsg.dataBuf[0]=Comexefailed;
            CAN2_Single_Send(tempNode2);//���ͽ��֡
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,tempNode2);
            OS_EXIT_CRITICAL();
            TransStatus.TrackUse.Usebit.ExeCommands=T_No;//����ִ�������ͷ�
        }
    }
}



/*******************************End of File************************************/



