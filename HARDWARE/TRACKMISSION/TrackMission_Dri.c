/********************************Copyright (c)**********************************\
**                   (c) Copyright 2019, Main, China
**                           All Rights Reserved
**                           By(达实久信医疗科技有限公司)
**----------------------------------文件信息------------------------------------
** 文件名称: CarMission_Dri.c
** 创建人员: 王凯
** 创建日期: 2019-06-29
** 文档描述: 轨道任务所需要的底层函数
******************************************************************************/
#include "myconfig.h"
CAR_CHANGEMISSION_DATA *g_CarApplyChangedata;//小车申请转轨任务头结点
/*******************************************************************************
** 函数名称: CarMission_Software_Init
** 功能描述: 小车通过转轨器任务链表头结点初始化
** 参数说明: None
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-06-29
********************************************************************************/
void CarMission_Software_Init(void)
{
    //初始化申请转轨任务头结点
    g_CarApplyChangedata = (CAR_CHANGEMISSION_DATA*)mymalloc(SRAMIN,sizeof(CAR_CHANGEMISSION_DATA));
    if(g_CarApplyChangedata!=NULL)
    {
        g_CarApplyChangedata->NextMission=NULL;
    }
}
/*******************************************************************************
** 函数名称: Choose_TransLocate_Change
** 功能描述: 带选中的活轨号和要到的定轨号的轨道转换函数
** 参数说明: MovingTrack: 选中的活轨
**			     FixedTrack: 要到的定轨号
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-08-14
** 补充说明：当返回值是0xffff时候，说明当前活轨到达不了目标定轨
********************************************************************************/
u16 Choose_TransLocate_Change(u8 MovingTrack,u8 FixedTrack)
{
    OS_CPU_SR cpu_sr;
	  u8 dirction;
	  INT32U MotorCount;
//    u8 count;//计数
    INT8U oserr;
    INT32S InsertingValue;

    if(TrackCount[0][FixedTrack]==0xffff)//如果是ffff的话 说明这个定轨到不了这个动轨
    {
        return 0xffff;//返回错误信息
    }
    /**************************当所需要定位的轨道距离零点最近，校零****************************************/
		/*2019年10月21日17点01分修改：当轨道要到的位置是零点方向并且不是找零任务的时候，才进行逻辑判读*/
		/*逻辑判断：如果轨道是最近的轨道或者达到阈值，找零*/
    if(MovingTrack!=0xff&&FixedTrack<(u8)(TransStatus.DockedNumber))//当不是校零任务引起的轨道切换，才会执行下面的逻辑
    {
        if(FixedTrack==Find_NearestTrack()&&ThresholdValue>=MAXThresholdValue)//如果目标轨道号是最近的轨道，，或者阈值到达上限，这个时候强制校零
        {
            FindZeroFuction(Needt_Back);//找零函数
            ThresholdValue=0;//阈值变成0
        }
				
    }
    else//如果等于0xff，说明是校零的任务，这时候将值编程0；
    {
        MovingTrack=0;
    }
    InsertingValue=(INT32S)(TrackCount[MovingTrack][FixedTrack]-TransStatus.EncoderCount);//目标轨道号和当前编码差值（有符号数）
    if(InsertingValue==(INT32S)0)//如果当前的编码数和需要移动的轨道号编码数相等（差值为0）
    {
        TransStatus.DockedNumber=MovingTrack<<8|FixedTrack;
        return TransStatus.DockedNumber;//返回当前那一条活轨对准哪一条定轨
    }
    else if(InsertingValue!=(INT32S)0)//当前的轨道编码和需要移动的编码不相等
    {
        dirction=(InsertingValue&0x80000000)>>31;
        if(InsertingValue<0)
            MotorCount=(~InsertingValue+1);
        else
            MotorCount=InsertingValue;
        if(ZeroDirction==anticlockwise)//如果是逆时针校零
        {
            dirction=1-dirction;//步数为正的时候，dir是0，1是顺时针，步数为负的时候，dir是1，0是逆时针
        }
        else if(ZeroDirction==clockwise)//如果是顺时针找零
        {
            dirction=dirction;//步数为正应该电机逆时针0旋转，步数为负应该电机顺指针旋转1
        }
        /*执行电机运动函数*/
        StepMotor_Run(150,dirction,MotorCount);
        OSSemPend(arrivePosSem,2000,&oserr);//等待到位信号，在定时器10中断服务函数中发送10
        if(oserr==OS_ERR_TIMEOUT)//超时//和预期的时间不符合，说明可能丢步，重试
        {
            //BeepAlwaysON=1;//蜂鸣器一直响
            // BeepChirptimes=BeepChirptimes+20;
            TransStatus.ErrorCode=MotorOverTime;//电机卡死超时错误
            StepMotor_Run(200,1-dirction,1000);//这时候反向走1000脉冲
            OSSemPend(arrivePosSem,1000,&oserr);//等待到位信号，如果还是超时，说明反向不能走，是车子把转轨器卡死
            if(oserr!=OS_ERR_NONE)////任然超时
            {
                TransStatus.ErrorCode=BeStuckByCar;//转轨器被车卡主
            }
            else//如果反向走成功
            {
                StepMotor_Run(200,dirction,1000);//再正向走
            }
            OSSemPend(arrivePosSem,0,&oserr);//等待到位信号，这里无限等待，如果等不到，说明除了严重故障，人工处理
            TransStatus.ErrorCode=No_Err;//如果等到说明故障能自恢复
            BeepAlwaysON=0;//关掉一直响
        }
				
				
        if(EnableStopButton==T_Yes)
        {
            TransStatus.DockedNumber=0xff;//改变正好对准的轨道号(急停情况下未知位置)
            EnableStopButton=T_No;
        }
        else
        {
            TransStatus.DockedNumber=MovingTrack<<8|FixedTrack;//改变正好对准的轨道号
        }
        ThresholdValue++;//阈值++，说明转轨器执行了一次轨道切换任务

        TransStatus.TrackUse.Usebit.Monopolize=T_No;//独享位清零
        //myfree(SRAMIN,tempNodel);//此处不可以释放释放结点，在电机执行线程中把这个地址释放
        return TransStatus.DockedNumber;//返回当前那一条活轨对准哪一条定轨
    }

    return TransStatus.DockedNumber;//返回当前那一条活轨对准哪一条定轨

}
/*******************************************************************************
** 函数名称: Apply_Pass_Mission
** 功能描述: 轨道需要处理的机械任务入链表
** 参数说明: tempNode: 数据结点指针
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-06-29
改动说明：2019-7-5王凯：将找零任务也接到转轨器任务链表中来，找零分为手动与自动模式
防止在自动模式下不必要的校零引起事故。方便统一管理，CAR_CHANGEMISSION_DATA中增加找零任务标记
********************************************************************************/
void Apply_Change_Mission(CAN_DATA_FRAME * tempNode)
{
    OS_CPU_SR cpu_sr;
    CAR_CHANGEMISSION_DATA *NewMissionNode;//新任务数据结点
    CAR_CHANGEMISSION_DATA *TempMissionNode;//临时结点，指向申请转轨任务头结点
    OS_ENTER_CRITICAL();
    NewMissionNode = (CAR_CHANGEMISSION_DATA*)mymalloc(SRAMIN,sizeof(CAR_CHANGEMISSION_DATA));//申请内存
    OS_EXIT_CRITICAL();
    if(NewMissionNode!=NULL)//申请成功
    {
        memset(NewMissionNode,0,sizeof(CAR_CHANGEMISSION_DATA));
        /**如果是预动的任务，预动任务只能是域发送到转轨器，如果这时候轨道是锁定的，直接返回给域正在执行命令的错误代码*/
        if(tempNode->id.MasteridBit.Subindex==TranPerMission)//如果是预动任务
        {
            NewMissionNode->NextMission=NULL;
					  NewMissionNode->CarNum=tempNode->canMsg.dataBuf[1];
            NewMissionNode->InitialPoint=tempNode->canMsg.dataBuf[0];//获得小车的起始位置
            NewMissionNode->Missiontype=MISSION_AUTO;//预动标志置1
            NewMissionNode->MissionMark=MISSION_PROMISSION;//是预动任务
            NewMissionNode->TerminalPoint=tempNode->canMsg.dataBuf[0];//起始位和终点位置一样
            NewMissionNode->FarmID=tempNode->id.canId;
        }
        /*轨道变换*/
        else if(tempNode->id.idBit.index==CarApplyChange&&tempNode->id.idBit.MasterslaveBit==ManyMaster)//如果任务是轨道转换任务
        {
            NewMissionNode->NextMission=NULL;
            NewMissionNode->CarNum=tempNode->id.idBit.sendDeviceId;//获得小车的编号（设备号）
            NewMissionNode->InitialPoint=tempNode->canMsg.dataBuf[0];//获得小车的起始位置
            NewMissionNode->Missiontype=MISSION_AUTO;//预动标志清0
            NewMissionNode->MissionMark=MISSION_CARCHAGE;//是轨道切换任务
            NewMissionNode->TerminalPoint=tempNode->canMsg.dataBuf[1];//获得小车的终点位置
            NewMissionNode->FarmID=tempNode->id.canId;
        }

        /********************************************如果是找零帧******************************************************************/
        else if(tempNode->id.idBit.index==TransFindzero)//如果是找零任务
        {
            /*2019-7-5王凯增加：对于域发送的多主协议，无需获得域设备号，且找零任务只需要知道找零标记即可*/
            NewMissionNode->NextMission=NULL;
            if(tempNode->id.idBit.index==TransFindzero)
            {
                NewMissionNode->MissionMark=MISSION_FINDZERO;//是找零任务
            }

            NewMissionNode->Missiontype=MISSION_MANUAL;//预动标志0
            NewMissionNode->FarmID=tempNode->id.canId;
        }


        else if(tempNode->id.MasteridBit.Subindex==ControlCheck)//自检任务
        {
            NewMissionNode->MissionMark=MISSION_CHEKSELF;//是自检任务
            NewMissionNode->Missiontype=MISSION_MANUAL;//预动标志0
            NewMissionNode->FarmID=tempNode->id.canId;
        }


        /*如果是轨道定位任务//zhucong*/
        else if(tempNode->id.idBit.index==TransLocation&&tempNode->id.idBit.MasterslaveBit==MasterSlave)
        {
            NewMissionNode->NextMission=NULL;
            NewMissionNode->InitialPoint=tempNode->canMsg.dataBuf[0];//需要到的定轨号码
            NewMissionNode->TerminalPoint=tempNode->canMsg.dataBuf[1];//选中的活轨号码
            NewMissionNode->Missiontype=MISSION_MANUAL;//预动标志0
            NewMissionNode->MissionMark=MISSION_LOCATION;//是轨道定位任务
            NewMissionNode->FarmID=tempNode->id.canId;
        }
        /**轨道移动任务*/
        else if(tempNode->id.idBit.index==TransMove&&tempNode->id.idBit.MasterslaveBit==MasterSlave)
        {
            NewMissionNode->NextMission=NULL;
            NewMissionNode->InitialPoint=tempNode->canMsg.dataBuf[0];//高低
            NewMissionNode->TerminalPoint=tempNode->canMsg.dataBuf[1];
            NewMissionNode->Missiontype=MISSION_MANUAL;//预动标志0
            NewMissionNode->MissionMark=MISSION_MOVE;//是轨道移动任务
            NewMissionNode->FarmID=tempNode->id.canId;
        }

        /*2019-07-05王凯：增加：当任务节点不是合法的索引，将申请的内存释放后，返回*/
        else
        {
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,NewMissionNode);
            OS_EXIT_CRITICAL();
            return;
        }
        NewMissionNode->TimeMark=OSTimeGet();//获得时间标志
        TempMissionNode=g_CarApplyChangedata;//临时结点指向头结点
				OS_ENTER_CRITICAL();
        while(TempMissionNode->NextMission!=NULL)//把新的结点接在链表的最后
        {
            TempMissionNode=TempMissionNode->NextMission;
        }
        TempMissionNode->NextMission=NewMissionNode;
        NewMissionNode->NextMission=NULL;
				//printf("任务已经入队列表\r\n");
				OS_EXIT_CRITICAL();
    }
    else
    {
        OS_ENTER_CRITICAL();
        myfree(SRAMIN,NewMissionNode);
        OS_EXIT_CRITICAL();

    }
}
/*******************************************************************************
** 函数名称: TrackCount_Init
** 功能描述:从W25Q中装载转轨器标定参数
** 参数说明: None
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-03
********************************************************************************/
bool TrackCount_Load(void)
{
    INT8U W25QCount=0;
    INT8U MovingCount=0;

    W25QXX_Read(W25QXXReceiveBuff,0,sizeof(W25QXXReceiveBuff));
//	   for(W25QCount=0; W25QCount<40; W25QCount++)
//	{
//		if(W25QXXReceiveBuff[W25QCount]==0xff)
//	   W25QXXReceiveBuff[W25QCount]=0;
//	}
//    W25QCount=0;
//在主区域和备份区域校验之后，从主数据区域开始,读砏25QXXReceiveBuff个字节
    for(MovingCount=0; MovingCount<MAX_MovingTrack; MovingCount++) //最大动轨，有几就有几组数据
    {
        for(W25QCount=0; W25QCount<MAX_FixedTrack; W25QCount++)
        {
            TrackCount[MovingCount][W25QCount]=W25QXXReceiveBuff[32*MovingCount+W25QCount*2]+
                                               (W25QXXReceiveBuff[32*MovingCount+W25QCount*2+1]<<8);
        }
    }

//		t=crc16(W25QXXReceiveBuff,sizeof(TrackCount));
//      if((W25QXXReceiveBuff[32]<<8)+W25QXXReceiveBuff[33]==t)
//			{
    return true;
//			}
//			else
//				return false;
}

/*******************************************************************************
** 函数名称: TrackCount_Save
** 功能描述: 将转轨器的轨道编码数保存到flash中
** 参数说明: None
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-06
********************************************************************************/
void TrackCount_Save(void)
{
    INT8U W25QCount=0;
    INT8U MovingCount=0;

    memset(W25QXXSendBuff,0,sizeof(W25QXXSendBuff));

    for(MovingCount=0; MovingCount<MAX_MovingTrack; MovingCount++) //最大动轨，有几就有几组数据
    {
        for(W25QCount=0; W25QCount<MAX_FixedTrack; W25QCount++)//最大定轨，有几个每组就有多少个数据
        {
            W25QXXSendBuff[32*MovingCount+W25QCount*2]=TrackCount[MovingCount][W25QCount];//低八位给第一个字节
            W25QXXSendBuff[32*MovingCount+W25QCount*2+1]=TrackCount[MovingCount][W25QCount]>>8;//高八位给第二个字节
        }
    }
    W25QXXSendBuff[128]=(crc16(W25QXXSendBuff,sizeof(TrackCount)))>>8;//crc校验码高八位
    W25QXXSendBuff[129]=crc16(W25QXXSendBuff,sizeof(TrackCount));//CRC校验码低八位

    W25QXX_Write((INT8U*)W25QXXSendBuff,0,sizeof(TrackCount)+2);//将编码数备份到主区域//主区域4k大小，一个扇区
    W25QXX_Write((INT8U*)W25QXXSendBuff,FLASH_SECTOR_SIZE,sizeof(TrackCount)+2);//将编码书备份到备份区//4k，一个扇区
}

/*******************************************************************************
** 函数名称: FindZeroFuction
** 功能描述: 找零功能封装
** 参数说明: None
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-18
********************************************************************************/
void FindZeroFuction(u8 mode)
{
    INT8U oserr;
	  TransStatus.DockedNumber=0xffff;
    if ((GPIOB->IDR & 0x02) == 0x00)//如果一上电就在零点开关上，反向走2000个脉冲
    {
        if(ZeroDirction==clockwise)
        {
            StepMotor_Run(100,PCtrl,1500);//z转//函数中有PPEND，会让出调度，不会引起阻塞
        }
        else
        {
            StepMotor_Run(100,NCtrl,1500);////函数中有PPEND，会让出调度，不会引起阻塞//顺时针旋转
        }
        OSSemPend(arrivePosSem,300,&oserr);
    }
    TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,ENABLE);
    while(FindZeroCtrl!=ISOnZero)//如果说零点开关没有被压住，就一直靠近零点开关走
    {
        if(ZeroDirction==clockwise)//如果是顺时针校零
        {
            StepMotor_NCtrl(100);////函数中有PPEND，会让出调度，不会引起阻塞//顺时针旋转
        }
        else
        {
            StepMotor_PCtrl(100);//z转//函数中有PPEND，会让出调度，不会引起阻塞
        }

        /*判断刹车信号是否有效*/
        if(TranferSpeed==TranferStop)//停止标志位0 立刻停止
        {
            TranferSpeed=1;//停止
            EnableStopButton=T_Yes;//是否急停按键按下
            TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,DISABLE);
            break;//跳出执行，停止
        }
//        /*判断找零开关是否有效*/
//        if(TransStatus.ErrorCode==MotorOverCurrent)//如果这时候发生电机过流现象,说明卡在最后了，说明教令开关失效
//        {
//            TransStatus.ErrorCode=CantFindZero;//错误类型细分为找零失效
//            TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,DISABLE);
//            return;
//        }
    }
    // TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,DISABLE);

    OSSemPend(arrivePosSem,300,&oserr);
//TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,ENABLE);
    if(EnableStopButton!=T_Yes)
    {
        while(FindZeroCtrl!=ISAwayZero)//如果此时教令开关没有松开，一直远离零点走
        {
            if(ZeroDirction==clockwise)
            {
                StepMotor_PCtrl(150);//z转//函数中有PPEND，会让出调度，不会引起阻塞
            }
            else
            {
                StepMotor_NCtrl(150);////函数中有PPEND，会让出调度，不会引起阻塞//顺时针旋转
            }
            /*判断是否存在急停按键按下的情况*/
            if(TranferSpeed==TranferStop)//停止标志位0 立刻停止
            {
                TranferSpeed=1;//停止
                EnableStopButton=T_Yes;//是否急停按键按下
                TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,DISABLE);

                break;//跳出执行，停止
            }
            //TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,ENABLE);
            /*判断找零开关是否有效*/
//            if(TransStatus.ErrorCode==MotorOverCurrent)//如果这时候发生电机过流现象，说明教令开关失效
//            {
//                TransStatus.ErrorCode=CantFindZero;//错误类型细分为找零失效
//                TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,DISABLE);
//                return;
//            }
        }
        TIM_ITConfig(TIM3,TIM_IT_CC3|TIM_IT_CC4,DISABLE);
        OSSemPend(arrivePosSem,300,&oserr);
    }
    FindZeroGlob=0;
    BeepChirptimes++;//交一次
    if(EnableStopButton==T_Yes)
    {
        TransStatus.DockedNumber=0xffff;//改变正好对准的轨道号(急停情况下未知位置)
        TransStatus.TrackUse.TrackStatus|=0x01;//有bug
        TransStatus.EncoderCount=0;
    }
    else
    {
        TransStatus.DockedNumber=0xfffe;//-1
        TransStatus.EncoderCount=0;
        TransStatus.ErrorCode=0;
        TransStatus.TrackUse.TrackStatus|=0x01;//有bug
        TransStatus.WarningCode=0;
    }
    if(mode==Need_Back&&EnableStopButton!=T_Yes)
    {
        Choose_TransLocate_Change(0xff,Find_NearestTrack());//find the nearest track.
    }
    else
    {
			EnableStopButton=T_No;
        return;
    }		        
}
/*******************************************************************************
** 函数名称: TrackLockFunction
** 功能描述: 轨道锁操作函数，锁定或者解锁轨道号
** 参数说明: lockorun: 锁定还是解锁
**			 trackNumber: 轨道编码号，直接写编码数
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-18
********************************************************************************/
u8 TrackLockFunction(u8 lockorun,u8 trackNumber)
{
    if(lockorun == LockTrack)
        TransStatus.TrackUse.TrackStatus=((TransStatus.TrackUse.TrackStatus)|(0x04<<(trackNumber/2)));//锁定轨道置1
    else if(lockorun == UnlockTrack)
        TransStatus.TrackUse.TrackStatus=((TransStatus.TrackUse.TrackStatus)&(~(0x04<<(trackNumber/2))));//解锁轨道清0
    else return 0;

    return TransStatus.TrackUse.TrackStatus;

}
/*******************************End of File************************************/

