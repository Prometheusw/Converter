/********************************Copyright (c)**********************************\
**                   (c) Copyright 2019, Main, China
**                           All Rights Reserved
**                           By(达实久信医疗科技有限公司)
**----------------------------------文件信息------------------------------------
** 文件名称: TrackMission_APP.c
** 创建人员: 王凯
** 创建日期: 2019-06-29
** 文档描述: 轨道任务
******************************************************************************/
#include "myconfig.h"
CAR_CHANGEMISSION_DATA * NOWMISSIONNODE;//当前任务节点s
/*筛选功能*/
CAR_CHANGEMISSION_DATA * SCREEN_FUNC(void)
{
    OS_CPU_SR cpu_sr;
    CAR_CHANGEMISSION_DATA *TempMissionNode=NULL;//临时结点，指向申请转轨任务头结点
    CAR_CHANGEMISSION_DATA *ReturnNode=NULL;//返回的预动筛选节点
    CAR_CHANGEMISSION_DATA *MissionNode=NULL;//返回会的节点备份
    CAR_CHANGEMISSION_DATA *LastNodeBackUp=NULL;//qianjiedian
    TempMissionNode = g_CarApplyChangedata;//指向头结点
    OS_ENTER_CRITICAL();
    while(TempMissionNode->NextMission!=NULL)
    {
        if(TransStatus.DeviceMode!=OperatingMode)//此处对模式进行判断
        {
            if(TempMissionNode->NextMission->Missiontype!=MISSION_AUTO)
            {
                MissionNode=TempMissionNode->NextMission;
                TempMissionNode->NextMission=TempMissionNode->NextMission->NextMission;
                OS_EXIT_CRITICAL();
                return MissionNode;
            }
        }
        else
        {
            if(TempMissionNode->NextMission->MissionMark==MISSION_CARCHAGE||
                    TempMissionNode->NextMission->MissionMark==MISSION_FINDZERO||
                    TempMissionNode->NextMission->MissionMark==MISSION_LOCATION||
                    TempMissionNode->NextMission->MissionMark==MISSION_MOVE)
                //找零和定位以及运动任务在这个筛选下应该都是不可用的，此处为了方便调试
            {
                MissionNode=TempMissionNode->NextMission;
                TempMissionNode->NextMission=TempMissionNode->NextMission->NextMission;
                OS_EXIT_CRITICAL();
                return MissionNode;
            }
            if(ISorNotPermisionDo==0&&ReturnNode==NULL&&TempMissionNode->NextMission->MissionMark==MISSION_PROMISSION)
            {
                LastNodeBackUp=TempMissionNode;
                ReturnNode=TempMissionNode->NextMission;
            }
        }
        TempMissionNode=TempMissionNode->NextMission;
    }
    if(ReturnNode!=NULL)
        LastNodeBackUp->NextMission=ReturnNode->NextMission;
    OS_EXIT_CRITICAL();
    return ReturnNode;
}
/*预动任务*/
ErrorType PreAction(u8 Num)
{
    u16 MIN_count=0xffff;//The Min Count;
    u8 chooseTrack;//选择的轨道号
    u16 D_Value;//差值
    u8 ErgodicTrack;//遍历轨道号
    chooseTrack=0xff;//为了防止所有的动轨都到不了这个定轨，一般不会出现这种情况，一般可能是标定的数据丢失
    MIN_count=0xffff;//最小编码筛选变量，设置为最大以便和其他的数进行比较
    /*选择最优的动轨编号*/
    for(ErgodicTrack=0; ErgodicTrack<MAX_MovingTrack; ErgodicTrack++)
    {
        if(TrackCount[ErgodicTrack][Num]!=0xffff)
        {
            D_Value=abs(TrackCount[ErgodicTrack][Num]-TransStatus.EncoderCount);
            if(D_Value<MIN_count)
            {
                MIN_count=D_Value;
                chooseTrack=ErgodicTrack;
            }
        }
    }
    if(chooseTrack==0xff)//如果选择不到动轨，说明目标轨道没有标定或者数据丢失或者域控制器数据没有更新
    {
        TransStatus.TrackUse.Usebit.ExeCommands=T_No;//命令执行完毕
        return TrackCountMISSS;
    }
    if(Choose_TransLocate_Change(0,Num)==Num)//调用转轨函数转换轨道
    {
        ISorNotPermisionDo=1;//是否预动标志设置为预动的轨道号
        TransStatus.TrackUse.Usebit.ExeCommands=T_No;//命令执行
        return No_Err;
    }
    return No_Err;
}
/*定位任务*/
ErrorType FixTrack()
{
    OS_CPU_SR cpu_sr;
    CAN_SEND_FRAME *TempResultFram;
    OS_ENTER_CRITICAL();
    TempResultFram=(CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));//结果帧
    OS_EXIT_CRITICAL();
    memset(TempResultFram,0,sizeof(CAN_SEND_FRAME));//申请到的内存清零
    Choose_TransLocate_Change(0,NOWMISSIONNODE->InitialPoint);
    BeepChirptimes++;
    if(NOWMISSIONNODE->FarmID!=0x04fe0210)//如果是外部定位 非modbus 需要回复
    {
        /*填充结果回复发送帧的数据*/
        TempResultFram->len=1;
        TempResultFram->id=NOWMISSIONNODE->FarmID;//
        TempResultFram->canMsg.dataBuf[0]=0x55;
        TempResultFram->nextMsg=NULL;
        CAN_Post_Queue(CAN2_CHANNEL,TempResultFram);//入发送队列
    }
    OS_ENTER_CRITICAL();
    myfree(SRAMIN,TempResultFram);
    OS_EXIT_CRITICAL();
    TransStatus.TrackUse.Usebit.ExeCommands=T_No;//正在执行命令
    return No_Err;
}
/*找零*/
ErrorType FindZeroAction()
{
    OS_CPU_SR cpu_sr;   
    FindZeroFuction(Need_Back);//找零函数
    if(NOWMISSIONNODE->FarmID!=0x04fe0211)//如果是外部定位 非modbus 需要回复
    {
			  CAN_SEND_FRAME *TempResultFram;
        OS_ENTER_CRITICAL();
        TempResultFram=(CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));//结果帧
        OS_EXIT_CRITICAL();
        memset(TempResultFram,0,sizeof(CAN_SEND_FRAME));//申请到的内存清零
        TempResultFram->len=1;
        TempResultFram->id=NOWMISSIONNODE->FarmID;//
        TempResultFram->canMsg.dataBuf[0]=No_Err;
        TempResultFram->nextMsg=NULL;
        CAN_Post_Queue(CAN2_CHANNEL,TempResultFram);//入发送队列
        OS_ENTER_CRITICAL();
        myfree(SRAMIN,TempResultFram);
        OS_EXIT_CRITICAL();
	  }
    TransStatus.TrackUse.Usebit.ExeCommands=T_No;//正在执行命令释放
    return No_Err;
}
/*自检*/
ErrorType CheakSelfAction()
{
    OS_CPU_SR cpu_sr;
    CAN_SEND_FRAME *TempResultFram;

    u8 MovingErdoc=0;
    u8 StaticErdoc=0;
    u8 chooseErdoc;
    u8 twiceErdoc;
    FindZeroFuction(Need_Back);//先找零
    for(StaticErdoc=0; StaticErdoc<MAX_FixedTrack; StaticErdoc++) //遍历count
    {
        if(TrackCount[MovingErdoc][StaticErdoc]!=0xffff)//不是0xffff就是有效的可以到达的
        {
            chooseErdoc=StaticErdoc;//
        }
    }
    if(TrackCount[0][0]!=0xffff)//如果零号是有效的，就定位到零号
    {
        twiceErdoc=0;
    }
    else//如果不能定位，就定位到一号
    {
        twiceErdoc=1;
    }
    if(Choose_TransLocate_Change(0,chooseErdoc)==(chooseErdoc|(0x00<<8)))//调用TransLocate（轨道号）轨道改变函数并且返回值位正确的	neiyou等待步进电机执行线程返回的信号量
    {
        {
            if(Choose_TransLocate_Change(0,twiceErdoc)==(twiceErdoc|(0x00<<8)))
                /*2019-07-05王凯:修改：给返回帧添加错误代码，错误代码是数据的第一个字节*/
                //memset(tempNode2->canMsg.dataBuf,0,sizeof(CAN_MSG));
                if(NOWMISSIONNODE->FarmID!=0x04fe0222)//如果是外部定位 非modbus 需要回复                {
                {
                    OS_ENTER_CRITICAL();
                    TempResultFram=(CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));//结果帧
                    OS_EXIT_CRITICAL();
                    memset(TempResultFram,0,sizeof(CAN_SEND_FRAME));//申请到的内存清零
                    TempResultFram->len=8;
                    TempResultFram->id=NOWMISSIONNODE->FarmID;//
                    TempResultFram->canMsg.dataBuf[0]=TransStatus.DeviceMode;//最字节是工作模式
                    TempResultFram->canMsg.dataBuf[1]=TransStatus.WarningCode;//警告代码
                    TempResultFram->canMsg.dataBuf[2]=TransStatus.ErrorCode;//错误代码
                    TempResultFram->canMsg.dataBuf[3]=TransStatus.DockedNumber;//对准的轨道号//可不要
                    TempResultFram->canMsg.dataBuf[4]=TransStatus.TrackUse.TrackStatus;//轨道锁
                    TempResultFram->canMsg.dataBuf[5]=(INT8U)TransStatus.EncoderCount;//低八位//当前编码数
                    TempResultFram->canMsg.dataBuf[6]=(TransStatus.EncoderCount)>>8;//高八位//当前编码数
                    TempResultFram->canMsg.dataBuf[7]=TransStatus.TrackUse.TrackStatus;//当前位置//01 12 23 11 22 33 ff
                    TempResultFram->nextMsg=NULL;
                    CAN_Post_Queue(CAN2_CHANNEL,TempResultFram);//入发送队列
                    OS_ENTER_CRITICAL();
                    myfree(SRAMIN,TempResultFram);
                    OS_EXIT_CRITICAL();
								}
            
            TransStatus.TrackUse.Usebit.ExeCommands=T_No;//正在执行命令释放
        }
    }
    return No_Err;
}
/*移动*/
ErrorType MoveAction()
{
    INT32S StepMotorCount;
    INT32U MotorCount;
    INT8U dirction;
    INT8U oserr;
    StepMotorCount=0x00000000+(NOWMISSIONNODE->TerminalPoint<<8)+(NOWMISSIONNODE->InitialPoint);
    if((NOWMISSIONNODE->TerminalPoint)>>7==0)
        StepMotorCount=StepMotorCount&0x0000ffff;
    else
        StepMotorCount=StepMotorCount|0xffff0000;
    dirction=(StepMotorCount&0x80000000)>>31;
    if(StepMotorCount<0)
        MotorCount=(~StepMotorCount+1);
    else
        MotorCount=StepMotorCount;
    if(ZeroDirction==anticlockwise)//如果是逆时针校零
    {
        dirction=1-dirction;//步数为正的时候，dir是0，1是顺时针，步数为负的时候，dir是1，0是逆时针
    }
    else if(ZeroDirction==clockwise)//如果是顺时针找零
    {
        dirction=dirction;//步数为正应该电机逆时针0旋转，步数为负应该电机顺指针旋转1
    }
    /*执行电机运动函数*/
    StepMotor_Run(100,dirction,MotorCount);
    OSSemPend(arrivePosSem,2000,&oserr);//等待到位信号，在定时器10中断服务函数中发送10
    if(oserr==OS_ERR_TIMEOUT)//超时//和预期的时间不符合，说明可能丢步，重试
    {
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
    }
    return No_Err;
}

/*小车调度*/
DISPATCHSTEP step=T_MOVE_POS0;
ErrorType DispatchCar()
{
    ErrorType err;
    OS_CPU_SR cpu_sr;
    uint8_t sserr;
    step=T_MOVE_POS0;
    CAN_SEND_FRAME *TempFramup=NULL;//通知小车上轨的主动帧结点
    CAN_SEND_FRAME *TempFramdown=NULL;//通知小车下轨的主动帧结点
    CAN_SEND_FRAME *CarIsDownTrack=NULL;//通知域控制器小车正在下轨道的帧节点，便于域控制器计算路径
    int retry=3;
    while(retry>0 && step!=T_FINISH)
    {

        switch(step)
        {
        case T_MOVE_POS0:
        {
            err=Choose_TransLocate_Change(0,NOWMISSIONNODE->InitialPoint);
            if((TrackCount[0][NOWMISSIONNODE->InitialPoint]==TrackCount[0][NOWMISSIONNODE->TerminalPoint])&&
                    (TrackCount[0][NOWMISSIONNODE->InitialPoint]!=0xffff)&&(TrackCount[0][NOWMISSIONNODE->TerminalPoint]!=0xffff))
            {
                step=T_NOTICE_OUT;
            }
            else
            {
                step=T_NOTICE_ENTRY;
            }
        }
        break;
        case T_NOTICE_ENTRY:
        {
            OS_ENTER_CRITICAL();
            TempFramup = (CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));//通知小车上轨的主动帧
            OS_EXIT_CRITICAL();
            memset(TempFramup,0,sizeof(CAN_SEND_FRAME));//因为并没有填满整个帧数据，全部设置为零，防止数据错乱
            TempFramup->len=2;
            TempFramup->id=NOWMISSIONNODE->CarNum<<16|ThisTransitionNumber<<8|CarCanUpTrack|0x14<<24;//索引号要改成通知小车可以上轨道的索引
            TempFramup->canMsg.dataBuf[0]=NOWMISSIONNODE->InitialPoint;//起始位
            TempFramup->canMsg.dataBuf[1]=NOWMISSIONNODE->TerminalPoint;//终点位置
            TempFramup->nextMsg=NULL;
            CAN_Post_Queue(CAN2_CHANNEL,TempFramup);//压入主动帧链表，发送通知小车上轨的帧，并在发送线程中等带对方的回应量
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,TempFramup);
            OS_EXIT_CRITICAL();
            step=T_WAIT_ENTRY;
        }
        break;
        case T_WAIT_ENTRY:
        {
            OSSemPend(CarAlreadyUpMbox,0,&sserr);//等待小车已经上轨信号
            step=T_MOVE_POS1;
        }
        break;
        case T_MOVE_POS1:
        {
            Choose_TransLocate_Change(0,NOWMISSIONNODE->TerminalPoint);
            step=T_NOTICE_OUT;
        }
        break;
        case T_NOTICE_OUT:
        {
            OS_ENTER_CRITICAL();
            TempFramdown = (CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));//通知小车x轨的主动帧
            OS_EXIT_CRITICAL();
            memset(TempFramdown,0,sizeof(CAN_SEND_FRAME));
            TempFramdown->len=0;
            TempFramdown->id=NOWMISSIONNODE->CarNum<<16|ThisTransitionNumber<<8|CarCanDownTrack|0x14<<24;//索引号要改成通知小车可以下轨道的索引
            //TempFramdown->canMsg.dataBuf[0]=TempMissionNode->InitialPoint;//起始位
            TempFramdown->nextMsg=NULL;
            CAN_Post_Queue(CAN2_CHANNEL,TempFramdown);//压入主动帧链表，发送通知小车下轨的帧，并在发送线程中等带对方的回应量
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,TempFramdown);
            OS_EXIT_CRITICAL();
            /*小车刚刚下轨道就给区域控制器发送小车已经下轨状态通知帧*/
            //组包
            OS_ENTER_CRITICAL();
            CarIsDownTrack = (CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));
            OS_EXIT_CRITICAL();
            memset(CarIsDownTrack,0,sizeof(CAN_SEND_FRAME));//帧清空
            CarIsDownTrack->len=1;//长度是1
            CarIsDownTrack->id=0x04<<24|ThisTransitionNumber<<16|CAN_TRANSFER_MAININDEX<<8|CarIsDowning;
            CarIsDownTrack->canMsg.dataBuf[0]=NOWMISSIONNODE->CarNum;
            CarIsDownTrack->nextMsg=NULL;
            //组包完成
            CAN_Post_Queue(CAN2_CHANNEL,CarIsDownTrack);//给区域控制器发
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,CarIsDownTrack);
            OS_EXIT_CRITICAL();
            step=T_WAIT_OUT;
        }
        break;
        case T_WAIT_OUT:
        {
            OSSemPend(CarAlreadyDownSem,15000,&sserr);//等待小车已经下轨信号量40S
            ISorNotPermisionDo=0;
            step=T_FINISH;
        }
        break;
        default:
            break;
        }
    }
		return No_Err;

}
/*执行函数*/
ErrorType IMPLEMENT()
{
    ErrorType err=No_Err;
    switch (NOWMISSIONNODE->MissionMark)
    {
    case MISSION_PROMISSION://预动作
        err=PreAction(NOWMISSIONNODE->InitialPoint);
        break;
    case MISSION_CARCHAGE://小车申请通过，真实任务调度小车
        err=DispatchCar();
        break;
    case MISSION_FINDZERO://找零
        err=FindZeroAction();
        break;
    case MISSION_CHEKSELF://自检查
        err=CheakSelfAction();
        break;
    case MISSION_LOCATION://定位
        err=FixTrack();
        break;
    case MISSION_MOVE://移动
        err=MoveAction();
        break;
    }
    return err;
}
/*释放函数*/
void RELEASENODE()
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    myfree(SRAMIN,NOWMISSIONNODE);
    OS_EXIT_CRITICAL();
}
/*筛选和执行任务*/
void SCREEN_IMPLEMENT_Task(void *pada)
{
    if(TransStatus.DeviceMode==OperatingMode)//暂时是自动模式/////////////////////////
        Load_One_FindZero();//在任务列表里面加一个找零任务，做W为上电找零
    while(1)
    {
        if((NOWMISSIONNODE=SCREEN_FUNC())==NULL)
        {
            OSTimeDlyHMSM(0,0,0,10);
            continue;
        }
        if(IMPLEMENT()==No_Err)
        {
            RELEASENODE();
        }
    }
}

/*******************************************************************************
** 函数名称: TransFindZero_Task
** 功能描述: 找零任务 定速100
** 参数说明: pdata: [输入/出]
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-03
********************************************************************************/
void TransFindZero_Task(void *pdata)
{
    pdata=pdata;
    while(1)
    {
        if(FindZeroGlob>0)//说明中断已经进入
        {
            OSTimeDlyHMSM(0,0,0,10);  //10ms的滤波
            if ((GPIOB->IDR & 0x02) == 0x00)
            {
                FindZeroCtrl=ISOnZero;//读取PB1的值，这个时候如果PB1是0，说明已经压到校零开关
                FindZeroGlob=0;
            }
            else if((GPIOB->IDR & 0x02) == 0x02)
            {
                FindZeroCtrl=ISAwayZero;//读取PB1的值，这个时候如果PB1是1，说明已经离开校零开关
                FindZeroGlob=0;
            }
        }
        else
            OSTimeDlyHMSM(0,0,0,5);  //让出CPU

    }
}
/*******************************End of File************************************/





