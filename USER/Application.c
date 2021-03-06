/********************************Copyright (c)**********************************\
**                   (c) Copyright 2019, Main, China
**                           All Rights Reserved
**                           By(达实久信医疗科技有限公司)
**----------------------------------文件信息------------------------------------
** 文件名称: Application.c
** 创建人员: 王凯
** 创建日期: 2019-08-09
** 文档描述:
*******************************End of Head************************************/
#include "myconfig.h"
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//中断分组配置
    ParameterInit();//参数配置
    HardwareInit();//硬件初始化
    //MOTOR_DIR_CTRL=1;
    OSInit();
    OSTaskCreate(START_Task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
    OSStart();
}
/*******************************************************************************
** 函数名称: start_task
** 功能描述: 开始任务 创建其他任务
** 参数说明: pdata: [输入/出]
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-04-29
********************************************************************************/
void START_Task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
    pdata = pdata;
    EventCreate();//创建信号量
    OS_ENTER_CRITICAL();//进入临界区(无法被中断打断)
    TaskCreate();
    OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.让出cpu
    OS_EXIT_CRITICAL();	//退出临界区(可以被中断打断)
}





