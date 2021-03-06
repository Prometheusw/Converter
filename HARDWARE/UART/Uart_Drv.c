/********************************Copyright (c)**********************************\
**                   (c) Copyright 2019, Main, China
**                           All Rights Reserved
**                           By(达实久信医疗科技有限公司)
**----------------------------------文件信息------------------------------------
** 文件名称: Uart_Drv.c
** 创建人员: 王凯
** 创建日期: 2019-07-31
** 文档描述: 串口的驱动 3  4
********************************************************************************/
#include "myconfig.h"
/*******************************************************************************
** 函数名称: Uart_HardwareInit
** 功能描述: 串口初始化函数
** 参数说明: uartId: 串口号
**			 bps: 波特率
**			 parity: 校验方式
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-31
********************************************************************************/
void Uart_HardwareInit(u8 uartId,float bps,u16 parity)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
//USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bps;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = parity;//奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    switch (uartId)
    {
    case ID_UART3:
    {   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
        //串口3对应引脚复用映射
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
        //USART3端口配置
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
        GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB10，PB11}

        USART_Init(USART3, &USART_InitStructure); //初始化串口3
        USART_Cmd(USART3, DISABLE);  //使能串口3
        while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
        USART_ClearFlag(USART3, USART_FLAG_TC);

        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
        //USART_ITConfig(USART3, USART_IT_TC, ENABLE);//开启相关中断

        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;//抢占优先级9
        NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级5
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

    }
    break;

    case ID_UART4:
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART4时钟
        //串口4对应引脚复用映射
        GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为USART4
        GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为USART4
        //USART4端口配置
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10与GPIOC11
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
        GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC10，PC11
        USART_Init(UART4, &USART_InitStructure); //初始化串口4
        USART_Cmd(UART4, ENABLE);  //使能串口4

        while(USART_GetFlagStatus(UART4,USART_FLAG_TC)!=SET);
        USART_ClearFlag(UART4, USART_FLAG_TC);
        USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断
        /// USART_ITConfig(UART4, USART_IT_TC, ENABLE);//开启相关中断

        //USART_ClearFlag(UART4, USART_FLAG_TC);
        NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口4中断通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级2
        NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级2
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

    }
    break;
    default:
        break;
    }
}
/*******************************************************************************
** 函数名称: Uart_SendByte
** 功能描述: 串口发送一个字节函数
** 参数说明: uartId: 串口号
**			 byte: 字节
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-31
********************************************************************************/
void Uart_SendByte(uint8_t uartId,uint8_t byte)
{
    switch (uartId)
    {
    case ID_UART4:
        USART_SendData(UART4,byte);
        while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET);
        USART_ClearFlag(UART4,USART_FLAG_TC );
        break;
    case ID_UART3:
        USART_SendData(USART3,byte);
        while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
        USART_ClearFlag(USART3,USART_FLAG_TC );
        //g_uartSta[uartId].uartSendSta = UART_SENDING;
        break;
    default:
        break;
    }
}
/*******************************************************************************
** 函数名称: Uart_ReceiveByte
** 功能描述: 串口接收一个字节
** 参数说明: uartId: 串口号
** 返回说明: uint8_t，是收到的字节
** 创建人员: 王凯
** 创建日期: 2019-07-31
********************************************************************************/
uint8_t Uart_ReceiveByte(uint8_t uartId)
{
    static u8 ch = 0;
    switch (uartId)
    {
    case ID_UART4:
        ch = (UART4->DR);
        break;
    case ID_UART3:
        ch = (USART3->DR);
        break;
    default:
        ch = 0;
        break;
    }
    return ch;
}
/*******************************************************************************
** 函数名称: USART3_IRQHandler
** 功能描述: 串口中断函数
** 参数说明: None
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-31
********************************************************************************/
void USART3_IRQHandler(void)
{
    u8 sbuf;
    OSIntEnter();//让CPU掌握中断嵌套层数
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        Uart_ReceiveByte(ID_UART3);
        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
    }
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        sbuf=USART3->DR;
        if( modbus.reflag==1)  //有数据包正在处理
        {
            return;
        }
        modbus.rcbuf[modbus.recount++]=sbuf;
        modbus.timout=0;
        if(modbus.recount==1)  //收到主机发来的一帧数据的第一字节
        {
            modbus.timrun=1;  //启动定时
        }
    }
    OSIntExit();

}
/*******************************************************************************
** 函数名称: UART4_IRQHandler
** 功能描述: 串口4中断函数，此设计没有用到
** 参数说明: None
** 返回说明: None
** 创建人员: 王凯
** 创建日期: 2019-07-31
********************************************************************************/
void UART4_IRQHandler(void)
{
	    OSIntEnter();//让CPU掌握中断嵌套层数

    USART_ClearFlag(UART4, USART_FLAG_TC);
	    OSIntExit();

}
/*****************************End of Head************************************/

//int fputc(int ch, FILE *f)
//{
//	USART_SendData(USART3,(uint8_t)ch);
//	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET)
//		
//	{
//	}
//	return ch;
//}




