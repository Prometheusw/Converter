#include "myconfig.h"
#define WAIT_CANMBOX_MAXTIME   50     //�ȴ���������ź�����ʱʱ����Ϊ50ms
#define CAN_SEND_MAXTIME       500
#define SYNC_DATA_MAXTIME      50     //ͬ�������ʱ��Ϊ5s
extern OS_EVENT * can1Mbox;

u8 CAN_threshold;




CAN_DATA_FRAME *g_canDataFrame[MAX_CAN_CHANNEL];
CAN_DATA_FRAME *g_canDataAckFrame[MAX_CAN_CHANNEL];  //Ӧ������
CAN_DATA_FRAME *g_canDataActFrame[MAX_CAN_CHANNEL];  //��������

CAN_SEND_FRAME *g_can1SendFrame;  //CAN1��������֡����
CAN_SEND_FRAME *g_can2SendFrame;  //CAN2��������֡����
ReplyFrame can1CtlAckFrame,can1CtlResFrame,can1InfoAckFrame,can2InfoAckFrame;
/*typedef struct _CAN_BPR_INIT
{
	float bps;
	u8 tsjw;
	u8 tbs1;
	u8 tbs2;
	u16 brp;
}CAN_BPR_INIT;*/
CAN_BPR_INIT CANBprInit[MAX_BPS_SIZE] =
{
    {CAN_BPS_5K,1,CAN_BS1_13tq,CAN_BS2_2tq,525},
    {CAN_BPS_10K,1,CAN_BS1_6tq,CAN_BS2_1tq,525},
    {CAN_BPS_20K,1,CAN_BS1_5tq,CAN_BS2_1tq,300},
	  {CAN_BPS_333K,1,CAN_BS1_10tq,CAN_BS2_2tq,97},
    {CAN_BPS_50K,1,CAN_BS1_6tq,CAN_BS2_1tq,105},
    {CAN_BPS_125K,1,CAN_BS1_6tq,CAN_BS2_1tq,42},
    {CAN_BPS_250K,1,CAN_BS1_6tq,CAN_BS2_1tq,21},
    {CAN_BPS_500K,1,CAN_BS1_5tq,CAN_BS2_1tq,12},
    {CAN_BPS_1000K,1,CAN_BS1_15tq,CAN_BS2_5tq,2},
    {CAN_BPS_25K,1,CAN_BS1_6tq,CAN_BS2_1tq,210},
    {CAN_BPS_30K,1,CAN_BS1_6tq,CAN_BS2_1tq,175},
    {CAN_BPS_800K,1,CAN_BS1_2tq,CAN_BS2_1tq,13},
    {CAN_BPS_833K,1,CAN_BS1_6tq,CAN_BS2_1tq,63},
    {CAN_BPS_34K,1,CAN_BS1_3tq,CAN_BS2_1tq,247},//zijia  34

};
/****************************************************************
���ܣ�CANӲ����ʼ������
��ڲ�����u8 brp��������
����ֵ����
*****************************************************************/
void CAN_Hardware_Init(u8 brp)
{
    if(brp==0)
    {
        CAN_HardwareInit(CAN1_CHANNEL,CAN_BPS_50K,CAN_Mode_Normal,CAN1_RX0_INT_ENABLE);
        CAN_HardwareInit(CAN2_CHANNEL,CAN_BPS_50K,CAN_Mode_Normal,CAN2_RX0_INT_ENABLE);
    }
    else if(brp==1)
    {
        CAN_HardwareInit(CAN1_CHANNEL,CAN_BPS_20K,CAN_Mode_Normal,CAN1_RX0_INT_ENABLE);
        CAN_HardwareInit(CAN2_CHANNEL,CAN_BPS_20K,CAN_Mode_Normal,CAN2_RX0_INT_ENABLE);
    }
    else if(brp==2)
    {
        CAN_HardwareInit(CAN1_CHANNEL,CAN_BPS_10K,CAN_Mode_Normal,CAN1_RX0_INT_ENABLE);
        CAN_HardwareInit(CAN2_CHANNEL,CAN_BPS_10K,CAN_Mode_Normal,CAN2_RX0_INT_ENABLE);
    }
    else if(brp==3)
    {
        CAN_HardwareInit(CAN1_CHANNEL,CAN_BPS_5K,CAN_Mode_Normal,CAN1_RX0_INT_ENABLE);
        CAN_HardwareInit(CAN2_CHANNEL,CAN_BPS_5K,CAN_Mode_Normal,CAN2_RX0_INT_ENABLE);
    }
    else if(brp==4)
    {
        CAN_HardwareInit(CAN1_CHANNEL,CAN_BPS_125K,CAN_Mode_Normal,CAN1_RX0_INT_ENABLE);
        CAN_HardwareInit(CAN2_CHANNEL,CAN_BPS_125K,CAN_Mode_Normal,CAN2_RX0_INT_ENABLE);
    }

}

u8 CAN_Mode_Init(CAN_TypeDef *canChan,CAN_InitTypeDef *CAN_InitStructure,CAN_FilterInitTypeDef *CAN_FilterInitStructure,u8 rxIntType)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    CAN_Init(canChan, CAN_InitStructure);   // ��ʼ��CAN1
    CAN_FilterInit(CAN1,CAN_FilterInitStructure);//�˲�����ʼ��
    if (CAN1_RX0_INT_ENABLE	& rxIntType)
    {
        CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.
        NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;     // �����ȼ�Ϊ6
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

    }

    if (CAN1_RX1_INT_ENABLE	& rxIntType)
    {
        CAN_ITConfig(CAN1,CAN_IT_FMP1,ENABLE);//FIFO1��Ϣ�Һ��ж�����.
        NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;     // �����ȼ�Ϊ8
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
    if (CAN2_RX0_INT_ENABLE	& rxIntType)
    {
        CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.
        NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;     // �����ȼ�Ϊ7
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
    if (CAN2_RX1_INT_ENABLE	& rxIntType)
    {
        CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO1��Ϣ�Һ��ж�����.
        NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;     // �����ȼ�Ϊ9
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    }
    return 0;
}
/****************************************************************
���ܣ�CANӲ����ʼ������
��ڲ�����u8 canChan:CANͨ��,float bps��������,u8 mode������ģʽ,u8 rxIntType
����ֵ����
*****************************************************************/
void CAN_HardwareInit(u8 canChan,float bps,u8 mode,u8 rxIntType)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef  CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    u8 i = 0;
    switch (canChan)
    {
    case CAN1_CHANNEL:
#if CAN1_USE_GPIOA_EN
        //ʹ�����ʱ��
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
        //��ʼ��GPIO
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
        GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
        //���Ÿ���ӳ������
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
#elif CAN1_USE_GPIOB_EN
        //ʹ�����ʱ��
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTBʱ��
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
        //��ʼ��GPIO
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
        GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PB8,PB9
        //���Ÿ���ӳ������
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOB8����ΪCAN1
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOB9����ΪCAN1
#elif CAN1_USE_GPIOD_EN
        //ʹ�����ʱ��
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��PORTDʱ��
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
        //��ʼ��GPIO
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
        GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��PD0,PD1
        //���Ÿ���ӳ������
        GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); //GPIOD0����ΪCAN1
        GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); //GPIOD1����ΪCAN1
#endif
        for (i = 0; i < MAX_BPS_SIZE; i++)
        {
            if (CANBprInit[i].bps == bps)
            {
                CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ
                CAN_InitStructure.CAN_ABOM=ENABLE;	//�����Զ����߹���
                CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
                CAN_InitStructure.CAN_NART=DISABLE;	//��ֹ�����Զ�����
                CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�
                CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������
                CAN_InitStructure.CAN_Mode = mode;	 //ģʽ����
                CAN_InitStructure.CAN_SJW = CANBprInit[i].tsjw;	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
                CAN_InitStructure.CAN_BS1 = CANBprInit[i].tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
                CAN_InitStructure.CAN_BS2 = CANBprInit[i].tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
                CAN_InitStructure.CAN_Prescaler = CANBprInit[i].brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1
                i = MAX_BPS_SIZE;
            }
        }
        //���ù�����
        if (rxIntType & CAN1_RX0_INT_ENABLE)
        {
            CAN_FilterInitStructure.CAN_FilterNumber = 0;	  //������0
            CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
            CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32λ
            CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;////32λID
            CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
            CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32λMASK
            CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
            CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//������0������FIFO0
            CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //���������0
            CAN_Mode_Init(CAN1,&CAN_InitStructure,&CAN_FilterInitStructure,rxIntType);
        }
        if (rxIntType & CAN1_RX1_INT_ENABLE)
        {
            CAN_FilterInitStructure.CAN_FilterNumber = 1;	  //������0
            CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
            CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32λ
            CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;////32λID
            CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
            CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32λMASK
            CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
            CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;//������0������FIFO0
            CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //���������0
            CAN_Mode_Init(CAN1,&CAN_InitStructure,&CAN_FilterInitStructure,rxIntType);
        }
        break;
    case CAN2_CHANNEL:
        //ʹ�����ʱ��
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTBʱ��
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN2ʱ��
        //��ʼ��GPIO
#if CAN2_USE_GPIOB5_6_EN
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
#elif CAN2_USE_GPIOB12_13_EN
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
#endif
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
        GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PB12,PB13��PB5��PB6
        //���Ÿ���ӳ������
#if CAN2_USE_GPIOB5_6_EN
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOB5����ΪCAN2
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOB6����ΪCAN2
#elif CAN2_USE_GPIOB12_13_EN
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOB12����ΪCAN2
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOB13����ΪCAN2
#endif
        for (i = 0; i < MAX_BPS_SIZE; i++)
        {
            if (CANBprInit[i].bps == bps)
            {
                CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ
                CAN_InitStructure.CAN_ABOM=ENABLE;	//�����Զ����߹���
                CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
                CAN_InitStructure.CAN_NART=DISABLE;	//��ֹ�����Զ�����
                CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�
                CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������
                CAN_InitStructure.CAN_Mode = mode;	 //ģʽ����
                CAN_InitStructure.CAN_SJW = CANBprInit[i].tsjw;	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
                CAN_InitStructure.CAN_BS1 = CANBprInit[i].tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
                CAN_InitStructure.CAN_BS2 = CANBprInit[i].tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
                CAN_InitStructure.CAN_Prescaler = CANBprInit[i].brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1
                i = MAX_BPS_SIZE;
            }
        }
        if (rxIntType & CAN2_RX0_INT_ENABLE)
        {
            CAN_FilterInitStructure.CAN_FilterNumber = 14;	  //������2
            CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
            CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32λ
            CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;////32λID
            CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
            CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32λMASK
            CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
            CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//������2������FIFO0
            CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //���������1
            CAN_Mode_Init(CAN2,&CAN_InitStructure,&CAN_FilterInitStructure,rxIntType);
        }
        if (rxIntType & CAN2_RX1_INT_ENABLE)
        {
            CAN_FilterInitStructure.CAN_FilterNumber = 3;	  //������2
            CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
            CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32λ
            CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;////32λID
            CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
            CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32λMASK
            CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
            CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO1;//������2������FIFO0
            CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //���������1
            CAN_Mode_Init(CAN2,&CAN_InitStructure,&CAN_FilterInitStructure,rxIntType);
        }
        break;
    default:
        break;
    }
}

/*******************************************************************************
** ��������: CANSEND_Init
** ��������: can�����ж�����
** ����˵��: None
** ����˵��: None
** ������Ա: ����
** ��������: 2019-04-29
********************************************************************************/
void CANSEND_Init(void)
{
    NVIC_InitTypeDef   NVIC_InitStructure;

    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);//����������ж�
    NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /*****************/
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);//����������ж�
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/****************************************************************
���ܣ�CANͨ���������������ú���
��ڲ�����u8 canChan��CANͨ��
����ֵ����;
*****************************************************************/
void CAN_FMR_Config(u8 canChan)
{
    CAN1->FMR |= FMR_INIT;  //�����������ڳ�ʼ��ģʽ
    if(canChan==CAN1_CHANNEL)
    {
        CAN1->FMR &= ~0xFF00;   //��������λ���������
        CAN1->FMR |=  0x1A00;   //ΪCAN1�ڷ���27���˲�����CAN2�ڷ���1���˲���
    }
    else if(canChan==CAN2_CHANNEL)
    {
        CAN1->FMR &= ~0xFF00;   //��������λ���������
        CAN1->FMR |= 0x0100;   //ΪCAN1�ڷ���1���˲�����CAN2�ڷ���27���˲���
    }
    CAN1->FMR &= ~FMR_INIT; //��������Ч
}


/****************************************************************
���ܣ�CAN������ʼ������    wangkai:���Ǵ�����ʼ������������ͷ���
��ڲ�������
����ֵ����;
*****************************************************************/
void CAN_Software_Init()
{
    //��ʼ��can1������������ͷ���
    g_canDataFrame[CAN1_CHANNEL] = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
    if (g_canDataFrame[CAN1_CHANNEL]!= NULL)
    {
        g_canDataFrame[CAN1_CHANNEL]->nextMsg = NULL;
    }
    //��ʼ��can2������������ͷ���
    g_canDataFrame[CAN2_CHANNEL] = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
    if (g_canDataFrame[CAN2_CHANNEL]!= NULL)
    {
        g_canDataFrame[CAN2_CHANNEL]->nextMsg = NULL;
    }
    //��ʼ��can1��ҪӦ�����������ͷ���
    g_canDataAckFrame[CAN1_CHANNEL] = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
    if (g_canDataAckFrame[CAN1_CHANNEL]!= NULL)
    {
        g_canDataAckFrame[CAN1_CHANNEL]->nextMsg = NULL;
    }
    //��ʼ��can2��ҪӦ�����������ͷ���
    g_canDataAckFrame[CAN2_CHANNEL] = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
    if (g_canDataAckFrame[CAN2_CHANNEL]!= NULL)
    {
        g_canDataAckFrame[CAN2_CHANNEL]->nextMsg = NULL;
    }

    //��ʼ��can1����ҪӦ�����������ͷ���
    g_canDataActFrame[CAN1_CHANNEL] = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
    if (g_canDataActFrame[CAN1_CHANNEL]!= NULL)
    {
        g_canDataActFrame[CAN1_CHANNEL]->nextMsg = NULL;
    }
    //��ʼ��can2����ҪӦ�����������ͷ���
    g_canDataActFrame[CAN2_CHANNEL] = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
    if (g_canDataActFrame[CAN2_CHANNEL]!= NULL)
    {
        g_canDataActFrame[CAN2_CHANNEL]->nextMsg = NULL;
    }

    //CAN1����֡����������ʼ��
    g_can1SendFrame = (CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));
    if (g_can1SendFrame!= NULL)
    {
        g_can1SendFrame->nextMsg = NULL;
    }
    //CAN2����֡����������ʼ��
    g_can2SendFrame = (CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));
    if (g_can2SendFrame!= NULL)
    {
        g_can2SendFrame->nextMsg = NULL;
    }
}



/****************************************************************
���ܣ�CAN_id��̬���Ӻ���
��ڲ�����u8 *id��id�б�,u8 *cnt:id���и���,temp:׼���Ƚϼ������ID
����ֵ����;
*****************************************************************/
void CAN_Update_ID(u8 *id,u8 *cnt,u8 temp)
{
    u8 i=0;
    if((*cnt)==0)
    {
        id[*cnt] = temp;
        (*cnt)++;
    }
    else
    {
        while(i<*cnt)
        {
            if(temp==id[i])
                break;
            else
                i++;
        }
        if(i==(*cnt))  //Ϊ��ID,��Ҫ���б��и���
        {
            //��ֹ��ַԽ�紦��
            if((*cnt)<ID_LIST_MAXSIZE)
            {
                id[*cnt] = temp;
                (*cnt)++;
            }
        }
    }
}

/****************************************************************
���ܣ�CAN���ݽ����ӿں���
��ڲ�����CAN_DATA_FRAME *frameBuf������������ݻ�����ָ��,CanRxMsg *msg����Ϣ������ָ��
����ֵ�����յ����ݳ���
wangkai�����ǽ���Ӧ��*msgָ������Ϣ���Ƶ�*framebuf�ṹ���У�framebufӦ������������Ľ��
*****************************************************************/
u8 CAN_DataAnalyze(CAN_DATA_FRAME *frameBuf,CanRxMsg *msg)
{

    switch (msg->IDE)
    {
    case CAN_ID_STD:
        frameBuf->IDE = msg->IDE;
        frameBuf->RTR = msg->RTR;
        frameBuf->dataLen = msg->DLC;
        frameBuf->id.canId = msg->StdId;
        memcpy(&frameBuf->canMsg,msg->Data,msg->DLC);
        memset(msg,0,sizeof(CanRxMsg));
        break;
    case CAN_ID_EXT:
        frameBuf->IDE = msg->IDE;
        frameBuf->RTR = msg->RTR;
        frameBuf->dataLen = msg->DLC;
        frameBuf->id.canId = msg->ExtId;/*************/
        memcpy(&frameBuf->canMsg,msg->Data,msg->DLC);//��msg���ݵ����ݶθ��Ƶ�framebuf��
        memset(msg,0,sizeof(CanRxMsg));//�˴�Ϊ��Ҫ����
        break;
    default:
        memset(msg,0,sizeof(CanRxMsg));
        break;
    }
    return frameBuf->RTR;
}




/****************************************************************
���ܣ�CAN���ݽ��սӿں���
��ڲ�����CAN_TypeDef *canChan��CANͨ����,u8 FIFONum-FIFO����,CAN_DATA_FRAME *buf-���ݽ��ջ�����ָ��

*****************************************************************/
u8 CAN_Receive_Msg(u8 canChan,u8 FIFONum,CAN_DATA_FRAME *buf)
{
    CanRxMsg RxMessage;
    switch (canChan)
    {
    case CAN1_CHANNEL:
        /*û�н��յ�����,ֱ���˳�*/
        if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)
        {
            return 0;
        }
//		  g_markBit.isCAN1Recv = YES;
        CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);//��ȡ����
        if (CAN_DataAnalyze(buf,&RxMessage) == 0)
        {
            memset(buf,0,sizeof(CAN_DATA_FRAME));
            return 0;
        }
        break;
    case CAN2_CHANNEL:
        /*û�н��յ�����,ֱ���˳�*/
        if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)
        {
					 //printf("��������\r\n");
            return 2;
        }
//			g_markBit.isCAN2Recv = YES;
        CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);//��ȡ����
        if (RxMessage.RTR == CAN_RTR_Remote)//�����Զ��֡//ֱ��post�ź������ٽ��������
        {
//          OS_ENTER_CRITICAL();
//				
//          OS_EXIT_CRITICAL();
					  CAN_DataAnalyze(RTRbuf,&RxMessage);
					if(RTRbuf->id.idBit.recvDeviceId==ThisTransitionNumber)//ֻ�з������豸��Ӧ��֡�Ż�����ź���
					{
            OSSemPost(can2InfoAckSem);
					  printf("�յ�һ��Ӧ���\r\n");
            memset(&RxMessage,0,sizeof(CAN_DATA_FRAME));//���RxMessage
//            OS_ENTER_CRITICAL();
//						myfree(SRAMIN,RTRbuf);
//            OS_ENTER_CRITICAL();

					}
            return CAN_RTR_Remote;
        }
        else //���������֡
        {
            CAN_DataAnalyze(buf,&RxMessage);
            return CAN_RTR_Data;
        }
    //wk�������ܵ������ݷŵ�buf��ָ�������õĽڵ��� ��if����ִ�к��ж�
    // break;
    default:
        break;
    }
    return buf->dataLen;
}

/****************************************************************
���ܣ�CANͨ�Ŵ���ָʾ����
��ڲ�����u8 canChan��CANͨ����
����ֵ�����ͽ����0��ʾ����ʧ�ܣ�1��ʾ���ͳɹ�
*****************************************************************/
void CAN_ComErr_Display(u8 canChan)
{
    u8 rec = 0;        //Ӳ�����մ��������
    u8 tec = 0;        //Ӳ�����ʹ��������
    u8 lec = 0;        //��һ�δ������
    if(canChan==CAN1_CHANNEL)
    {
        rec = (CAN1->ESR&0xff000000)>>24;
        tec = (CAN1->ESR&0x00ff0000)>>16;
        lec = (CAN1->ESR&0x00000070)>>4;
        if((rec>CAN_threshold)||(tec>CAN_threshold)||((lec>0)&&(lec<7)))     //��׼�ϸ�
        {
//				LED0 = 0;    //����ָʾ����
        }
        else
        {
            //LED0 = 1;
        }
        CAN1->ESR |= 0x00000070; //���������ֶ���Ϊ7
    }
    else if(canChan==CAN2_CHANNEL)
    {
        rec = (CAN2->ESR&0xff000000)>>24;
        tec = (CAN2->ESR&0x00ff0000)>>16;
        lec = (CAN2->ESR&0x00000070)>>4;
        if((rec>CAN_threshold)||(tec>CAN_threshold)||((lec>0)&&(lec<7)))     //��׼�ϸ�
        {
            //LED0 = 0;              //����ָʾ����
        }
        else
        {
            //LED0 = 1;
        }
        CAN2->ESR |= 0x00000070; //���������ֶ���Ϊ7
    }
}


/****************************************************************
���ܣ�CAN��֡���ͺ���
��ڲ�����u8 canChan��CANͨ����,CAN_DATA_FRAME *frameBuf�����ݻ�����ָ��
����ֵ�����ͽ��������æµ�����ͳɹ�������ʧ��,���߹ر�
*****************************************************************/
u8  CAN_Frame_Send(u8 canChan,CAN_DATA_FRAME *frame)
{
    static CAN_SEND_RESULT can1_send_state = SEND_IDLE;//can1_send_state�Ǹ�ö�ٱ������ȶ���λ����
    static CAN_SEND_RESULT can2_send_state = SEND_IDLE;
    OS_CPU_SR cpu_sr;
    u8 can1txok;
    u8 can2txok;
    u8 err = 0;
    if(canChan==CAN1_CHANNEL)
    {
        OS_ENTER_CRITICAL();
        if(can1_send_state== SEND_IDLE)   //�ж�CAN1������Դ�Ƿ���� ���н���
        {
            //ͨ�Ŵ���״ָ̬ʾ
            can1_send_state = SEND_BUSY;//wk:��״̬Ϊ��λæ����ֹ�����ķ�������ռ��
            OS_EXIT_CRITICAL();
            CAN_ComErr_Display(canChan);//��������ͳ��
            if(!(CAN1->ESR & CAN_BOFF_CHECK)) //����δ�ر�  wk��������Ҫ�˽�ESR�Ĵ���
            {
                CAN_Send_Msg(CAN1_CHANNEL,frame->id.canId,frame->IDE,&frame->canMsg,frame->dataLen);
//����CAN���ͺ�������ڲ�����u8 canChan��CANͨ����,u32 canId:��ϢID,u8 frameType:��ϢID�汾��2.0A or 2.0B,u8* msg:���ݻ�����ָ��,u8 len:���ݳ���
                can1txok = (u32)OSMboxPend(can1Mbox,WAIT_CANMBOX_MAXTIME,&err);  //�ȴ������������
                if(can1txok==SEND_SUCCESS)			        //CAN1���ͳɹ�
                {
                    can1txok = 0;
                    can1_send_state = SEND_IDLE;	//wk:������Դ����
                    return SEND_SUCCESS;             //���ͳɹ�
                }
                else
                {
                    can1_send_state = SEND_IDLE; //wk��ʧ��ҲҪ����Դ���У���ֹ���߿��ж����ܷ���
                    return SEND_FAILURE;             //����ʧ��
                }
            }
            else                                     //���߹ر�  wk��else���Ǻ������if���
            {
                can1_send_state = SEND_IDLE;
                return SEND_BUS_CLOSED;
            }
        }
        OS_EXIT_CRITICAL();
    }

    //can2�Ĵ�������ͬcan1
    else if(canChan==CAN2_CHANNEL)
    {
        OS_ENTER_CRITICAL();
        if(can2_send_state== SEND_IDLE)   //�ж�CAN2������Դ�Ƿ����
        {
            can2_send_state = SEND_BUSY;
            OS_EXIT_CRITICAL();
            //ͨ�Ŵ���״ָ̬ʾ
            CAN_ComErr_Display(canChan);
            if(!(CAN2->ESR & CAN_BOFF_CHECK))  //����δ�ر�
            {
                CAN_Send_Msg(CAN2_CHANNEL,frame->id.canId,frame->IDE,&frame->canMsg,frame->dataLen);
                can2txok = (u32)OSMboxPend(can2Mbox,WAIT_CANMBOX_MAXTIME,&err); //�ȴ������������
                /*�˴����ź�����can�����ж���*/
                if(can2txok==SEND_SUCCESS)		       //CAN2���ͳɹ�
                {
                    can2txok = 0;
                    can2_send_state = SEND_IDLE;     //�ͷ�CAN2����ͨ��
                    return SEND_SUCCESS;		         //���ͳɹ�
                }
                else
                {
                    can2_send_state = SEND_IDLE;             //����ʧ��
                    return SEND_FAILURE;
                }
            }
            else                           //���߹ر�
            {
                can2_send_state = SEND_IDLE;
                return SEND_BUS_CLOSED;
            }
        }
        OS_EXIT_CRITICAL();
    }
    return SEND_BUSY;
}



/*******************************************************************************
** ��������: CAN_RTRframe_Send
** ��������: CANͨ��Զ��֡���ͺ���
** ����˵��: canChan: CANͨ��
**			 id: CANԶ��֡ID
**			 IDE: ��Ϣ��ʶ�����ͣ�STM32�б�׼֡���ֶ�Ϊ0x0,��չ֡Ϊ0x4,�����кꣻ CAN_Id_Standard CAN_Id_Extended
** ����˵��: None
** ������Ա: ����
** ��������: 2019-06-26
********************************************************************************/
u8  CAN_RTRframe_Send(u8 canChan,u32 id,u8 IDE)
{
    static CAN_SEND_RESULT can1_send_state = SEND_IDLE;//can1_send_state�Ǹ�ö�ٱ������ȶ���λ����
    static CAN_SEND_RESULT can2_send_state = SEND_IDLE;
    OS_CPU_SR cpu_sr;
    u8 can1txok;
    u8 can2txok;
    u8 err = 0;
    if(canChan==CAN1_CHANNEL)
    {
        OS_ENTER_CRITICAL();
        if(can1_send_state== SEND_IDLE)   //�ж�CAN1������Դ�Ƿ���� ���н���
        {
            //ͨ�Ŵ���״ָ̬ʾ
            can1_send_state = SEND_BUSY;//wk:��״̬Ϊ��λæ����ֹ�����ķ�������ռ��
            OS_EXIT_CRITICAL();
            CAN_ComErr_Display(canChan);//��������ͳ��
            if(!(CAN1->ESR & CAN_BOFF_CHECK)) //����δ�ر�  wk��������Ҫ�˽�ESR�Ĵ���
            {
                CAN_RTRsend_Msg(CAN1_CHANNEL,id,IDE);
//����CAN���ͺ�������ڲ�����u8 canChan��CANͨ����,u32 canId:��ϢID,u8 frameType:��ϢID�汾��2.0A or 2.0B,u8* msg:���ݻ�����ָ��,u8 len:���ݳ���
                can1txok = (u32)OSMboxPend(can1Mbox,WAIT_CANMBOX_MAXTIME,&err);  //�ȴ������������
                if(can1txok==SEND_SUCCESS)			        //CAN1���ͳɹ�
                {
                    can1txok = 0;
                    can1_send_state = SEND_IDLE;	//wk:������Դ����
                    return SEND_SUCCESS;             //���ͳɹ�
                }
                else
                {
                    can1_send_state = SEND_IDLE; //wk��ʧ��ҲҪ����Դ���У���ֹ���߿��ж����ܷ���
                    return SEND_FAILURE;             //����ʧ��
                }
            }
            else                                     //���߹ر�  wk��else���Ǻ������if���
            {
                can1_send_state = SEND_IDLE;
                return SEND_BUS_CLOSED;
            }
        }
        OS_EXIT_CRITICAL();
    }

    //can2�Ĵ�������ͬcan1
    else if(canChan==CAN2_CHANNEL)
    {
        OS_ENTER_CRITICAL();
        if(can2_send_state== SEND_IDLE)   //�ж�CAN2������Դ�Ƿ����
        {
            can2_send_state = SEND_BUSY;
            OS_EXIT_CRITICAL();
            //ͨ�Ŵ���״ָ̬ʾ
            CAN_ComErr_Display(canChan);
            if(!(CAN2->ESR & CAN_BOFF_CHECK))  //����δ�ر�
            {
                CAN_RTRsend_Msg(CAN2_CHANNEL,id,IDE);
                can2txok = (u32)OSMboxPend(can2Mbox,WAIT_CANMBOX_MAXTIME,&err); //�ȴ������������
                /*�˴����ź�����can�����ж���*/
                if(can2txok==SEND_SUCCESS)		       //CAN2���ͳɹ�
                {
                    can2txok = 0;
                    can2_send_state = SEND_IDLE;     //�ͷ�CAN2����ͨ��
                    return SEND_SUCCESS;		         //���ͳɹ�
                }
                else
                {
                    can2_send_state = SEND_IDLE;             //����ʧ��
                    return SEND_FAILURE;
                }
            }
            else                           //���߹ر�
            {
                can2_send_state = SEND_IDLE;
                return SEND_BUS_CLOSED;
            }
        }
        OS_EXIT_CRITICAL();
    }
    return SEND_BUSY;
}

/****************************************************************
���ܣ�CAN���ݴ�������,
��ڲ�����u8 canChan��CANͨ����,CAN_DATA_FRAME *frameBuf�����ݻ�����ָ��wk����������������ݵ�����ָ��
����ֵ����

wk��
*****************************************************************/
void CAN_DataDeal(u8 canChan,CAN_DATA_FRAME *frameBuf)
{
    CAN_DATA_FRAME *deleteNode = NULL,*tempNode = NULL,*newMsgNode = NULL;
    OS_CPU_SR cpu_sr;
//	u8 sendId;
    u8 recvId;  //�����豸��ַ
    tempNode = frameBuf;//wk:��һ����ʱ��ָ��ָ��ͷ��㣬���������ʱ�����иı�ʱ����Ӱ��ͷ���
    switch(canChan)
    {
    case CAN1_CHANNEL:
    {
        while (tempNode->nextMsg != NULL)//wk��ÿ�ζ�����ͷ����ĵ�һ����㣬�������д�����������ɾ��������712--718
        {
            switch (tempNode->nextMsg->id.idBit.ackBit)//wk:Ӧ���־λ���Ƿ���ҪӦ��
            {
            case FRAME_ACTIVE://wk��Ϊ����֡//Ϊ��ҪӦ���֡
            {
                OS_ENTER_CRITICAL();
                newMsgNode = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
                OS_EXIT_CRITICAL();
                if (newMsgNode != NULL)
                {
                    memcpy(newMsgNode,tempNode->nextMsg,sizeof(CAN_DATA_FRAME));

                    tempNode = g_canDataActFrame[CAN1_CHANNEL];/*wangkai������֡��ͷ��㣬��������ʼ�����Ѿ������ȫ�ֱ���*/
                    /*��tempnode��ֵΪ����֡ͷ��㣬�������б���*/
                    while(tempNode->nextMsg != NULL)
                    {
                        tempNode = tempNode->nextMsg;
                    }
                    /*������֡������������ ���յ�������֡��������ȡ������֡���*/
                    tempNode->nextMsg = newMsgNode;
                    newMsgNode->nextMsg = NULL;
                    OSSemPost(can1ActSem);
                }
                break;
            }

            /*Ӧ��֡ͬ����֡��Ҳ���Ƚ����еĽ��������������б����ҵ����е�Ӧ��֡������Ӧ���־λ��*/
            /*�ٰ�Ӧ��֡�������б����������һ����㣬֮�󽫴ӱ����ҵ���Ӧ��֡���ӵ�Ӧ��֡�����ĺ���*/
            case FRAME_REPLY://ΪӦ��֡
            {
                OS_ENTER_CRITICAL();
                newMsgNode = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
                OS_EXIT_CRITICAL();
                if (newMsgNode != NULL)
                {
                    memcpy(newMsgNode,tempNode->nextMsg,sizeof(CAN_DATA_FRAME));
                    tempNode = g_canDataAckFrame[CAN1_CHANNEL];//��һ��������can1��2������֡ͷ���
                    while(tempNode->nextMsg != NULL)
                    {
                        tempNode = tempNode->nextMsg;
                    }
                    tempNode->nextMsg = newMsgNode;
                    newMsgNode->nextMsg = NULL;
                    OSSemPost(can1AckSem);
                }
                break;
            }
            default:
                break;
            }
            tempNode = frameBuf;//����ָ��������ݻ���ͷ���
            deleteNode = tempNode->nextMsg;		//�ѵ�һ��һ�����ɾ��
            tempNode->nextMsg = deleteNode->nextMsg;
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,deleteNode);//�ͷ��ڴ�
            OS_EXIT_CRITICAL();
            //}/*wangkai���Ķ����˴����������������������ͷ���֮��һֱ�Ƿǿ�  ��������*/
        }
        break;

    }


    /*wk��can2ͬcan1���ǰ��Լ��Ľ��ܵ��Ķ����������У�ÿ�η�����һ����㣬��������֡����Ӧ��֡����Ӧ�ķ��ڸ��Ե������У�֮��������ͷ���ɾ��*/
    case CAN2_CHANNEL:
    {
        while (tempNode->nextMsg != NULL)
        {
					/*�����ʱ������������������ʱ��֡����������������豸���ͣ���˵������������豸��֡������*/
					 if(tempNode->nextMsg->id.MasteridBit.MasterslaveBit==MasterSlave&&tempNode->nextMsg->id.MasteridBit.MainIndex!=EquipmentType&&tempNode->nextMsg->id.MasteridBit.MainIndex!=0)
					 {
						  //ɾ������
                tempNode = frameBuf;
                deleteNode = tempNode->nextMsg;
                tempNode->nextMsg = deleteNode->nextMsg;
                OS_ENTER_CRITICAL();
                myfree(SRAMIN,deleteNode);
                OS_EXIT_CRITICAL();
					 }
					 else//����
					 {
            recvId=tempNode->nextMsg->id.idBit.recvDeviceId;
            /*��id�Ž����жϣ���ֹ���������*/
            if(recvId==ThisTransitionNumber||recvId==0xff)//Ϊ���������ǹ㲥
            {
                LED_BlinkTime.LED1_Times++;//�õ�1��˸һ��
                switch (tempNode->nextMsg->id.idBit.ackBit)
                {
                case FRAME_ACTIVE:
                {
//                    if(tempNode->nextMsg->dataLen!=0)//���Ӷ����ݳ��ȵ��жϣ�������֡�У�������ֳ���Ϊ0�����
//                    {
                    OS_ENTER_CRITICAL();
                    newMsgNode = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
                    OS_EXIT_CRITICAL();
                    if (newMsgNode != NULL)
                    {
                        memcpy(newMsgNode,tempNode->nextMsg,sizeof(CAN_DATA_FRAME));
                        tempNode = g_canDataActFrame[CAN2_CHANNEL];
                        while(tempNode->nextMsg != NULL)
                        {
                            tempNode = tempNode->nextMsg;
                        }
                        tempNode->nextMsg = newMsgNode;
                        newMsgNode->nextMsg = NULL;
                        OSSemPost(can2ActSem);
                    }
                    break;
//                    }
//                    else break;
                }
                case FRAME_REPLY:
                {
                    //�����ź���������
                    OS_ENTER_CRITICAL();
                    newMsgNode = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
                    OS_EXIT_CRITICAL();
                    if (newMsgNode != NULL)
                    {
                        memcpy(newMsgNode,tempNode->nextMsg,sizeof(CAN_DATA_FRAME));
                        tempNode = g_canDataAckFrame[CAN2_CHANNEL];
                        while(tempNode->nextMsg != NULL)
                        {
                            tempNode = tempNode->nextMsg;
                        }
                        tempNode->nextMsg = newMsgNode;
                        newMsgNode->nextMsg = NULL;
                        OSSemPost(can2AckSem);
                    }
                    break;
                }
                default:
                    break;
                }
                tempNode = frameBuf;
                deleteNode = tempNode->nextMsg;
                tempNode->nextMsg = deleteNode->nextMsg;
                OS_ENTER_CRITICAL();
                myfree(SRAMIN,deleteNode);
                OS_EXIT_CRITICAL();
            }
            else
            {
                //ɾ������
                tempNode = frameBuf;
                deleteNode = tempNode->nextMsg;
                tempNode->nextMsg = deleteNode->nextMsg;
                OS_ENTER_CRITICAL();
                myfree(SRAMIN,deleteNode);
                OS_EXIT_CRITICAL();
            }
					}
        }
        break;
    }
    default:
        break;
    }
}

/*******************************************************************************
** ��������: CAN_Act_DataDeal
** ��������:
** ����˵��: canChan: [����/��]
**			 frameBuf: [����/��]
** ����˵��: None
** ������Ա: ����
** ��������: 2019-06-26 ��������в����еȴ���������ȵĹ���
********************************************************************************/
void CAN_Act_DataDeal(u8 canChan,CAN_DATA_FRAME *frameBuf)
{
    CAN_DATA_FRAME *deleteNode = NULL,*tempNode = NULL;
//		INT8U oserr;
    u32 id;
//    u8 device_id;
    u8 sendId;
    u8 recvId;
    u8 index;
    OS_CPU_SR cpu_sr;
    tempNode = frameBuf;
    switch(canChan)
    {
    case CAN1_CHANNEL:
    {
        while(tempNode->nextMsg != NULL)
        {
            sendId = tempNode->nextMsg->id.idBit.recvDeviceId;
            recvId = tempNode->nextMsg->id.idBit.sendDeviceId;
            //device_id = tempNode->nextMsg->id.idBit.sendDeviceId;//��÷����豸ID���Ժ�����жϣ��˴�����
            index=tempNode->nextMsg->id.idBit.index;
            id = Get_CAN_ExId(true,sendId,recvId)|index;//�������͵�ַ����յ�ַ
            tempNode->nextMsg->id.idBit.ackBit = FRAME_REPLY;//Ӧ���־λ��1
            //OSTimeDlyHMSM(0,0,0,5);
            CAN_Send_Frame_App(canChan,id,CAN_Id_Extended,&tempNode->nextMsg->canMsg,sizeof(CAN_MSG),frameBuf,CAN_ACK_RETRY_TIME);//����
            deleteNode = tempNode->nextMsg;
            tempNode->nextMsg = deleteNode->nextMsg;
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,deleteNode);
            OS_EXIT_CRITICAL();
        }
        break;
    }
    case CAN2_CHANNEL:
    {
        while(tempNode->nextMsg != NULL)
        {
            index=tempNode->nextMsg->id.idBit.index;
            if(tempNode->nextMsg->id.idBit.recvDeviceId==0xff)
            {
                Broadcast_Judegment(canChan,tempNode->nextMsg);//�����ϵ���������
            }
            else
            {
                if(tempNode->nextMsg->id.MasteridBit.MasterslaveBit==ManyMaster)//����Ƕ�����֡//Ҳ����С��11111
                {
                    Index_Judegment(canChan,id,index,tempNode->nextMsg,frameBuf);
                }
                else//���������//Ҳ������000000
                {
                    Master_Index_judegment(canChan,tempNode->nextMsg,frameBuf);
                }
            }
            deleteNode = tempNode->nextMsg;
            tempNode->nextMsg = deleteNode->nextMsg;
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,deleteNode);
            OS_EXIT_CRITICAL();
        }
        break;
    }
    default:
        break;
    }
}

/*******************************************************************************
** ��������: ACKSendFram
** ��������: Ӧ��֡���ͺ�������Զ��������ӽ���������Ż�
** ����˵��: canChan: [����/��]
**			 frameBuf: [����/��]
** ����˵��: None
** ������Ա: ����
** ��������: 2019-07-14
********************************************************************************/
void ACKSendFram(u8 canChan,CAN_DATA_FRAME *tempNode)
{
    u32 id;
    //u8 device_id;
    u8 sendId;
    u8 recvId;
    u8 index;
	  u8 SendRes;
		u8 ReSendRes;
    u16 MSindex;
    /*���յ���֡Ϊ����֡ʱ*/
    if(tempNode->id.idBit.MasterslaveBit==ManyMaster)//�Ƿ��Ƕ���//����Ϊ1//����Ϊ0
    {
        index=tempNode->id.idBit.index;//�����ǰ�λ
        sendId = tempNode->id.idBit.recvDeviceId;//�������豸�źͽ����豸����Ե�
        recvId = tempNode->id.idBit.sendDeviceId;//�������豸�źͽ����豸����Ե�
        // device_(null)id = tempNode->nextMsg->id.idBit.sendDeviceId;
        id = Get_CAN_ExId(FRAME_REPLY,sendId,recvId)|index|(ManyMaster<<28);//������־��1//����֡��־��1
        tempNode->id.idBit.ackBit = FRAME_REPLY;
       SendRes = CAN_RTRframe_Send(canChan,id,CAN_Id_Extended);//���̻ظ�һ��Զ��֡��Ӧ��//ͨ����//id
			if(SendRes==SEND_SUCCESS)
			{
				printf("<����ID��%llu>����Ӧ��֡���ͺ����ɹ�\r\n",printfcount);
			}
			else if(SendRes==SEND_FAILURE)
			{
				ReSendRes = CAN_RTRframe_Send(canChan,id,CAN_Id_Extended);//���̻ظ�һ��Զ��֡��Ӧ��//ͨ����//id
				printf("<����ID��%llu>��ERR������Ӧ��֡���ͺ���ʧ�ܣ��ط�����%d\r\n",printfcount,ReSendRes);//�ɹ���4��ʧ����3
			}
			else if(SendRes==SEND_BUS_CLOSED)
			{
				printf("<����ID��%llu>����Ӧ��֡���ͺ����ɹ�",printfcount);
			}
    }
    else if(tempNode->id.idBit.MasterslaveBit==MasterSlave)//����ģʽ�� 0000
    {
        MSindex=(tempNode->id.idBit.index+(tempNode->id.idBit.sendDeviceId<<8));//ID
        //device_id=tempNode->id.idBit.recvDeviceId;
        id=(MasterSlave<<28)+(FRAME_REPLY<<26)+(ThisTransitionNumber<<16)+MSindex;//������־0//Ӧ��֡��־11//�豸id//��������
        CAN_RTRframe_Send(canChan,id,CAN_Id_Extended);//���̻ظ�һ��Զ��֡��Ӧ��//ͨ����//id
    }
    else return;

}



/*wangkai����Ӧ��֡�����߳����˸Ķ����������OSSemPost(can2InfoAckSem)�ź�������ɾ��Ӧ��֡���������ĵ�һ�����*/
void CAN_Ack_DataDeal(u8 canChan,CAN_DATA_FRAME *frameBuf)//wk:framebuf�����Ӧ��֡ͷָ��
{
    CAN_DATA_FRAME *deleteNode = NULL,*tempNode = NULL;
    OS_CPU_SR cpu_sr;
    tempNode = frameBuf;

    switch(canChan)
    {
    case CAN1_CHANNEL:
    {
        while (tempNode->nextMsg != NULL)
        {
            OSSemPost(can1InfoAckSem);
            deleteNode = tempNode->nextMsg;
            tempNode->nextMsg = deleteNode->nextMsg;
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,deleteNode);
            OS_EXIT_CRITICAL();
        }
        break;
    }
    case CAN2_CHANNEL:
    {

        while (tempNode->nextMsg != NULL)
        {

//            if(tempNode->nextMsg->dataLen!=0)//˵���ǲ���Զ��֡������Ҫִ�ж���������֡
//            {
                if(tempNode->nextMsg->id.idBit.recvDeviceId==0xff)
                {
                    Broadcast_Judegment(canChan,tempNode->nextMsg);
                }
                else
                    StateQuery_Index_judegment(tempNode->nextMsg);//����ҪӦ���֡
//            }
            deleteNode = tempNode->nextMsg;
            tempNode->nextMsg = deleteNode->nextMsg;
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,deleteNode);
            OS_EXIT_CRITICAL();
        }
        break;
    }
    default:
        break;
    }

}



/****************************************************************
���ܣ�CAN1�����жϷ�����  wk�����յ������������
��ڲ�������
����ֵ����;
*****************************************************************/
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�				    
void CAN1_RX0_IRQHandler(void)
{

    u8 datalen = 0;
    OS_CPU_SR cpu_sr;
    CAN_DATA_FRAME *tempMsgNode = NULL,*newMsgNode = NULL;
    OSIntEnter();
    OS_ENTER_CRITICAL();
    newMsgNode = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
    if (newMsgNode != NULL)
    {
        OS_EXIT_CRITICAL();
        datalen =  CAN_Receive_Msg(CAN1_CHANNEL, CAN1_FIFO0,newMsgNode);
        if (datalen > 0)
        {
            tempMsgNode = g_canDataFrame[CAN1_CHANNEL];
            /*tempmesnodeָ��ӽ������ݵ�ͷ���*/
            while (tempMsgNode->nextMsg != NULL)
            {
                tempMsgNode = tempMsgNode->nextMsg;
            }
            /*�����������*/
            tempMsgNode->nextMsg = newMsgNode;
            newMsgNode->nextMsg = NULL;
            OSSemPost(can1RecvSem);

        }
        else
        {
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,newMsgNode);
            OS_EXIT_CRITICAL();
        }
    }
    else
    {
        OS_EXIT_CRITICAL();
    }
    OSIntExit();
}
#endif




/****************************************************************
���ܣ�CAN2�����жϷ�����
��ڲ�������
����ֵ����;
*****************************************************************/
#if CAN2_RX0_INT_ENABLE	//ʹ��RX0�ж�

void CAN2_RX0_IRQHandler(void)
{
    OS_CPU_SR cpu_sr;
    u8 framflage;
    CAN_DATA_FRAME *tempMsgNode = NULL,*newMsgNode = NULL;
    OSIntEnter();
    OS_ENTER_CRITICAL();
    newMsgNode = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));//�жϾ����������ڴ�
    if (newMsgNode != NULL)
    {
        OS_EXIT_CRITICAL();
        framflage=CAN_Receive_Msg(CAN2_CHANNEL, CAN2_FIFO0,newMsgNode);//֡����

        if(framflage==CAN_RTR_Data)//���������֡
        {
            tempMsgNode = g_canDataFrame[CAN2_CHANNEL];
            while (tempMsgNode->nextMsg != NULL)
            {
                tempMsgNode = tempMsgNode->nextMsg;
            }
            tempMsgNode->nextMsg = newMsgNode;
            newMsgNode->nextMsg = NULL;
            OSSemPost(can2RecvSem);
        }
        else//�����������֡�����������
        {
            myfree(SRAMIN,newMsgNode);
        }
    }
    else
    {
        OS_EXIT_CRITICAL();
    }
    OSIntExit();
}

#endif



/****************************************************************
���ܣ�CAN1�����жϷ�����
��ڲ�������
����ֵ����;
*****************************************************************/
void CAN1_TX_IRQHandler(void)
{
    u8 Tx1OK;
    u8 flag;
	  OSIntEnter();//2019��11��5�� 13��04��  ���������ͺ���û�м��жϲ���ͳ��
    if(SET == CAN_GetITStatus(CAN1,CAN_IT_TME))
    {
        flag = (CAN1->TSR & CAN_TSR_TXOK0)>>1;
        if(1==flag)//2019-07-11:�޸ģ�Ҫ��Ҫͬ�����䷢�͵Ľ��ֵ����ת���£�ԭ���������в��ܷ���0
        {
            Tx1OK=SEND_SUCCESS;//��ö������4
        }
        else
        {
            Tx1OK=SEND_FAILURE;//��3
        }
        OSMboxPost(can1Mbox,(void *)Tx1OK);
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
    }
    OSIntExit();//2019��11��5�� 13��04��  �����������ն�û���жϲ���ͳ��

}


/****************************************************************
���ܣ�CAN2�����жϷ�����
��ڲ�������
����ֵ����;
*****************************************************************/
void CAN2_TX_IRQHandler(void)
{
    u8 Tx2OK;
    u8 flag;
		OSIntEnter();/*2019��11��5�� 13��04��  ���������ͺ���û�м��жϲ���ͳ�ƣ�*/
    if(SET == CAN_GetITStatus(CAN2,CAN_IT_TME))
    {
        flag = (CAN2->TSR & CAN_TSR_TXOK0)>>1;
        if(1==flag)//2019-07-11:�޸ģ�Ҫ��Ҫͬ�����䷢�͵Ľ��ֵ����ת���£�ԭ���������в��ܷ���0
        {
            Tx2OK=SEND_SUCCESS;//��ö������4
        }
        else
        {
            Tx2OK=SEND_FAILURE;//��3
        }
        OSMboxPost(can2Mbox,(void *)Tx2OK);
        LED_BlinkTime.LED2_Times++;
        CAN_ClearITPendingBit(CAN2, CAN_IT_TME);

    }
	 OSIntExit();/*2019��11��5�� 13��04��  �����������ն�û���жϲ���ͳ�ƣ�*/

}



/****************************************************************
���ܣ�CAN����֡���ͽӿں���
��ڲ�����u8 canChan��CANͨ����,u32 canId:��ϢID,
u8 frameType:��ϢID�汾��2.0A or 2.0B,u8* msg:���ݻ�����ָ��,u8 len:���ݳ���
����ֵ��0,�ɹ�; ����,ʧ��;
*****************************************************************/
u8 CAN_Send_Msg(u8 channel,u32 id,u8 frameType,CAN_MSG *msg,u8 len)
{
    u8 res=0;
    OS_CPU_SR cpu_sr;
    static CanTxMsg TxMessage;
    OS_ENTER_CRITICAL();
    TxMessage.IDE = frameType;      //��չ֡
    if(frameType==CAN_Id_Standard)  //id��
    {
        TxMessage.StdId = id;
    }
    else if(frameType==CAN_Id_Extended)
    {
        TxMessage.ExtId = id;
    }
    TxMessage.RTR = 0;		  // ��Ϣ����Ϊ����֡
    TxMessage.DLC = len;		//һ֡8�ֽ�
    memcpy(TxMessage.Data,msg,len);
    OS_EXIT_CRITICAL();
    switch(channel)
    {
    case CAN1_CHANNEL:
    {
        res = CAN_Transmit(CAN1, &TxMessage);
//				g_markBit.isCAN1Send = YES;
    }
    break;
    case CAN2_CHANNEL:
    {
        res = CAN_Transmit(CAN2, &TxMessage);
//				g_markBit.isCAN2Send = YES;
    }
    break;
    default:
        break;
    }
    return res;
}



/*******************************************************************************
** ��������: CAN_RTRsend_Msg
** ��������: CANͨ��Զ��֡���ͽӿں���
** ����˵��: channel: CANͨ����
**			 id: ֡ID
**			 frameType: Ϣ��ʶ�����ͣ�CANЭ���б�׼֡Ϊ���ԣ���չ֡���Σ�STM32�б�׼֡���ֶ�Ϊ0x0,��չ֡Ϊ0x4,�����к�
** ����˵��: None
** ������Ա: ����
** ��������: 2019-06-26
********************************************************************************/
u8 CAN_RTRsend_Msg(u8 channel,u32 id,u8 frameType)
{
    u8 res=0;
    OS_CPU_SR cpu_sr;
    static CanTxMsg TxMessage;
    OS_ENTER_CRITICAL();
    TxMessage.IDE = frameType;      //��չ֡
    if(frameType==CAN_Id_Standard)  //id��
    {
        TxMessage.StdId = id;
    }
    else if(frameType==CAN_Id_Extended)
    {
        TxMessage.ExtId = id;
    }
    TxMessage.RTR = CAN_RTR_REMOTE;		  // ��Ϣ����ΪԶ��֡
    OS_EXIT_CRITICAL();
    switch(channel)
    {
    case CAN1_CHANNEL:
    {
        res = CAN_Transmit(CAN1, &TxMessage);
//				g_markBit.isCAN1Send = YES;
    }
    break;
    case CAN2_CHANNEL:
    {
        res = CAN_Transmit(CAN2, &TxMessage);
//				g_markBit.isCAN2Send = YES;
    }
    break;
    default:
        break;
    }
    return res;
}

/****************************************************************
���ܣ�CANͨ�Ŵ���ͳ�ƺ���
��ڲ�����u8 canChan��CANͨ����,u8:��һ�δ�������
����ֵ����
*****************************************************************/
void CAN_Err_Statistic(u8 channel,u8 lec)
{
//	if(channel==CAN1_CHANNEL)
//	{
//			if(lec>0&&lec<7)        //��һ����Ч�������ͷ�ΧΪ0��7
//			{
//				if(lec==ERR_FILL)
//				{
//					if(savedata.can1_err_cnt_fill<INT_MAX)
//					{
//							savedata.can1_err_cnt_fill++;
//					}
//				}
//				else if(lec==ERR_FORMAT)
//				{
//					if(savedata.can1_err_cnt_format<INT_MAX)
//					{
//							savedata.can1_err_cnt_format++;
//					}
//				}
//				else if(lec==ERR_ACK)
//				{
//					if(savedata.can1_err_cnt_ack<INT_MAX)
//					{
//							savedata.can1_err_cnt_ack++;
//					}
//				}
//				else if(lec==ERR_INVISIBLE)
//				{
//					if(savedata.can1_err_cnt_invisible<INT_MAX)
//					{
//							savedata.can1_err_cnt_invisible++;
//					}
//				}
//				else if(lec==ERR_VISIBLE)
//				{
//					if(savedata.can1_err_cnt_visible<INT_MAX)
//					{
//							savedata.can1_err_cnt_visible++;
//					}
//				}
//				else if(lec==ERR_CRC)
//				{
//					if(savedata.can1_err_cnt_crc<INT_MAX)
//					{
//							savedata.can1_err_cnt_crc++;
//					}
//
//				}
//			}
//	}
//	else if(channel==CAN2_CHANNEL)
//	{
//			if(lec>0&&lec<7)
//			{
//					if(lec==ERR_FILL)
//					{
//						if(savedata.can2_err_cnt_fill<INT_MAX)
//						{
//								savedata.can2_err_cnt_fill++;
//						}
//					}
//					else if(lec==ERR_FORMAT)
//					{
//						if(savedata.can2_err_cnt_format<INT_MAX)
//						{
//								savedata.can2_err_cnt_format++;
//						}
//					}
//					else if(lec==ERR_ACK)
//					{
//						if(savedata.can2_err_cnt_ack<INT_MAX)
//						{
//								savedata.can2_err_cnt_ack++;
//						}
//					}
//					else if(lec==ERR_INVISIBLE)
//					{
//						if(savedata.can2_err_cnt_invisible<INT_MAX)
//						{
//								savedata.can2_err_cnt_invisible++;
//						}
//					}
//					else if(lec==ERR_VISIBLE)
//					{
//						if(savedata.can2_err_cnt_visible<INT_MAX)
//						{
//								savedata.can2_err_cnt_visible++;
//						}
//					}
//					else if(lec==ERR_CRC)
//					{
//						if(savedata.can2_err_cnt_crc<INT_MAX)
//						{
//								savedata.can2_err_cnt_crc++;
//						}
//					}
//			}
//	}
//
}




/****************************************************************
���ܣ�CAN�������޼��غ���
��ڲ�����u8 err_limit����������
����ֵ��0,�ɹ�; ����,ʧ��;
*****************************************************************/
void Load_CAN_Threshold(u8 err_limit)
{
    if(err_limit==0)
    {
        CAN_threshold = CAN_ESR_MIN;
    }
    else if(err_limit==1)
    {
        CAN_threshold = CAN_ESR_MID;
    }
    else if(err_limit==2)
    {
        CAN_threshold = CAN_ESR_MAX;
    }
    //Modbus_Fill_CANErrCnt();
}



/****************************************************************
���ܣ�CAN������亯������
��ڲ�����u32 canId:��ϢID,u8 frameType:��ϢID�汾��2.0A or 2.0B,u8* msg:���ݻ�����ָ��,u8 len:���ݳ��ȣ�CAN_DATA_FRAME *frameBuf:�������ݻ�����
����ֵ����
*****************************************************************/
void CAN_Fill_Msg(u32 id,u8 frameType,CAN_MSG *msg,u8 len,CAN_DATA_FRAME *frameBuf)
{
    frameBuf->id.canId = id;
    frameBuf->IDE = frameType;
    frameBuf->dataLen = len;
    memcpy(&frameBuf->canMsg,msg,sizeof(CAN_MSG));
}




///****************************************************************
//���ܣ���ȡCAN��չ֡��ַ
//��ڲ�����bool ackBit,u8 sendAddr,u8 recvAddr
//����ֵ��CAN��չ֡��ַ;
//*****************************************************************/
u32 Get_CAN_ExId(bool ackBit,u8 sendAddr,u8 recvAddr)
{
    u32 ExId;
    ExId = (ackBit<<26)|(recvAddr<<16)|(sendAddr<<8);
    return ExId;
}



void CAN_Fill_Data(u8 data0,u8 data1,u8 data2,u8 data3,u8 data4,u8 data5,u8 data6,u8 data7,CAN_MSG *msg)
{

    msg->dataBuf[0] = data2;
    msg->dataBuf[1] = data3;
    msg->dataBuf[2] = data4;
    msg->dataBuf[3] = data5;
    msg->dataBuf[4] = data6;
    msg->dataBuf[5] = data7;
}

//void CAN_Fill_Mission(uint8_t mainIndex,Mission task,CAN_MSG *msg)
//{
//���·�Ϊ����������Ϣ�������쳣��Ϣ

//	Mission mission;
//	if(task.status<MISSION_SUSPEND || (task.status==MISSION_FINISH && task.faultCode==MISSION_ENABLED))
//	{
//		//�������
//		//*(data+0)=mainIndex;
//		mission.status=task.status;
//		mission.sendSystemId=task.sendSystemId;
//		mission.recvSystemId=task.recvSystemId;
//		mission.sendDeviceId=task.sendDeviceId;
//		mission.speed=task.speed;
//		mission.recvDeviceId=task.recvDeviceId;
//		mission.prior=task.prior;
//		mission.carrierId=task.carrierId;
//		mission.mode=task.mode;
//	}
//	else if(task.status>=MISSION_SUSPEND && task.status<MISSION_FINISH)
//	{
//		//�������
//		//*(data+0)=mainIndex;
//		mission.status=MISSION_SUSPEND+task.faultCode;
//		mission.sendSystemId=task.faultSystemId;
//		mission.recvSystemId=task.recvSystemId;
//		mission.sendDeviceId=task.faultDeviceId;
//		mission.speed=task.speed;
//		mission.recvDeviceId=task.recvDeviceId;
//		mission.prior=task.prior;
//		mission.carrierId=task.carrierId;
//		mission.mode=task.mode;
//	}
//	else if(task.status>=MISSION_FINISH)
//	{
//		//�������
//		//*(data+0)=mainIndex;
//		mission.status=MISSION_FINISH+task.faultCode;
//		mission.sendSystemId=task.faultSystemId;
//		mission.recvSystemId=task.recvSystemId;
//		mission.sendDeviceId=task.faultDeviceId;
//		mission.speed=task.speed;
//		mission.recvDeviceId=task.recvDeviceId;
//		mission.prior=task.prior;
//		mission.carrierId=task.carrierId;
//		mission.mode=task.mode;
//	}
//	msg->mainIndex = mainIndex;
//	memcpy(&msg->subIndex,&mission,7);
//}


/*wk�������*/
bool CAN_Send_Frame_App(u8 canChan,u32 id,u8 frameType,CAN_MSG *canMsg,u8 len,CAN_DATA_FRAME *frameBuf,u16 sendRetryTime)
{
    OS_CPU_SR cpu_sr;
    u16 cntCAN1 =0;
    u16 cntCAN2 =0;
    switch(canChan)
    {
    case CAN1_CHANNEL:
        OS_ENTER_CRITICAL();
        frameBuf = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
        OS_EXIT_CRITICAL();
        if (frameBuf!= NULL)
        {
            frameBuf->nextMsg = NULL;
            CAN_Fill_Msg(id,CAN_Id_Extended,canMsg,len,frameBuf);
            //2018.6.11 YQ�޸�
            //OSTimeDlyHMSM(0,0,0,5);//STM32���Բ���
            while(CAN_Frame_Send(canChan,frameBuf)!=SEND_SUCCESS)//����
            {
                OSTimeDlyHMSM(0,0,0,5);
                cntCAN1++;
                if(cntCAN1>=sendRetryTime)
                {
                    OS_ENTER_CRITICAL();
                    myfree(SRAMIN,frameBuf);
                    OS_EXIT_CRITICAL();
                    return false;                    //CAN���������⵼�·���ʧ��
                }
            }
            //���ͳɹ�
            OS_ENTER_CRITICAL();
            myfree(SRAMIN,frameBuf);
            OS_EXIT_CRITICAL();
            return true;
        }

        break;
    case CAN2_CHANNEL:
        OS_ENTER_CRITICAL();
        frameBuf = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
        OS_EXIT_CRITICAL();
        if (frameBuf!= NULL)
        {
            frameBuf->nextMsg = NULL;
        }
        CAN_Fill_Msg(id,CAN_Id_Extended,canMsg,len,frameBuf);
        //OSTimeDlyHMSM(0,0,0,2);   //֡��֮֡������2ms�ļ��
        while(CAN_Frame_Send(canChan,frameBuf)!=SEND_SUCCESS)
        {
            OSTimeDlyHMSM(0,0,0,20);
            cntCAN2++;
            if(cntCAN2>=sendRetryTime)
            {
                OS_ENTER_CRITICAL();
                myfree(SRAMIN,frameBuf);
                OS_EXIT_CRITICAL();
                return false;
            }
        }
        //���ͳɹ�
        OS_ENTER_CRITICAL();
        myfree(SRAMIN,frameBuf);
        OS_EXIT_CRITICAL();
        return true;
    //break;
    default:
        return false;
        //break;
    }
    return false;
}



//void CAN_Send_Frame_App(u8 canChan,u32 id,u8 frameType,CAN_MSG *canMsg,u8 len,CAN_DATA_FRAME *frameBuf,u16 sendRetryTime)
//{
//		OS_CPU_SR cpu_sr;
//		u16 cntCAN1 =0;
//		u16 cntCAN2 =0;
//	  switch(canChan)
//		{
//					case CAN1_CHANNEL:
//						OS_ENTER_CRITICAL();
//						frameBuf = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
//						OS_EXIT_CRITICAL();
//						if (frameBuf!= NULL)
//						{
//							frameBuf->nextMsg = NULL;
//						}
//						CAN_Fill_Msg(id,CAN_Id_Extended,canMsg,sizeof(CAN_MSG),frameBuf);

//						while(CAN_Frame_Send(canChan,frameBuf)!=SEND_SUCCESS)
//						{
//							OSTimeDlyHMSM(0,0,0,2);
//							cntCAN1++;
//							if(cntCAN1>=sendRetryTime)
//							{
//								break;
//							}
//						}

//						//���ͳɹ�
//						OS_ENTER_CRITICAL();
//						myfree(SRAMIN,frameBuf);
//						OS_EXIT_CRITICAL();
//						break;
//					case CAN2_CHANNEL:
//						OS_ENTER_CRITICAL();
//						frameBuf = (CAN_DATA_FRAME *)mymalloc(SRAMIN,sizeof(CAN_DATA_FRAME));
//						OS_EXIT_CRITICAL();
//						if (frameBuf!= NULL)
//						{
//							frameBuf->nextMsg = NULL;
//						}
//						//memset(&canMsg,0,sizeof(CAN_MSG));  //BUF���ݶ�����
//						CAN_Fill_Msg(id,CAN_Id_Extended,canMsg,sizeof(CAN_MSG),frameBuf);

//						while(CAN_Frame_Send(canChan,frameBuf)!=SEND_SUCCESS)
//						{
//							OSTimeDlyHMSM(0,0,0,2);
//							cntCAN2++;
//							if(cntCAN2>=sendRetryTime)
//							{
//								break;
//							}
//						}
//						//���ͳɹ�
//						OS_ENTER_CRITICAL();
//						myfree(SRAMIN,frameBuf);
//						OS_EXIT_CRITICAL();
//						break;
//					default:
//						break;
//			}
//}







///****************************************************************
//���ܣ�CAN1��������
//��ڲ�������
//����ֵ����;
//*****************************************************************/
void CAN1_HeartBeat_Task(void *pdata)
{
//		CAN_DATA_FRAME *frameBuf;
//		CAN_MSG canMsg;
//		u32 id;
//		u8 i;
//		pdata = pdata;
//		while(1)
//		{
//				for(i=0;i<DEVICE_NUM;i++)
//				{
//					if (i==DOMAIN_ID)                        continue;
//					if (channel1[i].commStatus<=COMM_CLOSED) continue;
    //��ʱ��������,Ϊ�˼���վ����ͬһʱ��Ĺ�������ȡ����ʱ5S��վ�㷢��������������վ�㷢��һ����������������������ظ�Ӧ��������
//					channel1[i].sendTimer++;
//					if (channel1[i].sendTimer>=50)       //5S����������վ�������
//					{
//						channel1[i].sendTimer=0;
//						id = Get_CAN_ExId(true,DOMAIN_ID,i);
//						CAN_Fill_Data(0,0,0,0,0,0,0,0,&canMsg);
//						OSTimeDlyHMSM(0,0,0,5);
//						CAN_Send_Frame_App(CAN1_CHANNEL,id,CAN_Id_Extended,&canMsg,sizeof(CAN_MSG),frameBuf,CAN_HEARTBEAT_RETRY_TIME);
//					}

    //������������ʱ
//					channel1[i].recvTimer++;
//					if (channel1[i].recvTimer>60)       //�������վ���������ʱΪ6s
//					{
//						HeartBeatTimeOut(CAN1_CHANNEL,&channel1[i]);
//					}
//					RayErr_Filter(sys->devices[i]);				//��г����˲�����
//				}
//				//����ȵ�ϵͳ���ݼ�����ɺ��ٽ��и���ϵͳ�豸�Ĵ�������
//				UpdateModbusRegInfo();//����MODBUS�Ĵ�����ֵ
//				OSTimeDlyHMSM(0,0,0,100);
//
//		}
}


/****************************************************************
���ܣ�CAN2��������
��ڲ�������
����ֵ����;
*****************************************************************/
void CAN2_HeartBeat_Task(void *pdata)
{
//		//static u8 syncTimer=0;
//		CAN_DATA_FRAME *frameBuf;
//		u32 id;
//		u8  i;
//		CAN_MSG canMsg;
//		pdata = pdata;
//		while(1)
//		{
//				for(i=0;i<SYSTEM_NUM;i++)
//				{
//					if (i==sys->systemId)                    continue;
//					if (channel2[i].commStatus<=COMM_CLOSED) continue;
//					//��ʱ��������
//					channel2[i].sendTimer++;
//					if (channel2[i].sendTimer>=50)
//					{
//						 channel2[i].sendTimer=0;
//						id = Get_CAN_ExId(true,sys->systemId,i);  //ģ�����
//						CAN_Fill_Data(0,0,0,0,0,0,0,0,&canMsg);
//						CAN_Send_Frame_App(CAN2_CHANNEL,id,CAN_Id_Extended,&canMsg,sizeof(CAN_MSG),frameBuf,CAN_HEARTBEAT_RETRY_TIME);
//					}
//
//					//���������ʱ
//					channel2[i].recvTimer++;
//					if (channel2[i].recvTimer>60)
//					{
//						HeartBeatTimeOut(CAN2_CHANNEL,&channel2[i]);
//					}
//
//					//ͬ����ʱ
//					if(systems[i]->isDataSync)  //ͬ��������
//					{
//						if(systems[i]->syncTimer<SYNC_DATA_MAXTIME)
//						{
//							systems[i]->syncTimer++;
//						}
//						else      //ͬ����ʱ
//						{
//							systems[i]->syncTimer = 0;
//							systems[i]->isDataSync = 0;
//							//2018.11.4,������Ҫ�޸ģ�ֻҪͬ��״̬����
//							Domain_Online(systems[i]);   //ͬ����ʱ��Ҫ��Է������������ͬ������,������Ҫ�޸ģ�����ֻͬ��״̬���ݣ���Ҫͬ��ͬ����������
//						}
//					}
//					else
//					{
//						systems[i]->syncTimer = 0;
//					}
//
//					}
//					OSTimeDlyHMSM(0,0,0,100);
//		}
}

//��CAN����֡ѹ�����
void CAN_Post_Queue(u8 canChan,CAN_SEND_FRAME *frame)
{
    OS_CPU_SR cpu_sr;
    CAN_SEND_FRAME *tempMsgNode = NULL,*newMsgNode = NULL;
    OS_ENTER_CRITICAL();
    newMsgNode = (CAN_SEND_FRAME *)mymalloc(SRAMIN,sizeof(CAN_SEND_FRAME));
    if (newMsgNode != NULL)
    {
        OS_EXIT_CRITICAL();
        memcpy(newMsgNode,frame,sizeof(CAN_SEND_FRAME));
        if(canChan==CAN1_CHANNEL)
        {
            tempMsgNode = g_can1SendFrame;
        }
        else if(canChan==CAN2_CHANNEL)
        {
            tempMsgNode = g_can2SendFrame;
        }
        while (tempMsgNode->nextMsg != NULL)
        {
            tempMsgNode = tempMsgNode->nextMsg;
        }
        /*���������������*/
        tempMsgNode->nextMsg = newMsgNode;//��framָ��Ľ����ڶ��е����
        newMsgNode->nextMsg = NULL;
        if(canChan==CAN1_CHANNEL)
        {
            OSSemPost(can1SendSem);
        }
        else if(canChan==CAN2_CHANNEL)
        {
            OSSemPost(can2SendSem);
        }
    }
    else
    {
        OS_EXIT_CRITICAL();
    }
}

//CAN1����������Ϣ֡
void CAN1_Send_Frame(CAN_SEND_FRAME *frame)
{
    u8 err = 0;
    // u8 recvId;
    CAN_DATA_FRAME *frameBuf;
    bool sendRes;
    u8 cntRetry = 0;
    while(1)
    {
        sendRes = CAN_Send_Frame_App(CAN1_CHANNEL,frame->id,CAN_Id_Extended,&frame->canMsg,frame->len,frameBuf,CAN_INFO_RETRY_TIME);

        if(sendRes==false)
        {
            return;
        }
        //recvId = frame->id&0x000000ff;
//				SetReplyFrame(&can1InfoAckFrame,recvId,frame->canMsg.mainIndex,frame->canMsg.subIndex);
        OSSemPend(can1InfoAckSem,MAX_ACK_TIMEOUT,&err);
        //�ȴ�����
        if(err==OS_ERR_NONE)
        {
            memset(&can1InfoAckFrame,0,sizeof(ReplyFrame));
            return;
        }
        else if(err==OS_ERR_TIMEOUT)
        {
            cntRetry++;
            //  frame->id=frame->id|0x02000000;//�ط���־
            if(cntRetry>=MAX_RETRY_TIMES)
            {
                memset(&can1InfoAckFrame,0,sizeof(ReplyFrame));
                return;
            }
        }
        else
        {
            memset(&can1InfoAckFrame,0,sizeof(ReplyFrame));
            return;
        }
    }
}

//CAN2����������Ϣ֡
void CAN2_Send_Frame(CAN_SEND_FRAME *frame)
{
    u8 err = 0;
    //u8 recvId;
    CAN_DATA_FRAME *frameBuf;
    bool sendRes;
    u8 cntRetry = 0;
    while(1)
    {
        //2018.6.22 YQ�޸�,Ϊ�˷�ֹ���Ͳ��ɹ�������������ʱ���Ϊ1000
        sendRes = CAN_Send_Frame_App(CAN2_CHANNEL,frame->id,CAN_Id_Extended,&frame->canMsg,frame->len,frameBuf,CAN2_INFO_RETRY_TIME);
        //���ｫCAN2_Send_Frame���β���չCAN_Id_Extended֡����
        if(sendRes==false)
        {
            return;
        }
        //recvId = frame->id&0x000000ff;
//				SetReplyFrame(&can2InfoAckFrame,recvId,frame->canMsg.mainIndex,frame->canMsg.subIndex);
        OSSemPend(can2InfoAckSem,MAX_ACK_TIMEOUT,&err); 				//�ȴ�Ӧ���ź���
        /*wangkai:Ϊ�˷�����ԣ��˴����޵ȴ��ź���*/
        //OSSemPend(can2InfoAckSem,0,&err);        //�ȴ�Ӧ���ź���

        if(err==OS_ERR_NONE)//����Է���ʱ�ظ�
        {
            memset(&can2InfoAckFrame,0,sizeof(ReplyFrame));
            return;
        }
        else if(err==OS_ERR_TIMEOUT)//����Է�û�м�ʱ�ظ����ط�3��
        {
            cntRetry++;
            //frame->id=frame->id|0x02000000;//�ط���־

            if(cntRetry>=MAX_RETRY_TIMES)
            {
                memset(&can2InfoAckFrame,0,sizeof(ReplyFrame));
							  printf("Ŀ�������Ժ�û�лظ���\r\n");
                return;
            }
        }
        else
        {
            memset(&can2InfoAckFrame,0,sizeof(ReplyFrame));
            return;
        }
    }
}

u32 CAN_FramID_Change(CAN_DATA_FRAME *frameBuf)
{

    u32 id;
    // u8 device_id;
    u8 sendId;
    u8 recvId;
    u8 index;
    CAN_DATA_FRAME * tempNode;

    index=tempNode->id.idBit.index;
    sendId = tempNode->id.idBit.recvDeviceId;
    recvId = tempNode->id.idBit.sendDeviceId;
    // device_id = tempNode->id.idBit.sendDeviceId;
    id = Get_CAN_ExId(FRAME_REPLY,sendId,recvId)|index;
    tempNode->id.idBit.ackBit = FRAME_REPLY;
    return id;
}


/*******************************************************************************
** ��������: CAN2_Single_Send
** ��������: CAN��֡���� ��Ϊ����������
** ����˵��: tempNode: [����/��]
** ����˵��: None
** ������Ա: ����
** ��������: 2019-06-28
********************************************************************************/
void CAN2_Single_Send(CAN_DATA_FRAME * tempNode)
{
    INT32U id;
    bool sendRes;//can֡�����Ƿ�ɹ�
    INT16U MSindex;
    //INT8U device_id;
    CAN_DATA_FRAME *frameBuf;
    /*�ڶ��������ʱ��*/
    if(tempNode->id.idBit.MasterslaveBit==ManyMaster)//�������֡�Ƕ����ķ�������
    {
        id=CAN_FramID_Change(tempNode)|(ManyMaster<<28);//
        CAN_Send_Frame_App(CAN2_CHANNEL,id,CAN_Id_Extended,&tempNode->canMsg,tempNode->dataLen,frameBuf,CAN_ACK_RETRY_TIME);
    }
    else if(tempNode->id.idBit.MasterslaveBit==MasterSlave)//���������
    {
        HeartBeat.BeatTime=0;
        /*��������֡���͵�ʱ�򣬾Ͱ�����֡��ʱ����*/

        MSindex=(tempNode->id.idBit.index+(tempNode->id.idBit.sendDeviceId<<8));//����ģʽ�µ�������
        //device_id=tempNode->id.idBit.recvDeviceId;//�豸�ž��Ǳ������豸�ţ�Ҳ���ǽ��ܵ���֡�Ľ��յ�ַ��
        id=(MasterSlave<<28)+(FRAME_REPLY<<26)+(ThisTransitionNumber<<16)+MSindex;//������־0//Ӧ��֡��־0//��id//��������
        sendRes = CAN_Send_Frame_App(CAN2_CHANNEL,id,CAN_Id_Extended,&tempNode->canMsg,tempNode->dataLen,frameBuf,CAN_ACK_RETRY_TIME);
//        if(sendRes!=true&&TransStatus.DeviceMode!=standAlone)//����ֵ������˵��can����������,������ѻ�״̬���Ͳ���������
//        {
//            TransStatus.DeviceMode=OffLine;//can֡������ȥ��˵��������
//        }
//        else
//        {
//            TransStatus.DeviceMode=1;
//        }

    }
    else return;
}
