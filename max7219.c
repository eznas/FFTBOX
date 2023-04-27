#include "max7219.h"

#if SPI_IS_HARDWARE  //�ж�SPI��ʽ

void Max7219_GPIO_Init(void)    //Ӳ��SPI
{
    SPI_InitTypeDef  SPI1_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
//    ADC_InitTypeDef ADC_InitStructure;

    //����SPI1�ܽ�
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

    GPIO_InitStructure.GPIO_Pin=Max7219_pinCS_GPIO;//CS/LOAD
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=Max7219_pinCLK_GPIO|SPI_MISO_GPIO|Max7219_pinDIN_GPIO;//CLK MISO MOSI
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

//
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    RCC_ADCCLKConfig(RCC_PCLK2_Div8); //����ADCʱ�ӷ�ƵΪ6��Ƶ
//    ADC_Cmd(ADC1, ENABLE); //ʹ��ADC1
//
//    ADC_ResetCalibration(ADC1); //����ADC1У׼�Ĵ�����
//    while(ADC_GetResetCalibrationStatus(ADC1)); //�ȴ���λУ׼����
//    ADC_StartCalibration(ADC1); //����ADУ׼
//    while(ADC_GetCalibrationStatus(ADC1));      //�ȴ�У׼����
//
//    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   //����ADCΪ����ģʽ
//    ADC_InitStructure.ADC_ScanConvMode = DISABLE;        //�����ڵ�ͨ��ģʽ��ִ��ת��
//    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  //�����ڵ���ģʽ��ִ��ת��
//    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //����ת���������ⲿ��������
//    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //����ADC�����Ҷ���
//    ADC_InitStructure.ADC_NbrOfChannel = 1;                //˳����й���ת����ADCͨ������Ŀ
//    ADC_InitStructure.ADC_Pga = ADC_Pga_4;
//    ADC_InitStructure.ADC_OutputBuffer= ADC_OutputBuffer_Enable;
//
//    ADC_Init(ADC1, &ADC_InitStructure);                    //����ADC_InitStructure��ָ���Ĳ�����ʼ��ADC1�Ĵ���

    SPI1_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
    SPI1_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16;  //64
    SPI1_InitStructure.SPI_DataSize=SPI_DataSize_8b;
    SPI1_InitStructure.SPI_Mode=SPI_Mode_Master;
    SPI1_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB; //��λ����
    SPI1_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI1_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI1_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI1_InitStructure.SPI_CRCPolynomial = 7;
    SPI_I2S_DeInit(SPI1);
    SPI_Init(SPI1, &SPI1_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
}

//���ܣ���MAX7219�ڲ��Ĵ���д������
//������addr��dat
void Write_Max7219(u8 addr,u8 dat)
{
    GPIO_ResetBits(GPIOA, Max7219_pinCS);
    SPI_I2S_SendData(SPI1, addr);  //д���ַ
    Delay_Us(4);//���룬������ʾʱ�����ӳ�����ʱ����
    SPI_I2S_SendData(SPI1, dat);        //д������
    Delay_Us(4);//���룬������ʾʱ�����ӳ�����ʱ����
    GPIO_SetBits(GPIOA, Max7219_pinCS) ;
}

#else

void Max7219_GPIO_Init(void)//���ģ��SPI
{
     GPIO_InitTypeDef GPIO_InitStructure;

     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Pin  = Max7219_pinCS_GPIO | Max7219_pinDIN_GPIO | Max7219_pinCLK_GPIO;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_Init(GPIOA,&GPIO_InitStructure);
}

//���ܣ���MAX7219д���ֽ�
//��ڲ�����data
void Write_Max7219_byte(u8 data)
{
    u8 i;
    GPIO_ResetBits(GPIOA, Max7219_pinCS);
    for(i = 0;i<8;i++)
    {
        GPIO_ResetBits(GPIOA, Max7219_pinCLK);
        if((data & 0x80) == 0)//д��Max7219_pinDIN = data & 0x80;ʵ����Ч��Max7219_pinDINһֱ�ǵ͵�ƽ
        {
            GPIO_ResetBits(GPIOA, Max7219_pinDIN);
        }
        else
        {
//            Max7219_pinDIN = 1;
            GPIO_SetBits(GPIOA, Max7219_pinDIN);
        }
        data = data<<1;
//        Max7219_pinCLK = 1;
        GPIO_SetBits(GPIOA, Max7219_pinCLK);
     }
}

//���ܣ���MAX7219�ڲ��Ĵ���д������
//������addr��dat
void Write_Max7219(u8 addr,u8 dat)
{
//   Max7219_pinCS = 0;
    GPIO_ResetBits(GPIOA, Max7219_pinCS);
   Write_Max7219_byte(addr);           //д���ַ
     //Delay_Us(10);//ģ��SPIʱ����ȥ��
   Write_Max7219_byte(dat);               //д������
     //Delay_Us(10);
//   Max7219_pinCS = 1;
   GPIO_SetBits(GPIOA, Max7219_pinCS);
}
#endif

void Init_MAX7219(void)
{
//    Write_Max7219(12, 0);
     Write_Max7219(0x09, 0x00);       //���뷽ʽ����
     Write_Max7219(0x0a, 0x00);       //����
     Write_Max7219(0x0b, 0x07);       //ɨ����ޣ�8���������ʾ
     Write_Max7219(0x0c, 0x01);       //����ģʽ��0����ͨģʽ��1
     Write_Max7219(0x0f, 0x00);       //��ʾ���ԣ�1�����Խ�����������ʾ��0
}
