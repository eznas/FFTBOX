#include "max7219.h"

#if SPI_IS_HARDWARE  //判断SPI方式

void Max7219_GPIO_Init(void)    //硬件SPI
{
    SPI_InitTypeDef  SPI1_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
//    ADC_InitTypeDef ADC_InitStructure;

    //配置SPI1管脚
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

//    RCC_ADCCLKConfig(RCC_PCLK2_Div8); //设置ADC时钟分频为6分频
//    ADC_Cmd(ADC1, ENABLE); //使能ADC1
//
//    ADC_ResetCalibration(ADC1); //重置ADC1校准寄存器。
//    while(ADC_GetResetCalibrationStatus(ADC1)); //等待复位校准结束
//    ADC_StartCalibration(ADC1); //开启AD校准
//    while(ADC_GetCalibrationStatus(ADC1));      //等待校准结束
//
//    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   //配置ADC为独立模式
//    ADC_InitStructure.ADC_ScanConvMode = DISABLE;        //设置在单通道模式下执行转换
//    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  //设置在单次模式下执行转换
//    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //设置转换不是由外部触发启动
//    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //设置ADC数据右对齐
//    ADC_InitStructure.ADC_NbrOfChannel = 1;                //顺序进行规则转换的ADC通道的数目
//    ADC_InitStructure.ADC_Pga = ADC_Pga_4;
//    ADC_InitStructure.ADC_OutputBuffer= ADC_OutputBuffer_Enable;
//
//    ADC_Init(ADC1, &ADC_InitStructure);                    //根据ADC_InitStructure中指定的参数初始化ADC1寄存器

    SPI1_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
    SPI1_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16;  //64
    SPI1_InitStructure.SPI_DataSize=SPI_DataSize_8b;
    SPI1_InitStructure.SPI_Mode=SPI_Mode_Master;
    SPI1_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB; //高位在先
    SPI1_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI1_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI1_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI1_InitStructure.SPI_CRCPolynomial = 7;
    SPI_I2S_DeInit(SPI1);
    SPI_Init(SPI1, &SPI1_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
}

//功能：向MAX7219内部寄存器写入数据
//参数：addr、dat
void Write_Max7219(u8 addr,u8 dat)
{
    GPIO_ResetBits(GPIOA, Max7219_pinCS);
    SPI_I2S_SendData(SPI1, addr);  //写入地址
    Delay_Us(4);//必须，点阵不显示时可以延长此延时调试
    SPI_I2S_SendData(SPI1, dat);        //写入数据
    Delay_Us(4);//必须，点阵不显示时可以延长此延时调试
    GPIO_SetBits(GPIOA, Max7219_pinCS) ;
}

#else

void Max7219_GPIO_Init(void)//软件模拟SPI
{
     GPIO_InitTypeDef GPIO_InitStructure;

     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Pin  = Max7219_pinCS_GPIO | Max7219_pinDIN_GPIO | Max7219_pinCLK_GPIO;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_Init(GPIOA,&GPIO_InitStructure);
}

//功能：向MAX7219写入字节
//入口参数：data
void Write_Max7219_byte(u8 data)
{
    u8 i;
    GPIO_ResetBits(GPIOA, Max7219_pinCS);
    for(i = 0;i<8;i++)
    {
        GPIO_ResetBits(GPIOA, Max7219_pinCLK);
        if((data & 0x80) == 0)//写成Max7219_pinDIN = data & 0x80;实测无效，Max7219_pinDIN一直是低电平
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

//功能：向MAX7219内部寄存器写入数据
//参数：addr、dat
void Write_Max7219(u8 addr,u8 dat)
{
//   Max7219_pinCS = 0;
    GPIO_ResetBits(GPIOA, Max7219_pinCS);
   Write_Max7219_byte(addr);           //写入地址
     //Delay_Us(10);//模拟SPI时可以去除
   Write_Max7219_byte(dat);               //写入数据
     //Delay_Us(10);
//   Max7219_pinCS = 1;
   GPIO_SetBits(GPIOA, Max7219_pinCS);
}
#endif

void Init_MAX7219(void)
{
//    Write_Max7219(12, 0);
     Write_Max7219(0x09, 0x00);       //译码方式：无
     Write_Max7219(0x0a, 0x00);       //亮度
     Write_Max7219(0x0b, 0x07);       //扫描界限；8个数码管显示
     Write_Max7219(0x0c, 0x01);       //掉电模式：0，普通模式：1
     Write_Max7219(0x0f, 0x00);       //显示测试：1；测试结束，正常显示：0
}
